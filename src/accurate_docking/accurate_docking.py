#!/usr/bin/env python
import re

import rospy
import actionlib
import math

from tf import TransformListener
from tf.transformations import euler_from_quaternion
from robotnik_navigation_msgs.msg import MoveAction, MoveGoal
from robotnik_navigation_msgs.msg import DockAction, DockGoal
from std_srvs.srv import Trigger, Empty
import os
import rospkg
import datetime


from rcomponent.rcomponent import *
import exceptions



class AccurateDocking(RComponent):
    """
    A class used to make a simple docking process
    """

    def __init__(self):
        RComponent.__init__(self)

        self.step = -1
        # array of {'initial': [x,y,theta], 'final': [x,y,theta]}
        self.results = []
        self.ongoing_result = {}
        self.total_iterations = 0
        self.stop = False


    def ros_read_params(self):
        """Gets params from param server"""
        RComponent.ros_read_params(self)

        self.dock_action_name = rospy.get_param('~dock_action_server', default = 'pp_docker')
        self.move_action_name = rospy.get_param('~move_action_server', default = 'move')

        self.robot_link = rospy.get_param('~base_frame', default = 'robot_base_link')

        self.pregoal_link = rospy.get_param('~pregoal_frame', default = 'laser_test_pregoal_footprint')
        self.goal_link = rospy.get_param('~goal_frame', default = 'laser_test_goal_footprint')
        self.reflectors_link = rospy.get_param('~reflectors_frame', default = 'robot_filtered_docking_station_laser')
        # array of [x, y, theta] for the first dock call
        param_offset_1 = rospy.get_param('~pregoal_offset_1', default = '[-0.5, 0, 0]')
        self.pregoal_offset_1 = (param_offset_1.replace('[','').replace(']','').replace(',','')).split(' ')
        
        if len(self.pregoal_offset_1)!= 3:
            self.pregoal_offset_1 = [0, 0, 0]
        else:
          for i in range(len(self.pregoal_offset_1)):
            self.pregoal_offset_1[i] = float(self.pregoal_offset_1[i])
        # array of [x, y, theta] for the second dock call
        param_offset_2 = rospy.get_param('~pregoal_offset_2', default = '[-0.0, 0, 0]')
        self.pregoal_offset_2 = (param_offset_2.replace('[','').replace(']','').replace(',','')).split(' ')
        
        if len(self.pregoal_offset_2)!= 3:
            self.pregoal_offset_2 = [0, 0, 0]
        else:
          for i in range(len(self.pregoal_offset_2)):
            self.pregoal_offset_2[i] = float(self.pregoal_offset_2[i])

        rospy.loginfo("%s::ros_read_params: docker offsets: %s - %s", rospy.get_name(), str(self.pregoal_offset_1), str(self.pregoal_offset_2))

        self.step_back_distance = rospy.get_param('~step_back_distance', default = 1.3)
        self.consecutive_iterations = rospy.get_param('~consecutive_iterations', default = 1)
        self.current_iteration = self.consecutive_iterations

    def ros_setup(self):
        """Creates and inits ROS components"""

        RComponent.ros_setup(self)

        self.tf = TransformListener()

        self.move_action_client = actionlib.SimpleActionClient(
            self.move_action_name, MoveAction)

        self.dock_action_client = actionlib.SimpleActionClient(
            self.dock_action_name, DockAction)

        self.start_docking_server = rospy.Service('~start', Trigger, self.start_docking_service_cb)
        self.stop_docking_server = rospy.Service('~stop', Trigger, self.stop_docking_service_cb)
        self.save_results_server = rospy.Service('~save_results', Empty, self.save_results_service_cb)


        return 0

    def ready_state(self):
        """Actions performed in ready state"""
        
        if self.current_iteration < self.consecutive_iterations:

          if self.step != -1:
            try:
              # Get relative goal position
              t = self.tf.getLatestCommonTime(
                  self.robot_link, self.goal_link)

              (position, quaternion) = self.tf.lookupTransform(
                  self.robot_link, self.goal_link, t)
              (_, _, orientation) = euler_from_quaternion(quaternion)
            except Exception, e:
              rospy.logerr("%s::ready_state: exception: %s", rospy.get_name(), e)
              self.stop()
              return

            try:
              # Get relative goal to the reflectors position
              t = self.tf.getLatestCommonTime(
                  self.robot_link, self.reflectors_link)

              (position_to_reflectors, quaternion_to_reflectors) = self.tf.lookupTransform(
                  self.robot_link, self.reflectors_link, t)
              (_, _, orientation_to_reflectors) = euler_from_quaternion(quaternion_to_reflectors)

            except Exception, e:
              rospy.logerr("%s::ready_state: exception: %s", rospy.get_name(), e)
              error_on_action()
              return

          if self.step == 0:
            self.ongoing_result = {}
            rospy.loginfo('%s::ros_setup: waiting for server %s', rospy.get_name(), self.move_action_name)
            self.move_action_client.wait_for_server()
            rospy.loginfo('%s::ros_setup: waiting for server %s', rospy.get_name(), self.dock_action_name)
            self.dock_action_client.wait_for_server()

            rospy.loginfo('%s::ready_state: Initial distance to goal-> x: %.3lf mm, y: %.3lf mm, %.3lf degrees', rospy.get_name(), position[0]*1000, position[1]*1000, math.degrees(orientation))
            self.ongoing_result['initial'] = [position[0], position[1], math.degrees(orientation)]
            # Docking
            dock_goal = DockGoal()
            dock_goal.dock_frame = self.pregoal_link
            dock_goal.robot_dock_frame = self.robot_link
            dock_goal.dock_offset.x = self.pregoal_offset_1[0]
            dock_goal.dock_offset.y = self.pregoal_offset_1[1]
            dock_goal.dock_offset.theta = self.pregoal_offset_1[2]
            rospy.loginfo('%s::ready_state: %d - docking to %s - offset = %s', rospy.get_name(), self.step, self.pregoal_link, str(dock_goal.dock_offset))
            self.dock_action_client.send_goal(dock_goal)
            self.dock_action_client.wait_for_result()
            result = self.dock_action_client.get_result()
            rospy.loginfo('%s::ready_state: %d - result = %s', rospy.get_name(), self.step, str(result.success))

            if result.success == True:
              self.step = 1
              rospy.sleep(2)
            else:
              rospy.logerr("%s::ready_state: Docking failed", rospy.get_name())
              error_on_action()

          elif self.step == 1:

            # Docking
            dock_goal = DockGoal()
            dock_goal.dock_frame = self.pregoal_link
            dock_goal.robot_dock_frame = self.robot_link
            dock_goal.dock_offset.x = self.pregoal_offset_2[0]
            dock_goal.dock_offset.y = self.pregoal_offset_2[1]
            dock_goal.dock_offset.theta = self.pregoal_offset_2[2]
            rospy.loginfo('%s::ready_state: %d - docking to %s - offset = %s', rospy.get_name(), self.step, self.pregoal_link, str(dock_goal.dock_offset))
            self.dock_action_client.send_goal(dock_goal)
            self.dock_action_client.wait_for_result()
            result = self.dock_action_client.get_result()
            rospy.loginfo('%s::ready_state: %d - result = %s', rospy.get_name(), self.step, str(result.success))

            if result.success == True:
              self.step = 2
            else:
              rospy.logerr("%s::ready_state: Docking failed", rospy.get_name())
              error_on_action()

          elif self.step == 2:
              # Orientate robot
              move_goal = MoveGoal()
              move_goal.goal.theta = orientation
              if abs(math.degrees(orientation)) > 0.2:
                rospy.loginfo('%s::ready_state: %d - rotating %.3lf degrees to %s', rospy.get_name(), self.step, math.degrees(orientation), self.goal_link)
                self.move_action_client.send_goal(move_goal)
                self.move_action_client.wait_for_result()
                result = self.move_action_client.get_result()
                rospy.loginfo('%s::ready_state: %d - result = %s', rospy.get_name(), self.step, str(result.success))

                if result.success == True:
                  self.step = 3
                else:
                  rospy.logerr("%s::ready_state: Move-Rotation failed", rospy.get_name())
                  error_on_action()
              else:
                rospy.loginfo('%s::ready_state: %d - avoids rotating %.3lf degrees to %s', rospy.get_name(), self.step, math.degrees(orientation), self.goal_link)
                self.step = 3

          elif self.step == 3:
            # Move forward
            move_goal = MoveGoal()
            move_goal.goal.x = position[0]
            rospy.loginfo('%s::ready_state: %d - moving forward %.3lf meters to %s', rospy.get_name(), self.step, move_goal.goal.x, self.goal_link)
            self.move_action_client.send_goal(move_goal)
            self.move_action_client.wait_for_result()
            result = self.move_action_client.get_result()
            rospy.loginfo('%s::ready_state: %d - result = %s', rospy.get_name(), self.step, str(result.success))

            if result.success == True:
              rospy.sleep(2)
              self.step = 4
            else:
              rospy.logerr("%s::ready_state: Move-Forward failed", rospy.get_name())
              error_on_action()

          elif self.step == 4:
            rospy.loginfo('%s::ready_state: final distance to goal -> x: %.3lf mm, y: %.3lf mm, %.3lf degrees', rospy.get_name(), position[0]*1000, position[1]*1000, math.degrees(orientation))
            rospy.loginfo('%s::ready_state: final distance to reflectors -> x: %.3lf mm, y: %.3lf mm, %.3lf degrees', rospy.get_name(), position_to_reflectors[0]*1000, position_to_reflectors[1]*1000, math.degrees(orientation_to_reflectors))
            self.ongoing_result['final'] = [position[0], position[1], math.degrees(orientation)]
            self.results.append(self.ongoing_result)
            self.step = 5

          elif self.step == 5:
            # Move backward
            move_goal = MoveGoal()
            move_goal.goal.x = -self.step_back_distance
            rospy.loginfo('%s::ready_state: %d - moving backwards %.3lf meters', rospy.get_name(), self.step, move_goal.goal.x)
            self.move_action_client.send_goal(move_goal)
            self.move_action_client.wait_for_result()
            result = self.move_action_client.get_result()
            rospy.loginfo('%s::ready_state: %d - result = %s', rospy.get_name(), self.step, str(result.success))

            if result.success == True:
              rospy.sleep(2)
              self.iteration_success()
            else:
              rospy.logerr("%s::ready_state: Move-Forward failed", rospy.get_name())
              self.error_on_action()

   
    def error_on_action(self):
      self.step = -1
      self.current_iteration = self.consecutive_iterations

    def iteration_success(self):
      self.current_iteration+=1
      self.total_iterations+= 1
      rospy.loginfo('%s::iteration_success: current it = %d - total = %s', rospy.get_name(), self.current_iteration, self.total_iterations)

      if self.stop == True:
        self.stop = False
        if self.current_iteration < self.consecutive_iterations:
          rospy.logwarn('%s::iteration_success: stopping due to user requirement', rospy.get_name())
          self.current_iteration = self.consecutive_iterations

      if self.current_iteration < self.consecutive_iterations:
        self.step = 0
      else:
        self.step = -1
      
      

    def shutdown(self):

        return RComponent.shutdown(self)

    def switch_to_state(self, new_state):
        """Performs the change of state"""

        return RComponent.switch_to_state(self, new_state)

    def stop_docking_service_cb(self, req):
      '''
        stop docking process
      '''

      if self.current_iteration < self.consecutive_iterations:
        self.stop = True
        return True, "Stopping after this iteration (%d of %d) Step = %d"%(self.current_iteration+1, self.consecutive_iterations, self.step)
      else:
        return False, "Docking not running"


    def start_docking_service_cb(self, req):
      '''
        start docking process
      '''

      if self.current_iteration < self.consecutive_iterations:
        return False, "Docking is still running. Iteration = (%d of %d) Step = %d"%(self.current_iteration+1, self.consecutive_iterations, self.step)
      else:
        self.current_iteration = 0
        self.step = 0
        self.stop = False
        return True, "Starting"

    def save_results_service_cb(self, req):
      '''
        saves current results process
      '''
      rp = rospkg.RosPack()
      filename = 'docking_%s.csv'%(str(datetime.datetime.now()).replace(' ', '_').replace('.', '').replace(':', '-'))

      folder = os.path.join(rp.get_path('accurate_docking'), 'data')
      file_path = folder+'/'+filename
      with open(file_path, 'w') as f:
        for result in self.results:
            f.write("%f,%f,%f,%f,%f,%f\n"%(result['initial'][0], result['initial'][1], result['initial'][2],result['final'][0], result['final'][1], result['final'][2] ))

      print self.results

      return []
