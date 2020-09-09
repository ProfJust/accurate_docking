#!/usr/bin/env python
import re

import rospy
import actionlib
import math
import time

from tf import TransformListener, transformations
from tf.transformations import euler_from_quaternion
from robotnik_navigation_msgs.msg import MoveAction, MoveGoal
from robotnik_navigation_msgs.msg import DockAction, DockGoal
from robotnik_msgs.srv import SetTransform
from robotnik_msgs.msg import ReturnMessage
from std_srvs.srv import Trigger, Empty
from std_msgs.msg import String
import os
import rospkg
import datetime
from move_base_msgs.msg import * 
from geometry_msgs.msg import PoseStamped
from actionlib_msgs.msg import GoalStatus

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
        self.move_base_action_name = rospy.get_param('~move_base_action_server', default = 'move_base')

        self.robot_link = rospy.get_param('~base_frame', default = 'robot_base_link')
        self.only_docker = rospy.get_param('~only_docker', default = False)

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
        param_offset_2 = rospy.get_param('~pregoal_offset_2', default = '[-0.1, 0, 0]')
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
        self.idle_time_between_transitions = rospy.get_param('~idle_time_between_transitions', default = 2)
        self.global_frame = rospy.get_param('~global_frame', default = 'robot_map')

        self.use_move_base = False
        # it will use move_base as long as the param init_pose is defined (correctly)
        if rospy.has_param('~init_pose'):
          init_pose = rospy.get_param('~init_pose', default = '[0, 0, 0]')
          self.init_pose = (init_pose.replace('[','').replace(']','').replace(',','')).split(' ')
        
          if len(self.init_pose)!= 3:
              rospy.logerr("%s::ros_read_params: incorrect format of init_pose", rospy.get_name())
          else:
              for i in range(len(self.init_pose)):
                  self.init_pose[i] = float(self.init_pose[i])
              self.use_move_base = True
              rospy.loginfo("%s::ros_read_params: using init_pose via move_base actions", rospy.get_name())
              self.init_pose_goal = PoseStamped()
              self.init_pose_goal.header.frame_id = self.global_frame
              self.init_pose_goal.pose.position.x = self.init_pose[0]
              self.init_pose_goal.pose.position.y = self.init_pose[1]
              quaternion = transformations.quaternion_from_euler(0, 0, self.init_pose[2])
              self.init_pose_goal.pose.orientation.x = quaternion[0]
              self.init_pose_goal.pose.orientation.y = quaternion[1]
              self.init_pose_goal.pose.orientation.z = quaternion[2]
              self.init_pose_goal.pose.orientation.w = quaternion[3]

    def ros_setup(self):
        """Creates and inits ROS components"""

        RComponent.ros_setup(self)

        self.tf = TransformListener()

        self.move_action_client = actionlib.SimpleActionClient(
            self.move_action_name, MoveAction)

        self.dock_action_client = actionlib.SimpleActionClient(
            self.dock_action_name, DockAction)
        
        if self.use_move_base:
            self.move_base_action_client = actionlib.SimpleActionClient(self.move_base_action_name, MoveBaseAction)

        self.start_docking_server = rospy.Service('~start', Trigger, self.start_docking_service_cb)
        self.stop_docking_server = rospy.Service('~stop', Trigger, self.stop_docking_service_cb)
        self.save_results_server = rospy.Service('~save_results', Empty, self.save_results_service_cb)

        self.set_pregoal_offset_1 = rospy.Service('~set_pregoal_offset_1', SetTransform, self.set_pregoal_offset_1_cb)
        self.set_pregoal_offset_2 = rospy.Service('~set_pregoal_offset_2', SetTransform, self.set_pregoal_offset_2_cb)

        self.docking_status_pub = rospy.Publisher('~status', String, queue_size=10)
        self.docking_status = "stopped"


        RComponent.ros_setup(self)

    def ros_publish(self):
      '''
        Publish topics at standard frequency
      '''

      self.docking_status_pub.publish(self.docking_status)

      RComponent.ros_publish(self)

    def ready_state(self):
        """Actions performed in ready state"""
        
        if self.current_iteration < self.consecutive_iterations:

          self.docking_status = "running"

          if self.step != -1:
            try:
              # Get relative goal position
              t = self.tf.getLatestCommonTime(
                  self.robot_link, self.goal_link)

              (position, quaternion) = self.tf.lookupTransform(
                  self.robot_link, self.goal_link, t)
              (_, _, orientation) = euler_from_quaternion(quaternion)
            except Exception as e:
              rospy.logerr("%s::ready_state: exception: %s", rospy.get_name(), e)
              self.docking_status = "error"
              self.stop = True
              return

            try:
              # Get relative goal to the reflectors position
              t = self.tf.getLatestCommonTime(
                  self.robot_link, self.reflectors_link)

              (position_to_reflectors, quaternion_to_reflectors) = self.tf.lookupTransform(
                  self.robot_link, self.reflectors_link, t)
              (_, _, orientation_to_reflectors) = euler_from_quaternion(quaternion_to_reflectors)

            except Exception as e:
              rospy.logerr("%s::ready_state: exception: %s", rospy.get_name(), e)
              self.docking_status = "error"
              self.error_on_action()
              return

          if self.step == 0:
            self.ongoing_result = {}
            rospy.loginfo('%s::ros_setup: waiting for server %s', rospy.get_name(), self.move_action_name)
            if self.move_action_client.wait_for_server(timeout = rospy.Duration(5.0)) == False:
              rospy.logerr("%s::ready_state: timeout waiting", rospy.get_name())
              self.error_on_action()
              self.docking_status = "error"
              return
            
            rospy.loginfo('%s::ros_setup: waiting for server %s', rospy.get_name(), self.dock_action_name)
            if self.dock_action_client.wait_for_server(timeout = rospy.Duration(5.0)) == False:
              rospy.logerr("%s::ready_state: timeout waiting", rospy.get_name())
              self.error_on_action()
              self.docking_status = "error"
              return
 
            if self.use_move_base:
              rospy.loginfo('%s::ros_setup: waiting for server %s', rospy.get_name(), self.move_base_action_name)
              if self.move_base_action_client.wait_for_server(timeout = rospy.Duration(5.0)) == False:
                rospy.logerr("%s::ready_state: timeout waiting", rospy.get_name())
                self.error_on_action()
                self.docking_status = "error"
                return

            rospy.loginfo('%s::ready_state: Initial distance to goal-> x: %.3lf mm, y: %.3lf mm, %.3lf degrees', rospy.get_name(), position[0]*1000, position[1]*1000, math.degrees(orientation))
            self.initial_time = time.time()
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
            rospy.loginfo('%s::ready_state: %d - result = %s. Elapsed time = %.1lf', rospy.get_name(), self.step, str(result.success), time.time() - self.initial_time)

            if result.success == True:
              self.step = 1
              rospy.sleep(self.idle_time_between_transitions)
            else:
              rospy.logerr("%s::ready_state: Docking failed", rospy.get_name())
              self.docking_status = "error"
              self.error_on_action()

          elif self.step == 1:
            if self.pregoal_offset_2 == self.pregoal_offset_1: # ignore second goal 
              if self.only_docker:
                self.step = 4
                rospy.loginfo('%s::ready_state: avoid step 1,2,3 . Elapsed time = %.1lf', rospy.get_name(), time.time() - self.initial_time)
              else:
                self.step = 2
                rospy.loginfo('%s::ready_state: avoid step 1. Elapsed time = %.1lf', rospy.get_name(), time.time() - self.initial_time)
              
            else:
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
              rospy.loginfo('%s::ready_state: %d - result = %s. Elapsed time = %.1lf', rospy.get_name(), self.step, str(result.success), time.time() - self.initial_time)

              if result.success == True:
                rospy.sleep(self.idle_time_between_transitions)
                if self.only_docker:
                  self.step = 4
                  rospy.loginfo('%s::ready_state: avoid step 1,2,3. Elapsed time = %.1lf', rospy.get_name(), time.time() - self.initial_time)
                else:
                  self.step = 2
              else:
                rospy.logerr("%s::ready_state: Docking failed", rospy.get_name())
                self.error_on_action()

          elif self.step == 2:
              # Orientate robot
              move_goal = MoveGoal()
              move_goal.goal.theta = orientation
              if abs(math.degrees(orientation)) > 0.2:
                rospy.loginfo('%s::ready_state: %d - rotating %.3lf degrees to %s', rospy.get_name(), self.step, math.degrees(orientation), self.goal_link)
                self.move_action_client.send_goal(move_goal)
                self.move_action_client.wait_for_result()
                result = self.move_action_client.get_result()
                rospy.loginfo('%s::ready_state: %d - result = %s. Elapsed time = %.1lf', rospy.get_name(), self.step, str(result.success), time.time() - self.initial_time)

                if result.success == True:
                  self.step = 3
                else:
                  rospy.logerr("%s::ready_state: Move-Rotation failed", rospy.get_name())
                  self.error_on_action()
                  self.docking_status = "error"
              else:
                rospy.loginfo('%s::ready_state: %d - avoids rotating %.3lf degrees to %s. Elapsed time = %.1lf', rospy.get_name(), self.step, math.degrees(orientation), self.goal_link, time.time() - self.initial_time)
                self.step = 3

          elif self.step == 3:
            # Move forward
            move_goal = MoveGoal()
            move_goal.goal.x = position[0]
            rospy.loginfo('%s::ready_state: %d - moving forward %.3lf meters to %s', rospy.get_name(), self.step, move_goal.goal.x, self.goal_link)
            self.move_action_client.send_goal(move_goal)
            self.move_action_client.wait_for_result()
            result = self.move_action_client.get_result()
            rospy.loginfo('%s::ready_state: %d - result = %s. Elapsed time = %.1lf', rospy.get_name(), self.step, str(result.success), time.time() - self.initial_time)

            if result.success == True:
              rospy.sleep(self.idle_time_between_transitions)
              self.step = 4
            else:
              rospy.logerr("%s::ready_state: Move-Forward failed", rospy.get_name())
              self.error_on_action()
              self.docking_status = "error"

          elif self.step == 4:
            self.final_time = time.time()
            self.ongoing_result['final'] = [position[0], position[1], math.degrees(orientation)]
            self.ongoing_result['time'] = self.final_time - self.initial_time
            rospy.logwarn('%s::ready_state: Error distance to goal -> x: %.3lf mm, y: %.3lf mm, %.3lf degrees. Elapsed time = %.1lf seconds', rospy.get_name(), position[0]*1000, position[1]*1000, math.degrees(orientation), self.ongoing_result['time'])
            rospy.loginfo('%s::ready_state: final distance to reflectors -> x: %.3lf mm, y: %.3lf mm, %.3lf degrees', rospy.get_name(), position_to_reflectors[0]*1000, position_to_reflectors[1]*1000, math.degrees(orientation_to_reflectors))
            
            self.results.append(self.ongoing_result)
            if self.step_back_distance == 0.0:
              self.iteration_success()
            else:
              self.step = 5

          elif self.step == 5:
            # Move backward
            if self.use_move_base == True:
              if self.move_base_action_client.wait_for_server(timeout = rospy.Duration(3.0) ):
                rospy.loginfo('%s::ready_state: %d - moving backwards via move_base to %.3lf, %.3lf, %.3lf', rospy.get_name(), self.step, self.init_pose[0], self.init_pose[1], self.init_pose[2])
                goal = MoveBaseGoal()				
                #set goal
                goal.target_pose = self.init_pose_goal			
                self.move_base_action_client.send_goal(goal)
                self.move_base_action_client.wait_for_result()
                result = ( self.move_base_action_client.get_state() == GoalStatus.SUCCEEDED)
                rospy.loginfo('%s::ready_state: %d - result = %d', rospy.get_name(), self.step, result)
                
                if result == True:
                  rospy.sleep(self.idle_time_between_transitions)
                  self.iteration_success()
                else:
                  rospy.logerr("%s::ready_state: Move-Forward failed", rospy.get_name())
                  self.error_on_action()
                  self.docking_status = "error"

              else:
                rospy.logerr("%s::ready_state: error waiting for move_base", rospy.get_name())
                self.error_on_action()
                self.docking_status = "error"
            else:
              move_goal = MoveGoal()
              move_goal.goal.x = -self.step_back_distance
              rospy.loginfo('%s::ready_state: %d - moving backwards %.3lf meters', rospy.get_name(), self.step, move_goal.goal.x)
              self.move_action_client.send_goal(move_goal)
              self.move_action_client.wait_for_result()
              result = self.move_action_client.get_result()
              rospy.loginfo('%s::ready_state: %d - result = %s', rospy.get_name(), self.step, str(result.success))

              if result.success == True:
                rospy.sleep(self.idle_time_between_transitions)
                self.iteration_success()
              else:
                rospy.logerr("%s::ready_state: Move-Forward via move_base failed", rospy.get_name())
                self.error_on_action()
                self.docking_status = "error"

        else:
          self.docking_status = "stopped"

   
    def error_on_action(self):
      self.step = -1
      self.current_iteration = self.consecutive_iterations

    def iteration_success(self):
      self.current_iteration+=1
      self.total_iterations+= 1
      rospy.logwarn('%s::iteration_success: current it = %d - total = %s', rospy.get_name(), self.current_iteration, self.total_iterations)

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
        f.write("#x0,y0,theta0,x1,y1,theta1,time\n")
        for result in self.results:
            f.write("%f,%f,%f,%f,%f,%f,%f\n"%(result['initial'][0], result['initial'][1], result['initial'][2],result['final'][0], result['final'][1], result['final'][2], result['time'] ))

      print(self.results)
      return []

    def set_pregoal_offset_1_cb(self, req):
      '''
        modify pregoal_offset_1
      '''
      self.pregoal_offset_1[0] = float(req.tf.translation.x)
      self.pregoal_offset_1[1] = float(req.tf.translation.y)
      #self.pregoal_offset_1[2] = req.tf.rotation

      ret = ReturnMessage()
      ret.success = True
      ret.message = "Changed pregoal_offset_1"

      return ret

    def set_pregoal_offset_2_cb(self, req):
      '''
        modify pregoal_offset_2
      '''
      self.pregoal_offset_2[0] = float(req.tf.translation.x)
      self.pregoal_offset_2[1] = float(req.tf.translation.y)
      #self.pregoal_offset_2[2] = req.tf.rotation

      ret = ReturnMessage()
      ret.success = True
      ret.message = "Changed pregoal_offset_2"

      return ret
