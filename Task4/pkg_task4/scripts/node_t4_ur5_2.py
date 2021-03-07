#!/usr/bin/env python

from __future__ import print_function
from six.moves import input

import sys
import copy
import rospy
import time
import moveit_commander
import moveit_msgs.msg
import geometry_msgs.msg
import rospkg
import yaml
import os
import actionlib
import math         
import threading


from std_msgs.msg import String
from collections import deque
from geometry_msgs.msg import Pose
from moveit_commander.conversions import pose_to_list
from std_srvs.srv import Empty
from pkg_vb_sim.srv import *
from pkg_vb_sim.msg import LogicalCameraImage
from pkg_task4.msg import ConveyorPkgs


#To activate conveyor in gazebo(server in pkg_vb_sim - node_service_server_conveyor_belt):-
def activate_conveyor_client(power, xyz):
    rospy.wait_for_service('/eyrc/vb/conveyor/set_power')
    try:
      activate_conveyor = rospy.ServiceProxy('/eyrc/vb/conveyor/set_power', conveyorBeltPowerMsg)
      resp1 = activate_conveyor(power)
      print("power:" + str(power) + " in gazebo")
      return resp1.result
    except rospy.ServiceException as e:
      print("Conveyor Service call failed: %s"%e)


#To activate gripper in gazebo(client in pkg_vb_sim):-
def activate_gripper_client(activate_vacuum_gripper):
    rospy.wait_for_service('/eyrc/vb/ur5/activate_vacuum_gripper/ur5_2')
    try:
      activate_gripper = rospy.ServiceProxy('/eyrc/vb/ur5/activate_vacuum_gripper/ur5_2', vacuumGripper)
      resp1 = activate_gripper(activate_vacuum_gripper)
      print("activate_vacuum_gripper:" + str(activate_vacuum_gripper) + " in gazebo")
      return resp1.result
    except rospy.ServiceException as e:
      print("UR5_2 Vaccume_Gripper Service call failed: %s"%e)  

def usage():
    return "[]"

q_ur5_2 = []
count = int(0)

class UR5_2(object):
  def __init__(self):
    super(UR5_2, self).__init__()
    
    ################################ IMPORTED ######################################
    # self._robot_ns = '/'  + arg_robot_name
    self._robot_ns = '/ur5_2'
    
    self._commander = moveit_commander.roscpp_initialize(sys.argv)
    self._robot = moveit_commander.RobotCommander(robot_description= self._robot_ns + "/robot_description", ns=self._robot_ns)
    self._scene = moveit_commander.PlanningSceneInterface(ns=self._robot_ns)
    self._group = moveit_commander.MoveGroupCommander("manipulator", robot_description= self._robot_ns + "/robot_description", ns=self._robot_ns)
    self._display_trajectory_publisher = rospy.Publisher( self._robot_ns + '/move_group/display_planned_path', moveit_msgs.msg.DisplayTrajectory, queue_size=1)
    self._exectute_trajectory_client = actionlib.SimpleActionClient( self._robot_ns + '/execute_trajectory', moveit_msgs.msg.ExecuteTrajectoryAction)
    self._exectute_trajectory_client.wait_for_server()

    self._planning_frame = self._group.get_planning_frame()
    self._eef_link = self._group.get_end_effector_link()
    self._group_names = self._robot.get_group_names()
    self._box_name = 'kachra'

    self._group.set_planning_time(99)

    # Attribute to store computed trajectory by the planner 
    self._computed_plan = ''

    # Current State of the Robot is needed to add box to planning scene
    self._curr_state = self._robot.get_current_state()

    rp = rospkg.RosPack()
    self._pkg_path = rp.get_path('pkg_task4')
    self._file_path = self._pkg_path + '/config/ur5_2_trajectories/'
    
    ####################################################################################

     # pose and name of box visible in logical_camera:- 
    self.box_name = ""
    self.camera_to_box_pose = geometry_msgs.msg.Pose() # position of box in reference of camera
    self.world_to_camera_pose = geometry_msgs.msg.Pose() # position  of camera in reference of world
    self.world_to_box_pose = geometry_msgs.msg.Pose()  # position of box in reference of world
    self.camera_to_ur5_2_pose = geometry_msgs.msg.Pose()
    self.is_fill = False

    # home_pose co-ordinates:-
    self.home_pose = geometry_msgs.msg.Pose()
    self.home_pose.position.x = -0.8
    self.home_pose.position.y = 0
    self.home_pose.position.z = 1 + 0.19
    # This to keep EE parallel to Ground Plane
    self.home_pose.orientation.x = -0.5
    self.home_pose.orientation.y = -0.5
    self.home_pose.orientation.z = 0.5
    self.home_pose.orientation.w = 0.5

    # home_pose                      
    self.lst_joint_angles_home_pose = [math.radians(7.84328385087),
                          math.radians(-139.933201191),
                          math.radians(-58.3065131143),
                          math.radians(-71.7219273039),
                          math.radians(89.9554380813),
                          math.radians(7.89480500776)]

  def clear_octomap(self):
      clear_octomap_service_proxy = rospy.ServiceProxy(self._robot_ns + "/clear_octomap", Empty)
      return clear_octomap_service_proxy()

  def ee_cartesian_translation(self, trans_x, trans_y, trans_z):
      waypoints = []
      waypoints.append(self._group.get_current_pose().pose)
      wpose = geometry_msgs.msg.Pose()
      wpose.position.x = waypoints[0].position.x + (trans_x)  
      wpose.position.y = waypoints[0].position.y + (trans_y)  
      wpose.position.z = waypoints[0].position.z + (trans_z)
      
      wpose.orientation.x = -0.5
      wpose.orientation.y = -0.5
      wpose.orientation.z = 0.5
      wpose.orientation.w = 0.5

      waypoints.append(copy.deepcopy(wpose))

      (plan, fraction) = self._group.compute_cartesian_path(waypoints, 0.01, 0.0)  
      rospy.loginfo("Path computed successfully. Moving the arm.")
      num_pts = len(plan.joint_trajectory.points)
      if (num_pts >= 3):
          del plan.joint_trajectory.points[0]
          del plan.joint_trajectory.points[1]
        
      self._group.execute(plan)

  def set_joint_angles(self, arg_list_joint_angles):
      list_joint_values = self._group.get_current_joint_values()
      self._group.set_joint_value_target(arg_list_joint_angles)
      self._computed_plan = self._group.plan()
      flag_plan = self._group.go(wait=True)

      list_joint_values = self._group.get_current_joint_values()
      pose_values = self._group.get_current_pose().pose
      if (flag_plan == True):
          pass
      else:
          pass
      return flag_plan

  def moveit_play_planned_path_from_file(self, arg_file_path, arg_file_name):
      file_path = arg_file_path + arg_file_name
      
      with open(file_path, 'r') as file_open:
          loaded_plan = yaml.load(file_open)
      
      ret = self._group.execute(loaded_plan)
      # rospy.logerr(ret)
      return ret
      

  def moveit_hard_play_planned_path_from_file(self, arg_file_path, arg_file_name, arg_max_attempts):
      n_attempts = 0
      flag_success = False

      while ( (n_attempts <= arg_max_attempts) and (flag_success is False) ):
          n_attempts += 1
          flag_success = self.moveit_play_planned_path_from_file(arg_file_path, arg_file_name)
          rospy.logwarn("attempts: {}".format(n_attempts) )
          self.clear_octomap()
      
      return True
  
  def func_callback_conveyor_color_topic(self, ConveyorPkgs):
    print("Inside conveyor_color_topic_subscriber_callback !")
    global q_ur5_2
    # q_ur5_2 = ConveyorPkgs

    for c_pkg in ConveyorPkgs.c_pkgs:
      print("--------------")
      print("color: " + c_pkg.color + ", name: " + c_pkg.name)
      var = []
      var.append(c_pkg.color)
      var.append(c_pkg.name)
      q_ur5_2.append(var)
      # q_ur5_2.append( tuple([str(c_pkg.color), str(c_pkg.name) ]) )
 
  # Callback function for logical camera subscriber :-
  def callback_logical_camera_2(self, LogicalCamera2Msg):
      # rospy.Subscriber("color_position_boxes", color_queue, self.func_callback_color_topic, queue_size=1)
      self.camera_to_box_pose.position.x = 80    # somewhere below ground
      self.camera_to_box_pose.position.y = 80
      for i in LogicalCamera2Msg.models:
          if(i.type == "ur5"):
              continue          # Do nothing if camera detects arm

          self.box_name = i.type  # Else, edit box_name, position
          self.camera_to_box_pose = i.pose
         
      global q_ur5_2
      # print("********  Length of q_ur5_2 is: " + str(len(q_ur5_2)) + "  **********")
      if(self.box_name == '' or self.box_name == 'kachra' or len(q_ur5_2) == 0):
          pass

      elif(q_ur5_2[0][1][3] != self.box_name[8] or q_ur5_2[0][1][4] != self.box_name[9]):
          pass

      else:
        pick_box_named = q_ur5_2[0]
        activate_conveyor_client(100, None)
        # pick_box_named[1][3] == self.box_name[8] and pick_box_named[1][4] == self.box_name[9] and 
        if(self.camera_to_box_pose.position.x <= 1.05 and self.camera_to_box_pose.position.y <= 0.15):  
            activate_conveyor_client(0, None)  # Stop conveyor movement
            if(self._group.get_current_pose().pose != self.home_pose):  # If arm is not at home_pose already go_to home_pose
                self.set_joint_angles(self.lst_joint_angles_home_pose)
            q_ur5_2.pop(0)

            print("Going to pick box named :- " + self.box_name  + ", color: " + pick_box_named[0])

            change_x = (self.camera_to_box_pose.position.z - 0)
            change_y = self.camera_to_box_pose.position.y
            self.ee_cartesian_translation(change_x, change_y, 0)
            # To activate vaccume gripper client -pick box from conveyor:- 
            gazebo_gripper_activate = activate_gripper_client(True)
            # self.set_joint_angles(self.lst_joint_angles_home_pose)
            self.ee_cartesian_translation(-1 * change_x, -1 * change_y, 0)
            self.moveit_hard_play_planned_path_from_file(self._file_path, 'home_pose_to_above_home_pose.yaml', 5)
            activate_conveyor_client(100, None)
            
            if(pick_box_named[0] == "red"):
              self.moveit_hard_play_planned_path_from_file(self._file_path, 'above_home_pose_to_above_red_bin.yaml', 5)
              # To deactivate vaccume gripper client - drop box in bin:-
              gazebo_gripper_activate = activate_gripper_client(False)
              self.moveit_hard_play_planned_path_from_file(self._file_path, 'above_red_bin_to_home_pose.yaml', 5)


            elif(pick_box_named[0] == "yellow"):
              # activate_conveyor_client(100, None)
              # self.moveit_hard_play_planned_path_from_file(self._file_path, 'home_pose_to_yellow_bin.yaml', 5)
              # gazebo_gripper_activate = activate_gripper_client(False)
              # self.moveit_hard_play_planned_path_from_file(self._file_path, 'yellow_bin_to_home_pose.yaml', 5)
              self.moveit_hard_play_planned_path_from_file(self._file_path, 'above_home_pose_to_yellow_bin.yaml', 5)
              # To deactivate vaccume gripper client - drop box in bin:-
              gazebo_gripper_activate = activate_gripper_client(False)
              self.moveit_hard_play_planned_path_from_file(self._file_path, 'yellow_bin_to_home_pose.yaml', 5)

            elif(pick_box_named[0] == "green"):
              # activate_conveyor_client(100, None)
              # self.moveit_hard_play_planned_path_from_file(self._file_path, 'home_pose_to_green_bin.yaml', 5)
              # gazebo_gripper_activate = activate_gripper_client(False)
              # self.moveit_hard_play_planned_path_from_file(self._file_path, 'green_bin_to_home_pose.yaml', 5)
              self.moveit_hard_play_planned_path_from_file(self._file_path, 'above_home_pose_to_above_green_bin.yaml', 5)
              gazebo_gripper_activate = activate_gripper_client(False)
              self.moveit_hard_play_planned_path_from_file(self._file_path, 'above_green_bin_to_home_pose.yaml', 5)

            else:
                print("Error in picking pkg, Dont know what is the color of pkg !")

            # gazebo_gripper_activate = activate_gripper_client(False)


      
  def logical_camera_2_subscriber(self):
    rospy.Subscriber("/eyrc/vb/logical_camera_2", LogicalCameraImage, self.callback_logical_camera_2, queue_size=1)
      
  def __del__(self):
        moveit_commander.roscpp_shutdown()
        rospy.loginfo("Object of class Ur5_2_Moveit Deleted.")


def main():
    rospy.init_node('node_t4_ur5_2', anonymous=True)
    activate_conveyor_client(100, None)
    global count
    count = int(0)
    try: 
        t4_ur5_2 = UR5_2()
        t4_ur5_2.moveit_hard_play_planned_path_from_file(t4_ur5_2._file_path, 'allzero_to_home_pose.yaml', 5)
    
        rospy.Subscriber('color_pos_boxes_order_conveyor', ConveyorPkgs, t4_ur5_2.func_callback_conveyor_color_topic)
        rospy.Subscriber("/eyrc/vb/logical_camera_2", LogicalCameraImage, t4_ur5_2.callback_logical_camera_2, queue_size=1)
        
        del t4_ur5_2

        rospy.spin()

    except rospy.ROSInterruptException:
        return
    except KeyboardInterrupt:
        return
        
    ################### TASK4 EXECUTION COMPLETED #####################

if __name__ == '__main__':
    main()

