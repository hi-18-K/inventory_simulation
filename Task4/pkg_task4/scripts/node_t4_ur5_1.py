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
import actionlib
import threading
import tf2_ros          
import tf2_geometry_msgs  
import math         
import rospkg
import yaml
import os
import cv2
from pyzbar.pyzbar import decode
from collections import deque

from std_msgs.msg import String
from sensor_msgs.msg import Image
from cv_bridge import CvBridge, CvBridgeError
from geometry_msgs.msg import Pose
from moveit_commander.conversions import pose_to_list
from pkg_vb_sim.msg import LogicalCameraImage

from std_srvs.srv import Empty
from pkg_vb_sim.srv import *

from pkg_task4.msg import *

ur5_1_pub = rospy.Publisher('color_pos_boxes_order_conveyor', ConveyorPkgs, queue_size=1)
# camera1_pub = rospy.Publisher('color_pos_boxes_in_shelf', ShelfPkgs, queue_size=1)

#To activate gripper in gazebo(client in pkg_vb_sim):-
def activate_gripper_client(activate_vacuum_gripper):
    rospy.wait_for_service('/eyrc/vb/ur5/activate_vacuum_gripper/ur5_1')
    try:
      # resp1 = 'NA'
      # while(resp1 == 'NA'):
      activate_gripper = rospy.ServiceProxy('/eyrc/vb/ur5/activate_vacuum_gripper/ur5_1', vacuumGripper)
      resp1 = activate_gripper(activate_vacuum_gripper)
      print("ur5_1_activate_vacuum_gripper:" + str(activate_vacuum_gripper) + " in gazebo")
        # rospy.sleep(0.5)
      return resp1.result
    except rospy.ServiceException as e:
      print("UR5_1 Vaccume_Gripper Service call failed: %s"%e)  
      return None

class UR5_1(object):
  def __init__(self):
    super(UR5_1, self).__init__()
    self._robot_ns = '/ur5_1'
    self._planning_group = "manipulator"
    
    self._commander = moveit_commander.roscpp_initialize(sys.argv)
    self._robot = moveit_commander.RobotCommander(robot_description= self._robot_ns + "/robot_description", ns=self._robot_ns)
    self._scene = moveit_commander.PlanningSceneInterface(ns=self._robot_ns)
    self._group = moveit_commander.MoveGroupCommander(self._planning_group, robot_description= self._robot_ns + "/robot_description", ns=self._robot_ns)
    self._display_trajectory_publisher = rospy.Publisher( self._robot_ns + '/move_group/display_planned_path', moveit_msgs.msg.DisplayTrajectory, queue_size=1)
    self._exectute_trajectory_client = actionlib.SimpleActionClient( self._robot_ns + '/execute_trajectory', moveit_msgs.msg.ExecuteTrajectoryAction)
    self._exectute_trajectory_client.wait_for_server()

    self._planning_frame = self._group.get_planning_frame()
    self._eef_link = self._group.get_end_effector_link()
    self._group_names = self._robot.get_group_names()
    self._box_name = ''

    # Attribute to store computed trajectory by the planner	
    self._computed_plan = ''

    # Current State of the Robot is needed to add box to planning scene
    self._curr_state = self._robot.get_current_state()

    rp = rospkg.RosPack()
    self._pkg_path = rp.get_path('pkg_task4')
    self._file_path = self._pkg_path + '/config/ur5_1_trajectories/'
    
    ####################################################################################
    # To take input from camera1 into a queue, named 'q_ur5_1'->
    # queue maintains pair - (color, index-position)
    self.is_fill = False
    self.bridge = CvBridge()
    self.q = []

    self.lst_joint_angles_0 = [math.radians(0),
                          math.radians(0),
                          math.radians(0),
                          math.radians(0),
                          math.radians(0),
                          math.radians(0)]
    # home_pose                      
    self.lst_joint_angles_home_pose = [math.radians(7.84328385087),
                          math.radians(-120.00),
                          math.radians(-58.3065131143),
                          math.radians(-90.7219273039),
                          math.radians(89.9554380813),
                          math.radians(7.89480500776)]

  
  def clear_octomap(self):
      clear_octomap_service_proxy = rospy.ServiceProxy(self._robot_ns + "/clear_octomap", Empty)
      return clear_octomap_service_proxy()

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
      number_attempts = 0
      flag_success = False

      while ( (number_attempts <= arg_max_attempts) and (flag_success is False) ):
          number_attempts += 1
          flag_success = self.moveit_play_planned_path_from_file(arg_file_path, arg_file_name)
          rospy.logwarn("attempts: {}".format(number_attempts) )
          # # self.clear_octomap()
      
      return True

  def check_and_execute(self, current):
    if(current[3] == '0' and current[4] == '0'):
        # if(self._group.get_current_joint_values() == self.lst_joint_angles_0):
        self.moveit_hard_play_planned_path_from_file(self._file_path + 'pkg00/', 'allzeros_to_pkg00_modified.yaml', 5)
        # elif(self._group.get_current_joint_values() == self.lst_joint_angles_home_pose):
          # self.moveit_hard_play_planned_path_from_file(self._file_path + 'pkg00', 'home_pose_pkg00_modified', 5)
        
        self.moveit_hard_play_planned_path_from_file(self._file_path + 'pkg00/', 'pkg00_modified_to_pkg00.yaml', 5)
        gazebo_gripper_activate = activate_gripper_client(True)
        print("gazebo_gripper_activate_ur5_1: " + str(gazebo_gripper_activate))
        self.moveit_hard_play_planned_path_from_file(self._file_path + 'pkg00/', 'pkg00_to_back_after_picking.yaml', 5)
        self.moveit_hard_play_planned_path_from_file(self._file_path + 'pkg00/', 'pkg00_modified_to_home_pose.yaml', 5)
        # self.moveit_hard_play_planned_path_from_file(self._file_path , 'home_pose_to_drop_loc.yaml', 5)
        gazebo_gripper_activate = activate_gripper_client(False)
        print("gazebo_gripper_activate_ur5_1: " + str(gazebo_gripper_activate))
        # self.moveit_hard_play_planned_path_from_file(self._file_path , 'drop_loc_to_home_pose.yaml', 5)
        
    elif(current[3] == '0' and current[4] == '1'):
      self.moveit_hard_play_planned_path_from_file(self._file_path + 'pkg01/', 'home_pose_to_pkg01.yaml', 5)
      gazebo_gripper_activate = activate_gripper_client(True)
      print("gazebo_gripper_activate_ur5_1: " + str(gazebo_gripper_activate))
      self.moveit_hard_play_planned_path_from_file(self._file_path + 'pkg01/', 'pkg01_to_back_after_picking.yaml', 5)
      self.moveit_hard_play_planned_path_from_file(self._file_path + 'pkg01/', 'pkg01_modified_to_home_pose.yaml', 5)
      # self.moveit_hard_play_planned_path_from_file(self._file_path , 'home_pose_to_drop_loc.yaml', 5)
      gazebo_gripper_activate = activate_gripper_client(False)
      print("gazebo_gripper_activate_ur5_1: " + str(gazebo_gripper_activate))
      # self.moveit_hard_play_planned_path_from_file(self._file_path , 'drop_loc_to_home_pose.yaml', 5)

    elif(current[3] == '0' and current[4] == '2'):
      self.moveit_hard_play_planned_path_from_file(self._file_path + 'pkg02/', 'home_pose_to_pkg02_modified.yaml', 5)
      self.moveit_hard_play_planned_path_from_file(self._file_path + 'pkg02/', 'pkg02_modified_to_pkg02.yaml', 5)
      gazebo_gripper_activate = activate_gripper_client(True)
      print("gazebo_gripper_activate_ur5_1: " + str(gazebo_gripper_activate))
      self.moveit_hard_play_planned_path_from_file(self._file_path + 'pkg02/', 'pkg02_to_back_after_picking.yaml', 5)
      self.moveit_hard_play_planned_path_from_file(self._file_path + 'pkg02/', 'pkg02_modified_to_home_pose.yaml', 5)
      # self.moveit_hard_play_planned_path_from_file(self._file_path , 'home_pose_to_drop_loc.yaml', 5)
      gazebo_gripper_activate = activate_gripper_client(False)
      print("gazebo_gripper_activate_ur5_1: " + str(gazebo_gripper_activate))
      # self.moveit_hard_play_planned_path_from_file(self._file_path , 'drop_loc_to_home_pose.yaml', 5)

    elif(current[3] == '1' and current[4] == '0'):
      self.moveit_hard_play_planned_path_from_file(self._file_path, 'home_pose_to_random.yaml', 5)
      self.moveit_hard_play_planned_path_from_file(self._file_path, 'random_to_home_pose.yaml', 5)
      # self.moveit_hard_play_planned_path_from_file(self._file_path, 'home_pose_to_drop_loc.yaml', 5)
      # self.moveit_hard_play_planned_path_from_file(self._file_path, 'drop_loc_to_home_pose.yaml', 5)
      self.moveit_hard_play_planned_path_from_file(self._file_path + 'pkg10/', 'home_pose_to_pkg10.yaml', 5)
      gazebo_gripper_activate = activate_gripper_client(True)
      print("gazebo_gripper_activate_ur5_1: " + str(gazebo_gripper_activate))
      self.moveit_hard_play_planned_path_from_file(self._file_path + 'pkg10/', 'pkg10_to_back_after_picking.yaml', 5)
      self.moveit_hard_play_planned_path_from_file(self._file_path + 'pkg10/', 'pkg10_modified_to_home_pose.yaml', 5)
      # self.moveit_hard_play_planned_path_from_file(self._file_path , 'home_pose_to_drop_loc.yaml', 5)
      gazebo_gripper_activate = activate_gripper_client(False)
      print("gazebo_gripper_activate_ur5_1: " + str(gazebo_gripper_activate))
      # self.moveit_hard_play_planned_path_from_file(self._file_path , 'drop_loc_to_home_pose.yaml', 5)

    elif(current[3] == '1' and current[4] == '1'):
      pass
      
    elif(current[3] == '1' and current[4] == '2'):
      self.moveit_hard_play_planned_path_from_file(self._file_path, 'home_pose_to_drop_loc.yaml', 5)
      self.moveit_hard_play_planned_path_from_file(self._file_path, 'drop_loc_to_home_pose.yaml', 5)
      # self.moveit_hard_play_planned_path_from_file(self._file_path, 'home_pose_to_random2.yaml', 5)
      # self.moveit_hard_play_planned_path_from_file(self._file_path, 'random2_to_home_pose.yaml', 5)
      self.moveit_hard_play_planned_path_from_file(self._file_path + 'pkg12/', 'home_pose_to_pkg12.yaml', 5)
      gazebo_gripper_activate = activate_gripper_client(True)
      print("gazebo_gripper_activate_ur5_1: " + str(gazebo_gripper_activate))
      self.moveit_hard_play_planned_path_from_file(self._file_path + 'pkg12/', 'pkg12_to_back_after_picking.yaml', 5)
      self.moveit_hard_play_planned_path_from_file(self._file_path + 'pkg12/', 'pkg12_modified_to_home_pose.yaml', 5)
      # self.moveit_hard_play_planned_path_from_file(self._file_path , 'home_pose_to_drop_loc.yaml', 5)
      gazebo_gripper_activate = activate_gripper_client(False)
      print("gazebo_gripper_activate_ur5_1: " + str(gazebo_gripper_activate))
      # self.moveit_hard_play_planned_path_from_file(self._file_path , 'drop_loc_to_home_pose.yaml', 5)
      
    elif(current[3] == '2' and current[4] == '0'):
      self.moveit_hard_play_planned_path_from_file(self._file_path, 'home_pose_to_random.yaml', 5)
      self.moveit_hard_play_planned_path_from_file(self._file_path, 'random_to_home_pose.yaml', 5)
      self.moveit_hard_play_planned_path_from_file(self._file_path + 'pkg20/', 'home_pose_to_pkg20.yaml', 5)
      gazebo_gripper_activate = activate_gripper_client(True)
      print("gazebo_gripper_activate_ur5_1: " + str(gazebo_gripper_activate))
      self.moveit_hard_play_planned_path_from_file(self._file_path + 'pkg20/', 'pkg20_to_back_after_picking.yaml', 5)
      self.moveit_hard_play_planned_path_from_file(self._file_path + 'pkg20/', 'pkg20_modified_to_home_pose.yaml', 5)
      # self.moveit_hard_play_planned_path_from_file(self._file_path , 'home_pose_to_drop_loc.yaml', 5)
      gazebo_gripper_activate = activate_gripper_client(False)
      print("gazebo_gripper_activate_ur5_1: " + str(gazebo_gripper_activate))
      # self.moveit_hard_play_planned_path_from_file(self._file_path , 'drop_loc_to_home_pose.yaml', 5)
    
    elif(current[3] == '2' and current[4] == '1'):
      pass

    elif(current[3] == '2' and current[4] == '2'):
      self.moveit_hard_play_planned_path_from_file(self._file_path, 'home_pose_to_random.yaml', 5)
      self.moveit_hard_play_planned_path_from_file(self._file_path, 'random_to_home_pose.yaml', 5)
      self.moveit_hard_play_planned_path_from_file(self._file_path + 'pkg22/', 'home_pose_to_pkg22_modified.yaml', 5)
      self.moveit_hard_play_planned_path_from_file(self._file_path + 'pkg22/', 'pkg22_modified_to_pkg22.yaml', 5)
      gazebo_gripper_activate = activate_gripper_client(True)
      print("gazebo_gripper_activate_ur5_1: " + str(gazebo_gripper_activate))
      self.moveit_hard_play_planned_path_from_file(self._file_path + 'pkg22/', 'pkg22_to_back_after_picking.yaml', 5)
      self.moveit_hard_play_planned_path_from_file(self._file_path + 'pkg22/', 'pkg22_modified_to_home_pose.yaml', 5)
      # self.moveit_hard_play_planned_path_from_file(self._file_path , 'home_pose_to_drop_loc.yaml', 5)
      gazebo_gripper_activate = activate_gripper_client(False)
      print("gazebo_gripper_activate_ur5_1: " + str(gazebo_gripper_activate))
      # self.moveit_hard_play_planned_path_from_file(self._file_path , 'drop_loc_to_home_pose.yaml', 5)
      
    elif(current[3] == '3' and current[4] == '0'):
      self.moveit_hard_play_planned_path_from_file(self._file_path + 'pkg30/', 'home_pose_to_pkg30_modified.yaml', 5)
      self.moveit_hard_play_planned_path_from_file(self._file_path + 'pkg30/', 'pkg30_modified_to_pkg30.yaml', 5)
      gazebo_gripper_activate = activate_gripper_client(True)
      print("gazebo_gripper_activate_ur5_1: " + str(gazebo_gripper_activate))
      self.moveit_hard_play_planned_path_from_file(self._file_path + 'pkg30/', 'pkg30_to_back_after_picking.yaml', 5)
      self.moveit_hard_play_planned_path_from_file(self._file_path + 'pkg30/', 'pkg30_modified_to_home_pose.yaml', 5)
      # self.moveit_hard_play_planned_path_from_file(self._file_path , 'home_pose_to_drop_loc.yaml', 5)
      gazebo_gripper_activate = activate_gripper_client(False)
      print("gazebo_gripper_activate_ur5_1: " + str(gazebo_gripper_activate))
      # self.moveit_hard_play_planned_path_from_file(self._file_path , 'drop_loc_to_home_pose.yaml', 5)

    elif(current[3] == '3' and current[4] == '1'):
      pass

    elif(current[3] == '3' and current[4] == '2'):
      self.moveit_hard_play_planned_path_from_file(self._file_path + 'pkg32/', 'home_pose_to_pkg32_modified.yaml', 5)
      self.moveit_hard_play_planned_path_from_file(self._file_path + 'pkg32/', 'pkg32_modified_to_pkg32.yaml', 5)
      gazebo_gripper_activate = activate_gripper_client(True)
      print("gazebo_gripper_activate_ur5_1: " + str(gazebo_gripper_activate))
      self.moveit_hard_play_planned_path_from_file(self._file_path + 'pkg32/', 'pkg32_to_back_after_picking.yaml', 5)
      self.moveit_hard_play_planned_path_from_file(self._file_path + 'pkg32/', 'pkg32_modified_to_home_pose.yaml', 5)
      # self.moveit_hard_play_planned_path_from_file(self._file_path , 'home_pose_to_drop_loc.yaml', 5)
      gazebo_gripper_activate = activate_gripper_client(False)
      print("gazebo_gripper_activate_ur5_1: " + str(gazebo_gripper_activate))
      # self.moveit_hard_play_planned_path_from_file(self._file_path , 'drop_loc_to_home_pose.yaml', 5)

  def get_qr_data(self,arg_image): 
    qr_result = decode(arg_image)
    if ( len( qr_result ) > 0): 
      return (qr_result[0].data) # Return the data utf-8 data read from qr 
    # (We are scanning one portion of shelf at a time) 
    else :
      return ('NA')

  # callback function used by subscriber of camera1/image_raw topic 
  def callback_camera1(self,data):
    if not self.is_fill:
      try:
        cv_image = self.bridge.imgmsg_to_cv2(data, "bgr8")

      except CvBridgeError as e:
        rospy.logerr(e)

      image = cv_image

      img00 = image[300:430, 110:230, :]
      img01 = image[300:430, 300:420, :]
      img02 = image[300:430, 490:610, :]

      img10 = image[480:600, 110:230, :]
      img11 = image[480:600, 300:420, :]
      img12 = image[480:600, 490:610, :]

      img20 = image[630:760, 110:230, :]
      img21 = image[630:760, 300:420, :]
      img22 = image[630:760, 490:610, :]

      img30 = image[780:900, 110:230, :]
      img31 = image[780:900, 300:420, :]
      img32 = image[780:900, 490:610, :]
      print("Inside callback !")
      
      self.q.append( (self.get_qr_data(img00), "img00") ) 
      self.q.append( (self.get_qr_data(img01), "img01") )
      self.q.append( (self.get_qr_data(img02), "img02") )

      self.q.append( (self.get_qr_data(img10), "img10") )
      # self.q.append( (self.get_qr_data(img11), "img11") )
      self.q.append( (self.get_qr_data(img12), "img12") )

      self.q.append( (self.get_qr_data(img20), "img20") )
      # self.q.append( (self.get_qr_data(img21), "img21") )
      self.q.append( (self.get_qr_data(img22), "img22") )

      self.q.append( (self.get_qr_data(img30), "img30") )
      # self.q.append( (self.get_qr_data(img31), "img31") )
      self.q.append( (self.get_qr_data(img32), "img32") )

      self.is_fill = True  
      global ur5_1_pub
      pub_list = []
      for i in range(0, len(self.q)):
        print(str(i+1) + ": " + str(self.q[i][0]) + ", " + str(self.q[i][1]))
        if(self.q[i][0] == 'NA'):
          continue
        pub_data = ConveyorClrPkg()
        pub_data.color = self.q[i][0]
        pub_data.name = self.q[i][1]
        pub_list.append(pub_data)

      global ur5_1_pub
      ur5_1_pub.publish(pub_list)

      for i in range(0, len(self.q)):
        self.check_and_execute(self.q[i][1])
      
  # Subscribe camera1/image_raw topic:
  def sub_camera1(self):
    image_sub = rospy.Subscriber("/eyrc/vb/camera_1/image_raw", Image, self.callback_camera1, queue_size=1)   

  def __del__(self):
        moveit_commander.roscpp_shutdown()
        rospy.loginfo("Object of class Ur5Moveit Deleted.")


def main():
    rospy.init_node('node_t4_ur5_1', anonymous=True)
    t4_ur5_1 = UR5_1()
    # t4_ur5_1.set_joint_angles(t4_ur5_1.lst_joint_angles_home_pose)
    print("inside main before callback !")
    # rospy.Subscriber("color_pos_boxes_in_shelf", ShelfPkgs ,t4_ur5_1.func_callback_color_topic, queue_size = 3)
    t4_ur5_1.sub_camera1()

    print("inside main after callback !")
    # t4_ur5_1.sub_color_pos_boxes_in_shelf()

    del t4_ur5_1

    rospy.spin()
    # try:
    #     # gazebo_gripper_activate = activate_gripper_client(False)
    #     # print("gazebo_gripper_activate_ur5_1: " + str(gazebo_gripper_activate))

    #     # t4_ur5_1.set_joint_angles(self.lst_joint_angles_0)
        
    #     # for i in range(0, 9):
    #     #   t4_ur5_1.check_and_execute(t4_ur5_1.q_ur5_1[i])

    #     # rospy.spin()
    #     # del t4_ur5_1 
    # except rospy.ROSInterruptException:
    #     return
    # except KeyboardInterrupt:
    #     return
    


    
    ################### TASK4 EXECUTION COMPLETED #####################

if __name__ == '__main__':
    main()


