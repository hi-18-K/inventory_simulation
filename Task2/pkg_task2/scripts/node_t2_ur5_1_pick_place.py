#!/usr/bin/env python

from __future__ import print_function
from six.moves import input

import sys
import copy
import rospy
import moveit_commander
import moveit_msgs.msg
import geometry_msgs.msg
from math import pi
from std_msgs.msg import String
from moveit_commander.conversions import pose_to_list
from pkg_vb_sim.srv import *


def all_close(goal, actual, tolerance):
  all_equal = True
  if type(goal) is list:
    for index in range(len(goal)):
      if abs(actual[index] - goal[index]) > tolerance:
        return False

  elif type(goal) is geometry_msgs.msg.PoseStamped:
    return all_close(goal.pose, actual.pose, tolerance)

  elif type(goal) is geometry_msgs.msg.Pose:
    return all_close(pose_to_list(goal), pose_to_list(actual), tolerance)

  return True


#To activate gripper in gazebo(client in pkg_vb_sim):-

def activate_gripper_client(activate_vacuum_gripper):
    rospy.wait_for_service('/eyrc/vb/ur5_1/activate_vacuum_gripper')
    try:
      activate_gripper = rospy.ServiceProxy('/eyrc/vb/ur5_1/activate_vacuum_gripper', vacuumGripper)
      resp1 = activate_gripper(activate_vacuum_gripper)
      print("activate_vacuum_gripper:" + str(activate_vacuum_gripper) + " in gazebo")
      return resp1.result
    except rospy.ServiceException as e:
      print("Service call failed: %s"%e)  

def usage():
    return "[]"


class MoveGroupPythonIntefaceTutorial(object):
  def __init__(self):
    super(MoveGroupPythonIntefaceTutorial, self).__init__()

    moveit_commander.roscpp_initialize(sys.argv)
    rospy.init_node('node_t2_ur5_1_pick_place', anonymous=True)

    robot = moveit_commander.RobotCommander()

    scene = moveit_commander.PlanningSceneInterface()
    group_name = "ur5_1_planning_group"
    move_group = moveit_commander.MoveGroupCommander(group_name)

    display_trajectory_publisher = rospy.Publisher('/move_group/display_planned_path',
                                                   moveit_msgs.msg.DisplayTrajectory,
                                                   queue_size=20)

    planning_frame = move_group.get_planning_frame()
    print("============ Planning frame: %s" % planning_frame)

    # We can also print the name of the end-effector link for this group:
    eef_link = move_group.get_end_effector_link()
    print("============ End effector link: %s" % eef_link)

    # We can get a list of all the groups in the robot:
    group_names = robot.get_group_names()
    print("============ Available Planning Groups:", robot.get_group_names())

    # Sometimes for debugging it is useful to print the entire state of the
    # robot:
    print("============ Printing robot state ===========")
    print(robot.get_current_state())
    print("")

    # Misc variables
    self.box_name = 'box'
    self.robot = robot
    self.scene = scene
    self.move_group = move_group
    self.display_trajectory_publisher = display_trajectory_publisher
    self.planning_frame = planning_frame
    self.eef_link = eef_link
    self.group_names = group_names

  def go_to_pose_goal(self, arg_pose):
    move_group = self.move_group

    move_group.set_pose_target(arg_pose)
    
    plan = move_group.go(wait=True)

    if (plan == True):
        rospy.loginfo(
            '\033[94m' + ">>> go_to_pose() Success" + '\033[0m')
    else:
        rospy.logerr(
            '\033[94m' + ">>> go_to_pose() Failed. Solution for Pose not Found." + '\033[0m')
    # Calling `stop()` ensures that there is no residual movement
    move_group.stop()
    move_group.clear_pose_targets()

    current_pose = self.move_group.get_current_pose().pose
    return all_close(arg_pose, current_pose, 0.01)


  def wait_for_state_update(self, box_is_known=False, box_is_attached=False, timeout=4):
    box_name = self.box_name
    scene = self.scene
    start = rospy.get_time()
    seconds = rospy.get_time()
    while (seconds - start < timeout) and not rospy.is_shutdown():
      attached_objects = scene.get_attached_objects([box_name])
      is_attached = len(attached_objects.keys()) > 0

      is_known = box_name in scene.get_known_object_names()

      if (box_is_attached == is_attached) and (box_is_known == is_known):
        return True

      # Sleep so that we give other threads time on the processor
      rospy.sleep(0.1)
      seconds = rospy.get_time()

    return False


  def add_box(self, timeout=4):
    box_name = self.box_name
    scene = self.scene

    box_pose = geometry_msgs.msg.PoseStamped()
    box_pose.header.frame_id = "world"
    box_pose.pose.orientation.w = 1.0
    box_pose.pose.position.x = 0.05 # 0.09
    box_pose.pose.position.y = 0.48 # 0.43
    box_pose.pose.position.z = 1.84
    box_name = "box"
    scene.add_box(box_name, box_pose, size=(0.15, 0.15, 0.15))

    self.box_name=box_name
    return self.wait_for_state_update(box_is_known=True, timeout=timeout)


  def attach_box(self, timeout=4):
    box_name = self.box_name
    robot = self.robot
    scene = self.scene
    eef_link = self.eef_link
    group_names = self.group_names

    grasping_group = 'ur5_1_planning_group'
    touch_links = robot.get_link_names(group=grasping_group)
    scene.attach_box(eef_link, box_name, touch_links=touch_links)

    return self.wait_for_state_update(box_is_attached=True, box_is_known=False, timeout=timeout)


  def detach_box(self, timeout=4):
    box_name = self.box_name
    scene = self.scene
    eef_link = self.eef_link

    scene.remove_attached_object(eef_link, name=box_name)
    
    return self.wait_for_state_update(box_is_known=True, box_is_attached=False, timeout=timeout)


  def remove_box(self, timeout=4):
    box_name = self.box_name
    scene = self.scene

    scene.remove_world_object(box_name)

    return self.wait_for_state_update(box_is_attached=False, box_is_known=False, timeout=timeout)


def main():
  try:
    print("")
    print("----------------------------------------------------------")
    print("Welcome to Task2")
    print("----------------------------------------------------------")
    print("Press Ctrl-D to exit at any time")
    print("")
    tutorial = MoveGroupPythonIntefaceTutorial()

    # tutorial.go_to_pose_goal()

    tutorial.add_box()
    # intermediate locations 
    # (for avoiding collision and "go_to_pose() Failed. Solution for Pose not Found.")

    # GotoBox -pos max hieght attaining position
    ur5_pose_1 = geometry_msgs.msg.Pose()
    ur5_pose_1.position.x = 0.000822714776654
    ur5_pose_1.position.y = 0.10915010937
    ur5_pose_1.position.z = 1.95105858432
    ur5_pose_1.orientation.x = 1.32843786881e-07
    ur5_pose_1.orientation.y = 0.00164148791415
    ur5_pose_1.orientation.z = 0.000210623104507
    ur5_pose_1.orientation.w = 2.12177767514e-09

    # Frustum penetrating box 
    ur5_pose_2 = geometry_msgs.msg.Pose()
    ur5_pose_2.position.x = 0.0533460477845
    ur5_pose_2.position.y = 0.259171751739
    ur5_pose_2.position.z = 1.9143211227
    ur5_pose_2.orientation.x = 9.7605292128e-06
    ur5_pose_2.orientation.y = 0.00166394819707
    ur5_pose_2.orientation.z = -7.42958421915e-05
    ur5_pose_2.orientation.w = 1.0

    print("Going to max height attaining pose:- ")
    tutorial.go_to_pose_goal(ur5_pose_1)
    rospy.sleep(2)

    print("Going to pick box pose:- ")
    tutorial.go_to_pose_goal(ur5_pose_2)
    rospy.sleep(6)

    tutorial.attach_box()
    
    print("Box attached in rviz successfully!")

    if len(sys.argv) == 1:
      activate_vacuum_gripper = True
    else:
        print(usage())
        sys.exit(1)
    gazebo_gripper_activate = activate_gripper_client(activate_vacuum_gripper)
    print(gazebo_gripper_activate)

    rospy.sleep(1)
   
    # intermediate drop locations 
    # (for avoiding collision and "go_to_pose() Failed. Solution for Pose not Found.")

    ur5_pose_before_drop_1 = geometry_msgs.msg.Pose()
    ur5_pose_before_drop_1.position.x = -0.0901454549566
    ur5_pose_before_drop_1.position.y = -0.062274347901
    ur5_pose_before_drop_1.position.z = 1.81182607816
    ur5_pose_before_drop_1.orientation.x = -7.70672480231e-05
    ur5_pose_before_drop_1.orientation.y = 0.00155566570423
    ur5_pose_before_drop_1.orientation.z = 6.51110000322e-05
    ur5_pose_before_drop_1.orientation.w = 2.12177767514e-09

    ur5_pose_before_drop_2 = geometry_msgs.msg.Pose()
    ur5_pose_before_drop_2.position.x = -0.149408652019
    ur5_pose_before_drop_2.position.y = -0.221361969977
    ur5_pose_before_drop_2.position.z = 1.8796851007
    ur5_pose_before_drop_2.orientation.x = 5.5688246173e-05
    ur5_pose_before_drop_2.orientation.y = 0.00150812428518
    ur5_pose_before_drop_2.orientation.z = -0.000126352428789
    ur5_pose_before_drop_2.orientation.w = 2.12177767514e-09

    ur5_pose_before_drop_3 = geometry_msgs.msg.Pose()
    ur5_pose_before_drop_3.position.x = -0.284998169562
    ur5_pose_before_drop_3.position.y = -0.25530143817
    ur5_pose_before_drop_3.position.z = 1.8635091385
    ur5_pose_before_drop_3.orientation.x = 5.83672567794e-05
    ur5_pose_before_drop_3.orientation.y = 0.00148177354145
    ur5_pose_before_drop_3.orientation.z = 0.000168866917652
    ur5_pose_before_drop_3.orientation.w = 2.12177767514e-09

    #GOTOBucket(drop) loc - final drop location
    ur5_pose_final_drop_loc = geometry_msgs.msg.Pose()
    ur5_pose_final_drop_loc.position.x = -0.666272224105
    ur5_pose_final_drop_loc.position.y = -0.241918009411
    ur5_pose_final_drop_loc.position.z = 1.00553602213
    ur5_pose_final_drop_loc.orientation.x = -3.30905775786e-05
    ur5_pose_final_drop_loc.orientation.y = 0.001610721457
    ur5_pose_final_drop_loc.orientation.z = 3.13001701172e-05
    ur5_pose_final_drop_loc.orientation.w = 2.12177767514e-09

    print("After attaching box going to dropping pose:- ")
    print("Going to intermediate poses:- ")
    tutorial.go_to_pose_goal(ur5_pose_before_drop_1)
    tutorial.go_to_pose_goal(ur5_pose_before_drop_2)
    tutorial.go_to_pose_goal(ur5_pose_before_drop_3)

    print("Going to final drop location pose")
    tutorial.go_to_pose_goal(ur5_pose_final_drop_loc)
    rospy.sleep(6)

    tutorial.detach_box()
    print("Box is dropped successfully in rviz!")
    if len(sys.argv) == 1:
      activate_vacuum_gripper = False
    else:
        print(usage())
        sys.exit(1)
    gazebo_gripper_activate = activate_gripper_client(activate_vacuum_gripper)
    print(gazebo_gripper_activate)
    rospy.sleep(1)

    print("After dropping box going to initial pose(allZeros position):- ")
    print("Going to intermediate pose(straightUp):- ")

    # Intermediate pose
    # (for avoiding collision and "go_to_pose() Failed. Solution for Pose not Found.")
    ur5_pose_after_drop_1 = geometry_msgs.msg.Pose()
    ur5_pose_after_drop_1.position.x = 0.0953699822384
    ur5_pose_after_drop_1.position.y = 0.10912919735
    ur5_pose_after_drop_1.position.z = 1.85633325896
    ur5_pose_after_drop_1.orientation.x = 3.06336175362
    ur5_pose_after_drop_1.orientation.y = 1.56998443354
    ur5_pose_after_drop_1.orientation.z = 3.06314357573
    ur5_pose_after_drop_1.orientation.w = 2.12177767514e-09

    print("Now going to allZeros pose:- ")

    ur5_pose_after_drop_2 = geometry_msgs.msg.Pose()
    ur5_pose_after_drop_2.position.x = 0.817313113173
    ur5_pose_after_drop_2.position.y = 0.108761433027
    ur5_pose_after_drop_2.position.z = 0.944579923819
    ur5_pose_after_drop_2.orientation.x = -3.14159265074
    ur5_pose_after_drop_2.orientation.y = 0.000155989015405
    ur5_pose_after_drop_2.orientation.z = 3.1411410382
    ur5_pose_after_drop_2.orientation.w = 2.12177767514e-09

    tutorial.go_to_pose_goal(ur5_pose_after_drop_1)
    tutorial.go_to_pose_goal(ur5_pose_after_drop_2)
    
    tutorial.remove_box()

    print("================== Task2 execution completed! =======================")
  except rospy.ROSInterruptException:
    return
  except KeyboardInterrupt:
    return

if __name__ == '__main__':
  main()
