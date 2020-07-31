#!/usr/bin/env python
from __future__ import print_function

import sys
import copy
import rospy
import moveit_commander
import moveit_msgs.msg
import geometry_msgs.msg
from math import pi
from std_msgs.msg import String
from moveit_commander.conversions import pose_to_list

# TODO:
# put in loop
# check for valid path
# communication for position of blocks

def main():
    moveit_commander.roscpp_initialize(sys.argv)
    rospy.init_node('move_group_python_interface_tutorial',
                    anonymous=True)
                    
    robot = moveit_commander.RobotCommander()
    scene = moveit_commander.PlanningSceneInterface()
    group = moveit_commander.MoveGroupCommander("manipulator")

    rospy.sleep(1)

    # table pose relative to frame of robot
    table_pose = geometry_msgs.msg.PoseStamped()
    table_pose.header.frame_id = robot.get_planning_frame()
    table_pose.pose.position.x = 0.0
    table_pose.pose.position.y = -0.7
    table_pose.pose.position.z = -0.1
    table_name = "table"
    scene.add_box(table_name, table_pose, size=(1, 1, 0.2))

    # pose goal is relative to frame of robot
    pose_goal = geometry_msgs.msg.Pose()
    pose_goal.orientation.w = 1.0
    pose_goal.position.x = 0.0
    pose_goal.position.y = -0.7
    pose_goal.position.z = 0.1
    group.set_pose_target(pose_goal)

    plan = group.go(wait=True)
    # Calling `stop()` ensures that there is no residual movement
    group.stop()
    # It is always good to clear your targets after planning with poses.
    # Note: there is no equivalent function for clear_joint_value_targets()
    group.clear_pose_targets()

if __name__ == "__main__":
    main()