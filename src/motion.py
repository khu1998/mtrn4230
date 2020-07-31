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
#  - communication block position to pick up

def main():
    moveit_commander.roscpp_initialize(sys.argv)
    rospy.init_node('motion_planner', anonymous=True)
                    
    robot = moveit_commander.RobotCommander()
    scene = moveit_commander.PlanningSceneInterface()  # set motion planning scene
    rospy.sleep(2)  # sleep to load scene otherwise table gets skipped
    group = moveit_commander.MoveGroupCommander("manipulator")

    # add table as collision object
    # table pose frame is same as robot frame
    table_pose = geometry_msgs.msg.PoseStamped()
    table_pose.header.frame_id = robot.get_planning_frame()
    table_pose.pose.position.x = 0.0
    table_pose.pose.position.y = -0.7
    table_pose.pose.position.z = -0.1
    scene.add_box("table", table_pose, size=(1, 1, 0.2))

    # pose goal is relative to frame of robot
    pose_goal = geometry_msgs.msg.Pose()
    pose_goal.orientation.w = 1.0  # hard coded orientation, to be figured out

    while not rospy.is_shutdown():
        try:
            position = raw_input("Please enter desired global co-ordinate in the form of <x>,<y>,<z>: ")
        except EOFError:
            print() # for new line
            rospy.loginfo("Got Ctrl+D, stopping motion planning...")
            break

        rospy.loginfo("Got position -> {}".format(position))
        try:
            x, y, z = map(float, position.split(','))
        except Exception:
            rospy.loginfo("Failed to interpret end position")
            continue
        rospy.loginfo("Moving end effector to x -> {}, y -> {}, z -> {}".format(x, y, z))
        pose_goal.position.x = 0.0 + x
        pose_goal.position.y = -0.7 + y
        pose_goal.position.z = -0.2 + z

        # set pose target
        group.set_pose_target(pose_goal)

        if group.go(wait=True):
            rospy.loginfo("Path planning was a success")
        else:
            rospy.logwarn("Path planning was a failure")

        # Calling `stop()` ensures that there is no residual movement
        group.stop()
        
        # It is always good to clear your targets after planning with poses.
        # Note: there is no equivalent function for clear_joint_value_targets()
        group.clear_pose_targets()

        rospy.sleep(0.1) # sleep to give other threads processing time

if __name__ == "__main__":
    main()