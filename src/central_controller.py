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


object_list = []
order_input = {}
static_order_plan = []

def vision_callback(string):
    global object_list
    #rospy.loginfo("Vision callback triggered")
    lines = string.data.split('|')
    msg_id = lines[0]
    elms = lines[1:-1]

    object_list = [None]*len(elms)

    for i, elm in enumerate(elms):
        if not elm:
            break
        e = elm.split(',')
        tup = (e[1], float(e[2]), float(e[3]), float(e[4]))
        object_list[i] = tup

def order_callback(string):
    global order_input, static_order_plan
    if static_order_plan:
        return
    rospy.loginfo("Order callback triggered: "+string.data)
    lines = string.data.split('|')
    msg_id = lines[0]
    elms = lines[1:]

    order_input = {}

    for i, elm in enumerate(elms):
        if not elm:
            break
        e = elm.split(',')
        order_input[e[0]] = int(e[1])
    #rospy.loginfo(" order callback: " +str(order_input))

def main():
    global object_list, static_order_plan, order_input
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

    rospy.Subscriber("cv_pos", String, vision_callback)
    rospy.Subscriber("cv_order", String, order_callback)


    drop_point = ("drop_pt", 0.6, 0.7, 0.3)


    while not rospy.is_shutdown():

        # Get camera input as list of object positions

        # Get order input

        # Construct high-level path/order := L

        # For obj in L:
        #   Set goal as obj position
        #   Once picked up, set goal as drop-off position

        # Get camera input

        if not object_list:
            rospy.loginfo("No vision data found")
            rospy.sleep(0.5)
            continue

        rospy.loginfo("Object list: "+str(object_list))
        rospy.loginfo("Order list: "+str(order_input))
        # Get order input
        if not static_order_plan:

            # Construct high-level path/order := L

            static_order_plan = []

            # TODO: replace with actual optimized plan
            for key, val in order_input.items():
                num_required = val
                for elm in object_list:
                    if num_required == 0:
                        break
                    if elm[0] == key:
                        static_order_plan.append((key,elm))
                        static_order_plan.append(("dropoff",drop_point))
                        num_required -= 1
            # End TODO

        for target in static_order_plan:
            x = target[1][1]
            y = target[1][2]
            z = target[1][3]+0.2

            rospy.loginfo("Moving end effector to {}: x -> {}, y -> {}, z -> {}+0.2".format(target[0], x, y, z-0.2))
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

        rospy.loginfo("Order completed!")
        rospy.loginfo("Moving on to next order.")
        static_order_plan = []

if __name__ == "__main__":
    main()