#!/usr/bin/env python
from __future__ import print_function

import sys
import copy
import rospy
import moveit_commander
import moveit_msgs.msg
import geometry_msgs.msg
from math import pi
from std_msgs.msg import String, Bool
from moveit_commander.conversions import pose_to_list
from gripper import toggle_gripper


# IMPORTANT SETUP INFO
# In the terminal that runs central_controller.py,
# the variable ROS_MASTER_URI must be set up
# For Jamie's setup:
#   source ./simulation_ws/devel/setup.bash
#   export SVGA_VGPU10=0
#   export ROS_IP=$(ifconfig enp0s17 | grep "inet addr" | cut -d':' -f2 | cut -d' ' -f1)
#   echo $ROS_IP
#   export ROS_MASTER_URI=http://"$ROS_IP":11311/
#   python ./simulation_ws/src/mtrn4230/src/central_controller.py

USE_GRIPPER = False
DEBUG_DELL = False

object_list = []
order_input = {}
static_order_plan = []
move_robot = False

def move_to_target(target):
    global pose_goal, scene, robot, group, status_pub, USE_GRIPPER

    x = target[1][1]
    y = target[1][2]
    z = target[1][3]+0.005

    rospy.loginfo("Moving end effector to {}: x -> {}, y -> {}, z -> {}+0.2".format(target[0], x, y, z-0.2))
    status_pub.publish(String("Moving end effector to {}: x -> {}, y -> {}, z -> {}+0.2".format(target[0], x, y, z-0.2)))

    pose_goal.position.x = 0.0 + x
    pose_goal.position.y = -0.7 + y
    pose_goal.position.z = -0.2 + z

    # set pose target
    group.set_pose_target(pose_goal)

    if group.go(wait=True):
        rospy.loginfo("Path planning was a success")
    else:
        rospy.logwarn("Path planning was a failure")

    # Naive path plan. After pick up, it moves to "central location", drop
    # it off at the drop off zone, then return back to the central location.
    if target[0] == "dropoff":
        #rotate arm to dropoff location
        rospy.loginfo("Dropping off")

        goal = group.get_current_joint_values()
        goal[0] = 0
        # print(goal)
        # print(goal_step)
        # for step in range(10):
        #     goal[0] = goal[0]-goal_step 
        if group.go(goal, wait=True):
            rospy.loginfo("At dropoff zone. Dropping item.")
            if USE_GRIPPER:
                toggle_gripper(False)
        else:
            rospy.logwarn("Failed to drop off at dropoff zone.")
        
        group.set_max_velocity_scaling_factor(1)
        group.set_max_acceleration_scaling_factor(1)
        # #rotate arm back to initial position
        # goal[0] = -pi/2
        # if group.go(goal, wait=True):
        #     rospy.loginfo("Back to init location.")
        # else:
        #     rospy.logwarn("Failed to move to init location.")
    else:
        rospy.loginfo("Toggling gripper")

        if USE_GRIPPER:
            toggle_gripper(True)
            # gripper(scene,group)
        group.set_max_velocity_scaling_factor(0.1)
        group.set_max_acceleration_scaling_factor(0.1)
        
    # Calling `stop()` ensures that there is no residual movement
    group.stop()
        

    # It is always good to clear your targets after planning with poses.
    # Note: there is no equivalent function for clear_joint_value_targets()
    group.clear_pose_targets()

    rospy.sleep(0.1) # sleep to give other threads processing time

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
    global order_input

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

def status_callback(string):
    ""#""ospy.loginfo("Status callback triggered: "+string.data)

def reset_callback(msg):
    global static_order_plan, drop_point
    rospy.loginfo("Reset callback triggered.")

    static_order_plan =[("dropoff",drop_point)]
    move_to_target(static_order_plan[0])

def gripper(scene,group):
    pose = group.get_current_pose()
    rospy.loginfo(pose)
    x = pose.pose.position.x
    y = pose.pose.position.y
    z = pose.pose.position.z

    names = scene.get_known_object_names_in_roi(x-0.1,y-0.1,z-0.25,x+0.1,y+0.1,z)
    rospy.logwarn(names)

def move_robot_callback(string):
    global move_robot
    if string.data == "On":
        move_robot = True
    else:
        move_robot = False
    rospy.loginfo("MoveRobot callback triggered: "+string.data+"\nMoveRobot value is {}\n".format("True" if move_robot else "False"))

def main():
    global object_list, static_order_plan, order_input, move_robot
    global pose_goal, scene, robot, group, status_pub, USE_GRIPPER, drop_point

    moveit_commander.roscpp_initialize(sys.argv)
    rospy.init_node('motion_planner', anonymous=True)
                    
    robot = moveit_commander.RobotCommander()
    scene = moveit_commander.PlanningSceneInterface()  # set motion planning scene
    rospy.sleep(2)  # sleep to load scene otherwise table gets skipped
    group = moveit_commander.MoveGroupCommander("manipulator")
    group.set_max_velocity_scaling_factor(0.1)
    group.set_max_acceleration_scaling_factor(0.3)

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
    pose_goal.orientation.w = 0.5
    pose_goal.orientation.x = 0.5
    pose_goal.orientation.y = 0.5
    pose_goal.orientation.z = -0.5

    rospy.Subscriber("cv_pos", String, vision_callback)
    rospy.Subscriber("cv_order", String, order_callback)
    status_pub = rospy.Publisher("cv_status", String, queue_size=3)
    rospy.Subscriber("cv_status", String, status_callback)
    rospy.Subscriber("cv_move_robot", String, move_robot_callback)
    rospy.Subscriber("cv_reset", Bool, reset_callback)

    status_pub.publish(String("Robot connected."))

    drop_point = ("drop_pt", 0.0, 0.2, 0.45)

    connected_index = 0

    while not rospy.is_shutdown():

        #

        # Get camera input as list of object positions

        # Get order input

        # Construct high-level path/order := L

        # For obj in L:
        #   Set goal as obj position
        #   Once picked up, set goal as drop-off position

        # Get camera input

        if not DEBUG_DELL:
            if not object_list:
                rospy.loginfo("No vision data found")
                status_pub.publish(String("Robot connected: "+str(connected_index)+"\n"))
                connected_index = connected_index+1
                rospy.sleep(0.5)
                continue

        rospy.loginfo("Object list: "+str(object_list))
        #rospy.loginfo("Order list: "+str(order_input))
        # Get order input

        if not order_input:
            rospy.loginfo("Waiting for order.")
            status_pub.publish(String("Waiting for order."))
            continue

        if not static_order_plan:

            # Construct high-level path/order := L

            static_order_plan = []
            if DEBUG_DELL:
                static_order_plan.append(("point1",("shape type/colour", 0.44, 0.3, 0.256)))
                static_order_plan.append(("dropoff", (drop_point)))
                static_order_plan.append(("point2", ("shape type/colour", 0.54, 0.2, 0.256)))
                static_order_plan.append(("dropoff", (drop_point)))
                static_order_plan.append(("point3", ("shape type/colour", 0.2, 0.5, 0.256)))
                static_order_plan.append(("dropoff", (drop_point)))
            else:
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
        
        status_pub.publish(String("Order received: "+str(order_input)))
        rospy.sleep(1)
        object_list = []
        order_input = {}
        for target in static_order_plan:
            move_to_target(target)

        rospy.loginfo("Order completed!")
        rospy.loginfo("Moving on to next order.")
        status_pub.publish(String("Order completed. Moving on to next order."))
        static_order_plan = []

if __name__ == "__main__":
    main()

