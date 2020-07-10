#!/usr/bin/python
#
# Send joint values to UR5 using messages
#

from std_msgs.msg import Header
from trajectory_msgs.msg import JointTrajectory

from trajectory_msgs.msg import JointTrajectoryPoint
import rospy

#[-ve is clockwise, +ve is counter-clockwise; +ve is up, +ve is up]
#first waypoint - rotate base 90 deg clockwise, second waypoint lower shoulder, third waypoint lower elbow
waypoints = [[-1.57,0,0,0,0,0],[-1.57,-0.3,0,0,0,0],[-1.57,-0.3,0.3,0,0,0]]


waypoint = [0,0,0,0,0,0]
def main():

    rospy.init_node('send_joints')
    pub = rospy.Publisher('/arm_controller/command',
                          JointTrajectory,
                          queue_size=10)

    # Create the topic message
    traj = JointTrajectory()
    traj.header = Header()
    # Joint names for UR5
    traj.joint_names = ['shoulder_pan_joint', 'shoulder_lift_joint',
                        'elbow_joint', 'wrist_1_joint', 'wrist_2_joint',
                        'wrist_3_joint']

    rate = rospy.Rate(1)
    cnt = 0
    pts = JointTrajectoryPoint()
    traj.header.stamp = rospy.Time.now()

    while not rospy.is_shutdown():
        joint_num = int(input("Which joint number would you like to move: "))
        while joint_num not in [i for i in range(6)]:
            joint_num = int(input("Which joint number would you like to move: "))
        delta  = float(input("How much would you like to move it by: "))
        waypoint[joint_num] += delta
        pts.positions = waypoint
        pts.time_from_start = rospy.Duration(1.0)

        # Set the points to the trajectory
        traj.points = []
        traj.points.append(pts)
        # Publish the message
        pub.publish(traj)
        rate.sleep()

        cnt+=1

if __name__ == '__main__':
    try:
        main()
    except rospy.ROSInterruptException:
        print ("Program interrupted before completion")
