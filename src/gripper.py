#!/usr/bin/env python
from __future__ import print_function

import rospy
from std_srvs.srv import Empty

def gripper(name, flag):
    node_name = "/ur5/{}/{}".format(name, flag)
    rospy.wait_for_service(node_name)
    try:
        return rospy.ServiceProxy(node_name, Empty)()
    except rospy.ServiceException, e:
        print("Service call failed: {}".format(e))

def toggle_gripper(flag):
    flag = "on" if flag else "off"
    gripper("vacuum_gripper", flag)
    # gripper("vacuum_gripper1", flag)
    # gripper("vacuum_gripper2", flag)
    # gripper("vacuum_gripper3", flag)
    # gripper("vacuum_gripper4", flag)
    # gripper("vacuum_gripper5", flag)
    # gripper("vacuum_gripper6", flag)
    # gripper("vacuum_gripper7", flag)
    # gripper("vacuum_gripper8", flag)
