#!/usr/bin/env python
# -*- coding: utf-8 -*-
# 2021-11-14-tello 制导 动捕

import rospy
import tf
from geometry_msgs.msg import Twist
from std_msgs.msg import Int32
from std_msgs.msg import Empty
import numpy as np
import math
from geometry_msgs.msg import PoseStamped

# from geometry_msgs.msg import TwistStamped
f = 20

class bizhang:
    def __init__(self):
        global command, uav1_pos, uav2_pos
        rate = rospy.Rate(f)
        command = 0
        rospy.Subscriber('/command', Int32, self.command_callback)
        rospy.Subscriber('/tello1/ground_truth_to_tf/pose', PoseStamped, self.uav1_pose_callback)
        rospy.Subscriber('/tello5/ground_truth_to_tf/pose', PoseStamped, self.uav2_pose_callback)
        uav1_vel_pub = rospy.Publisher('/tello1/cmd_vel', Twist, queue_size=10)
        uav1_vel= Twist()
        uav1_pos, uav2_pos= np.array([0, 0, 0]), np.array([0, 0, 0])
        F_chili = np.array([0,0])
        u =np.array([0,0])

        # Main while loop.
        while not rospy.is_shutdown():
            
            F_chili[0]=

            if command == 1:

                uav1_vel_pub.publish(uav1_vel)

            rate.sleep()


    def command_callback(self, msg):
        global command
        command = msg.data

    def uav1_pose_callback(self, msg):
        global uav1_pos
        uav1_pos = np.array([msg.pose.position.x, msg.pose.position.y, msg.pose.position.z])

    def uav2_pose_callback(self, msg):
        global uav2_pos
        uav2_pos = np.array([msg.pose.position.x, msg.pose.position.y, msg.pose.position.z])



if __name__ == '__main__':
    # ROS节点初始化
    rospy.init_node('teidao', anonymous=True)
    try:
        bizhang()
    except rospy.ROSInterruptException:
        pass
