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
Vm = 0.2
thet1_0, thet2_0, thet3_0 = math.pi/2, math.pi/4, math.pi/2
psi1_0, psi2_0, psi3_0 = 0, -math.pi/2, 0


class guidance:
    def __init__(self):
        global command, uav1_pos, uav2_pos, uav3_pos, target_pos
        rate = rospy.Rate(f)
        command = 0
        rospy.Subscriber('/command', Int32, self.command_callback)
        rospy.Subscriber('/vrpn_client_node/tello_szx1/pose', PoseStamped, self.uav1_pose_callback)
        rospy.Subscriber('/vrpn_client_node/tello_szx2/pose', PoseStamped, self.uav2_pose_callback)
        rospy.Subscriber('/vrpn_client_node/tello_szx3/pose', PoseStamped, self.uav3_pose_callback)
        rospy.Subscriber('/vrpn_client_node/tello2_1/pose', PoseStamped, self.target_pose_callback)
        uav1_vel_pub = rospy.Publisher('/tello1/cmd_vel', Twist, queue_size=10)
        uav2_vel_pub = rospy.Publisher('/tello2/cmd_vel', Twist, queue_size=10)
        uav3_vel_pub = rospy.Publisher('/tello3/cmd_vel', Twist, queue_size=10)
        target_vel_pub = rospy.Publisher('/tello4/cmd_vel', Twist, queue_size=10)
        uav1_takeoff_pub = rospy.Publisher('/tello1/takeoff', Empty, queue_size=10)
        uav2_takeoff_pub = rospy.Publisher('/tello2/takeoff', Empty, queue_size=10)
        uav3_takeoff_pub = rospy.Publisher('/tello3/takeoff', Empty, queue_size=10)
        target_takeoff_pub = rospy.Publisher('/tello4/takeoff', Empty, queue_size=10)
        uav1_land_pub = rospy.Publisher('/tello1/land', Empty, queue_size=10)
        uav2_land_pub = rospy.Publisher('/tello2/land', Empty, queue_size=10)
        uav3_land_pub = rospy.Publisher('/tello3/land', Empty, queue_size=10)
        target_land_pub = rospy.Publisher('/tello4/land', Empty, queue_size=10)
        uav1_vel, uav2_vel, uav3_vel, target_vel = Twist(), Twist(), Twist(), Twist()
        uav1_pos, uav2_pos, uav3_pos, target_pos = np.array([0, 0, 0]), np.array([0, 0, 0]), np.array([0, 0, 0]), np.array([0, 0, 0])

        '''参数初始化'''
        thet1, thet2, thet3 = thet1_0, thet2_0, thet3_0
        psi1, psi2, psi3 = psi1_0, psi2_0, psi3_0
        r1, q_lon1, q_lat1 = 0, 0, 0
        r2, q_lon2, q_lat2 = 0, 0, 0
        r3, q_lon3, q_lat3 = 0, 0, 0

        # Main while loop.
        while not rospy.is_shutdown():

            if command == 0:  # 停止
                uav1_vel = Twist()
                uav2_vel = Twist()
                uav3_vel = Twist()
                target_vel = Twist()
                uav1_vel_pub.publish(uav1_vel)
                uav2_vel_pub.publish(uav2_vel)
                uav3_vel_pub.publish(uav3_vel)
                target_vel_pub.publish(target_vel)

            elif command == 2:
                uav1_takeoff_pub.publish()
                uav2_takeoff_pub.publish()
                uav3_takeoff_pub.publish()
                target_takeoff_pub.publish()
                print('起飞！！')

            elif command == 3:
                uav1_land_pub.publish()
                uav2_land_pub.publish()
                uav3_land_pub.publish()
                target_land_pub.publish()
                print('降落！！')

            if command == 1:
                r1, q_lon1, q_lat1, thet1, psi1, uav1_vel = self.bili_guidance(uav1_pos, target_pos, r1, q_lon1, q_lat1,
                                                                               thet1, psi1)
                # print(q_lon1, q_lat1, thet1, psi1)
                print(uav3_vel)
                r2, q_lon2, q_lat2, thet2, psi2, uav2_vel = self.bili_guidance(uav2_pos, target_pos, r2, q_lon2, q_lat2,
                                                                               thet2, psi2)
                # print(q_lon2, q_lat2, thet2, psi2)
                r3, q_lon3, q_lat3, thet3, psi3, uav3_vel = self.bili_guidance(uav3_pos, target_pos, r3, q_lon3, q_lat3,
                                                                               thet3, psi3)
                uav1_vel_pub.publish(uav1_vel)
                uav2_vel_pub.publish(uav2_vel)
                uav3_vel_pub.publish(uav3_vel)
                target_vel_pub.publish(target_vel)
                if r1 < 0.2 or r2 < 0.2 or r3 < 0.2:
                    command = 0
                    # print(r1, r2, r3)
                    print('Bomb!!!')
            else:
                r1, q_lon1, q_lat1 = self.cal_r_q(uav1_pos, target_pos)
                r2, q_lon2, q_lat2 = self.cal_r_q(uav2_pos, target_pos)
                r3, q_lon3, q_lat3 = self.cal_r_q(uav3_pos, target_pos)
                thet1, thet2, thet3 = thet1_0, thet2_0, thet3_0
                psi1, psi2, psi3 = psi1_0, psi2_0, psi3_0

            rate.sleep()

    def bili_guidance(self, uav_pos, target_pos, r, q_lon, q_lat, thet, psi):
        r2, q_lon2, q_lat2 = self.cal_r_q(uav_pos, target_pos)
        # dotr = (r2 - r) / f
        dq_lon = (q_lon2 - q_lon)
        dq_lat = (q_lat2 - q_lat)
        '''比例导引制导律'''
        dthet = 5 * dq_lon
        dpsi = 5 * dq_lat
        thet2 = thet + dthet
        psi2 = psi + dpsi
        uav_vel = Twist()
        x = Vm * math.cos(thet) * math.sin(psi)
        y = Vm * math.cos(thet) * math.cos(psi)
        z = Vm * math.sin(thet)
        uav_vel.linear.x = self.vel_com_xy(y)
        uav_vel.linear.y = self.vel_com_xy(-x)
        uav_vel.linear.z = self.vel_com_z(z)
        return r2, q_lon2, q_lat2, thet2, psi2, uav_vel

    def vel_com_xy(self, vel):
        # tello速度补偿
        vel2 = abs(vel)
        if vel2 <= 0.3:
            vel3 = 3 * vel2
        elif vel2 <= 0.5:
            vel3 = 1.7 * vel2
        elif vel2 <= 0.6:
            vel3 = 1.3 * vel2
        elif vel2 <= 0.8:
            vel3 = 1.2 * vel2
        elif vel2 <= 1:
            vel3 = 1.05 * vel2
        else:
            vel3 = k * vel2
        return np.sign(vel)*vel3

    def vel_com_z(self, vel):
        # tello速度补偿
        vel2 = abs(vel)
        if vel2 <= 0.3:
            vel3 = 5 * vel2
        elif vel2 <= 0.5:
            vel3 = 3.8 * vel2
        elif vel2 <= 1:
            vel3 = 2.8 * vel2
        else:
            vel3 = vel2
        return np.sign(vel)*vel3

    def cal_r_q(self, uav_pos, target_pos):
        # print(target_pos)
        dd = target_pos - uav_pos
        r = math.sqrt(pow(dd[0], 2) + pow(dd[1], 2) + pow(dd[2], 2))
        q_lon = math.atan2(dd[1], math.sqrt(pow(dd[0], 2) + pow(dd[2], 2)))  # 水平面视线角
        q_lat = -math.atan2(dd[2], dd[0])  # 垂直平面视线角
        return r, q_lon, q_lat

    def quat_to_angle(self, orientation):
        x = orientation.x
        y = orientation.y
        z = orientation.z
        w = orientation.w
        rpy = tf.transformations.euler_from_quaternion([x, y, z, w])
        return rpy

    def command_callback(self, msg):
        global command
        print('Receive command  ', msg.data)
        if command != 2 and msg.data == 1:
            print('请先起飞！！')
        else:
            command = msg.data

    def uav1_pose_callback(self, msg):
        global uav1_pos
        # uav1_pos = np.array([-msg.pose.position.z / 1000, msg.pose.position.x / 1000, msg.pose.position.y / 1000])
        if msg.pose.position.y < 999999:
            uav1_pos = np.array([msg.pose.position.y / 1000, msg.pose.position.z / 1000, -msg.pose.position.x / 1000])

    def uav2_pose_callback(self, msg):
        global uav2_pos
        # uav2_pos = np.array([-msg.pose.position.z / 1000, msg.pose.position.x / 1000, msg.pose.position.y / 1000])
        if msg.pose.position.y < 999999:
            uav2_pos = np.array([msg.pose.position.y / 1000, msg.pose.position.z / 1000, -msg.pose.position.x / 1000])

    def uav3_pose_callback(self, msg):
        global uav3_pos
        # uav3_pos = np.array([-msg.pose.position.z / 1000, msg.pose.position.x / 1000, msg.pose.position.y / 1000])
        if msg.pose.position.y < 999999:
            uav3_pos = np.array([msg.pose.position.y / 1000, msg.pose.position.z / 1000, -msg.pose.position.x / 1000])

    def target_pose_callback(self, msg):
        global target_pos
        # target_pos = np.array([-msg.pose.position.z / 1000, msg.pose.position.x / 1000, msg.pose.position.y / 1000])
        if msg.pose.position.y < 999999:
            target_pos = np.array([msg.pose.position.y / 1000, msg.pose.position.z / 1000, -msg.pose.position.x / 1000])


if __name__ == '__main__':
    # ROS节点初始化
    rospy.init_node('teidao', anonymous=True)
    try:
        guidance()
    except rospy.ROSInterruptException:
        pass
