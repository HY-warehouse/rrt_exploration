#!/usr/bin/env python
# -*- coding: utf-8 -*-

import rospy
import tf
import math
from nav.msg import Odometry
from geometry_msgs.msg import Pose, Twist, Transform, TransformStamped

class GetPose():
    def __init__(self):
        rospy.init_node('get_pose', anonymous=True)
        self.odom_sub1 = rospy.Subscriber('/robot_1/odom', Odometry, self.odom_callback)
        self.odom_sub2 = rospy.Subscriber('/robot_2/odom', Odometry, self.odom_callback)
        self.odom_sub3 = rospy.Subscriber('/robot_3/odom', Odometry, self.odom_callback)
        self.odom_sub4 = rospy.Subscriber('/robot_4/odom', Odometry, self.odom_callback)
        self.odom1 = Odometry()
        self.odom2 = Odometry()
        self.odom3 = Odometry()
        self.odom4 = Odometry()
        self.pose1 = Pose()
        self.pose2 = Pose()
        self.pose3 = Pose()
        self.pose4 = Pose()
        self.euler_angles_y1=0
        self.euler_angles_y2=0
        self.euler_angles_y3=0
        self.euler_angles_y4=0

    def odom_callback(self, msg):
        self.odom1 = msg
        self.pose1 = self.odom.pose.pose
        self.euler_angles_y1 = math.atan2(2*(self.odom.pose.pose.orientation.w * self.odom.pose.pose.orientation.z + self.odom.pose.pose.orientation.x * self.odom.pose.pose.orientation.y), 1 - 2 * (self.odom.pose.pose.orientation.y * self.odom.pose.pose.orientation.y + z*z))
        
        self.pose1.orientation.x = self.odom.pose.pose.orientation.x
        self.pose1.orientation.y = self.odom.pose.pose.orientation.y
        self.pose1.orientation.z = self.odom.pose.pose.orientation.z
        self.pose1.orientation.w = self.odom.pose.pose.orientation.w
        print(self.pose1)


if __name__ == '__main__':
    try:
        get_pose = GetPose()
        rospy.spin()
    except rospy.ROSInterruptException:
        pass