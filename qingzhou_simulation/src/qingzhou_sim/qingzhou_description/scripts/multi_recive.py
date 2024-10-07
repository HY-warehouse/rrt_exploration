#!/usr/bin/env python
# coding=UTF-8

import rospy,math,random
import numpy as np
import message_filters
from std_msgs.msg import String
from sensor_msgs.msg import LaserScan
from laser_geometry import LaserProjection
from geometry_msgs.msg import PoseWithCovarianceStamped

def multi_callback(Subcriber_laser,Subcriber_pose):
    print ("同步完成！")

if __name__ == '__main__':
    rospy.init_node('multi_receive',anonymous=True)

    subcriber_laser = message_filters.Subscriber('scan', LaserScan, queue_size=1)
    subcriber_pose  = message_filters.Subscriber('car_pose', PoseWithCovarianceStamped, queue_size=1)
    
    sync = message_filters.ApproximateTimeSynchronizer([subcriber_laser, subcriber_pose],10,0.1,allow_headerless=True)

    sync.registerCallback(multi_callback)

    rospy.spin()
