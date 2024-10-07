#! /usr/bin/env python
import rospy
import tf2_ros
from geometry_msgs.msg import TransformStamped

def callback(msg):
    rospy.loginfo(msg.transform.translation)

if __name__ == "__main__":
    rospy.init_node('tf2_listener')
    tfBuffer = tf2_ros.Buffer()
    listener = tf2_ros.TransformListener(tfBuffer)
    rate = rospy.Rate(10.0)
    while not rospy.is_shutdown():
        try:
            trans = tfBuffer.lookup_transform('robot_1/map', 'object_1', rospy.Time())
        except (tf2_ros.LookupException, tf2_ros.ConnectivityException, tf2_ros.ExtrapolationException):
            continue

        callback(trans)

        rate.sleep()
