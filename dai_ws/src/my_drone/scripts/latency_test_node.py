#!/usr/bin/env python
import rospy
from nav_msgs.msg import Odometry
from sensor_msgs.msg import Image

def odom_cb(msg):
    rospy.loginfo("ODOM received at: %f", msg.header.stamp.to_sec())

def image_cb(msg):
    rospy.loginfo("IMAGE received at: %f", msg.header.stamp.to_sec())

if __name__ == '__main__':
    rospy.init_node('latency_test_node')
    rospy.Subscriber("/odom", Odometry, odom_cb)
    rospy.Subscriber("/oak/rgb/preview/image_raw", Image, image_cb)
    rospy.spin()
