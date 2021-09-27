#! /usr/bin/env python3

import rospy
from sensor_msgs.msg import LaserScan

def callback(msg):
    onepoint_range_tfmini= msg.ranges       #for tfmini plus only one point ranges
    print(f" tfmini_range = {onepoint_range_tfmini[0]}")

rospy.init_node('scan_values')
sub = rospy.Subscriber('/spur/laser/scan', LaserScan, callback)
rospy.spin()
