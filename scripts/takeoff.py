#!/usr/bin/env python3

import rospy
from mavros_msgs.srv import SetMode
from mavros_msgs.srv import CommandBool
from mavros_msgs.srv import CommandTOL
import time

rospy.init_node('mavros_takeoff_python')
rate = rospy.Rate(10)

# Set Mode
print ("\nSetting Mode")
rospy.wait_for_service('/mavros/set_mode')
try:
    change_mode = rospy.ServiceProxy('/mavros/set_mode', SetMode)
    response = change_mode(custom_mode="GUIDED")
    rospy.loginfo(response)
except rospy.ServiceException as e:
    print("Set mode failed: %s" %e)

# Arm
def arming():
    print ("\nArming")
    rospy.wait_for_service('/mavros/cmd/arming')
    if True:
        arming_cl = rospy.ServiceProxy('/mavros/cmd/arming', CommandBool)
        response = arming_cl(value = True)
        rospy.loginfo(response)
    else :
        print("Arming failed: %s" %e)

arming()

# Takeoff
def takeoff(target_alt):
    print ("\nTaking off")
    rospy.wait_for_service('/mavros/cmd/takeoff')
    try:
        takeoff_cl = rospy.ServiceProxy('/mavros/cmd/takeoff', CommandTOL)
   
        response = takeoff_cl(altitude= target_alt , latitude=0, longitude=0, min_pitch=0, yaw=0)
        rospy.loginfo(response)
    except rospy.ServiceException as e:
        print("Takeoff failed: %s" %e)


takeoff(3)

print ("\nHovering...")
time.sleep(10)

# Land
print ("\nLanding")
rospy.wait_for_service('/mavros/cmd/land')
try:
    takeoff_cl = rospy.ServiceProxy('/mavros/cmd/land', CommandTOL)
    response = takeoff_cl(altitude=0, latitude=0, longitude=0, min_pitch=0, yaw=0)
    rospy.loginfo(response)
except rospy.ServiceException as e:
    print("Landing failed: %s" %e)
time.sleep(10)
# Disarm
print ("\nDisarming")
rospy.wait_for_service('/mavros/cmd/arming')
try:
    arming_cl = rospy.ServiceProxy('/mavros/cmd/arming', CommandBool)
    response = arming_cl(value = False)
    rospy.loginfo(response)
except rospy.ServiceException as e:
    print("Disarming failed: %s" %e)
