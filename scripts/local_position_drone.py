#!/usr/bin/env python3

import rospy
from mavros_msgs.msg import State, GlobalPositionTarget
from std_msgs.msg import Float64
from geometry_msgs.msg import PoseStamped
from sensor_msgs.msg import NavSatFix
from mavros_msgs.srv import SetMode
from mavros_msgs.srv import CommandBool, CommandLong
from mavros_msgs.srv import CommandTOL
import time



def position(msg):     
    #print("\nposition\n")
    print(msg.pose.position)



def position_listner():
    sub = rospy.Subscriber("/mavros/local_position/pose",  data_class =PoseStamped , callback = position)

position_listner()
rospy.init_node("local position of drone", anonymous=True )
rospy.spin()
    
pub = rospy.Publisher("/mavros/setpoint_position/local",PoseStamped, queue_size = 10 )  

def movement(a, b, c):
    loacl_position_class = PoseStamped()
    loacl_position_class.pose.position.x = a    
    loacl_position_class.pose.position.y= b 
    loacl_position_class.pose.position.z= c
    
    
