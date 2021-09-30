#! /usr/bin/env python3
import rospy
import time
from mavros_apm_api import *
from sensor_msgs.msg import LaserScan


copter = Copter()

def avoidance_tfmini(msg):
    tfmini_range= msg.ranges[0]                #for tfmini plus only one point ranges
    #print(f"tfmini_range = {tfmini_range}")
    if  float(.40) <  tfmini_range < float(3):
        print("\n💢️obstacle detected")
        dis_to_avoid = tfmini_range - 3
        copter.set_velocity_localy(0, dis_to_avoid, 0)
        if True:
            print("\n*obstacle avoided successfully*")
      
            
if __name__ == "__main__" :
        rospy.init_node('obstacle_avoidance', anonymous=True)
        rate = rospy.Rate(2)                                                          
        copter.setmode("GUIDED")   
        copter.arming()   
        time.sleep(5) 
        copter.takeoff(2)
        time.sleep(10)
        copter.move_localy(0, 0, 2)
        time.sleep(10)        
        sub = rospy.Subscriber('/spur/laser/scan', LaserScan, avoidance_tfmini)
        
        rospy.spin()
        #time.sleep(15)               
