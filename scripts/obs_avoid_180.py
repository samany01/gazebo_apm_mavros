#! /usr/bin/env python3
import rospy
import time
from mavros_apm_api import *
from sensor_msgs.msg import LaserScan
from math import *


copter = Copter()

def avoidance_tfmini(msg):
    for i in range (1, len(msg.ranges)):
        if  float(.40) <  msg.ranges[i] < float(3):
            ray_angle=msg.angle_min + (i*msg.angle_increment)
            #print("range_num ={}".format(i))
            #print("ray_angle ={}".format(ray_angle))
            #print("obs_dist = {}".format (msg.ranges[i]))
            save_zone = 3.5 - msg.ranges[i]
            y_dir_avoid=-save_zone*cos(ray_angle)
            x_dir_avoid = save_zone*sin(ray_angle)
            print("\n💢️obstacle detected")
            copter.set_velocity_localy(x_dir_avoid, y_dir_avoid, 0)
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
