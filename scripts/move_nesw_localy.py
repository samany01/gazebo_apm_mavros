#!/usr/bin/env python3

import rospy
from mavros_apm_api import *
import time
import math


copter = Copter()

def nesw(yaw, x, y, z):
        copter.change_yaw(yaw)
        if yaw == 0 :
                time.sleep(7)
                
        copter.set_velocity_localy(x, y, z)
        delay = math.sqrt(x**2+y**2+z**2)
        print(delay)
        time.sleep(delay)        
        
        
def main():

        rospy.init_node("drone_controller_move_4_directions", anonymous=True)
        copter.setmode("GUIDED")
        copter.arming()
        time.sleep(5)
        copter.takeoff(2)
        time.sleep(10)
        nesw(0, 0, 3,0)
        nesw(90, 3, 0,0)
        nesw(180, 0, -3,0)
        nesw(270, -3, 0,0)
        nesw(360, 0, 0,0)   
        copter.setmode("LAND")     
        rospy.spin()
        
if __name__ == "__main__": 
        try:
                main()
        except KeyboardInterrupt :
                exit()
