#!/usr/bin/env python3

from mavros_apm_api import*
import time



copter = Copter()

def main():
        rospy.init_node("drone_controller", anonymous=True)
        copter.setmode("GUIDED")
        copter.arming()
        time.sleep(5)
        copter.takeoff(2)
        rospy.spin()
        
if __name__ == "__main__": 
        try:
                main()
        except KeyboardInterrupt :
                exit()
        
