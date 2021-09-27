#! /usr/bin/env python3
import rospy
from sensor_msgs.msg import LaserScan
from dronekit import connect, VehicleMode, LocationGlobalRelative
import time
import socket
#import exceptions
import argparse


def connectMyCopter():
    parser = argparse.ArgumentParser(description='Commands')
    parser.add_argument('--connect')
    args = parser.parse_args()
    connection_string = args.connect
    vehicle = connect(connection_string, wait_ready=True)
    return vehicle
    
def callback(msg):
    onepoint_range_tfmini= msg.ranges       #for tfmini plus only one point ranges
    txt = " tfmini_range = {}"
    print(txt.format(onepoint_range_tfmini[0]))
    return onepoint_range_tfmini

rospy.init_node('scan_values')
sub = rospy.Subscriber('/spur/laser/scan', LaserScan, callback)
rospy.spin()

#takeoff and landing
def arm_and_takeoff(aTargetAltitude):
    while not vehicle.is_armable:
        print("waiting for vehicle to become armable") 
        time.sleep(1)
    vehicle.mode = VehicleMode("GUIDED")
    vehicle.armed = True
    
    while vehicle.mode != "GUIDED" :
        print("waiting for vehicle to be guided")
        time.sleep(1)
    
    while vehicle.armed==False:
        print("waiting for vehicle to become armed")
        time.sleep(1)
        
    vehicle.simple_takeoff(aTargetAltitude)
     
    while True:
        txt = " tfmini_range = {}"
        print(txt.format(onepoint_range_tfmini[0]))
        print(" Altitude: ", vehicle.location.global_relative_frame.alt)
        # Break and return from function just below target altitude.
        if vehicle.location.global_relative_frame.alt>=aTargetAltitude* 0.95:
            break
        time.sleep(1)
    print("Reached target altitude")
    return None        
             
vehicle=connectMyCopter()
print("about to takeoff..")
vehicle.mode=VehicleMode("GUIDED")
arm_and_takeoff(2)




vehicle.mode=VehicleMode("LAND")
time.sleep(2)
while True:
    time.sleep(2)
vehicle.close() 
