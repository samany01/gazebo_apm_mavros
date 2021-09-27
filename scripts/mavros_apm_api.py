#!/usr/bin/env python3

import rospy
from mavros_msgs.msg import State
from std_msgs.msg import Float64
from sensor_msgs.msg import NavSatFix
from mavros_msgs.srv import SetMode
from geographic_msgs.msg import GeoPoseStamped
from geometry_msgs.msg import PoseStamped, Point, TwistStamped
from mavros_msgs.srv import CommandBool, CommandLong
from mavros_msgs.srv import CommandTOL
import time


class Copter:

        def __init__ (self):
        #publishers
                self.pub_velocity_localy = rospy.Publisher("/mavros/setpoint_velocity/cmd_vel",TwistStamped, queue_size = 10 )  
                self.pub_move_localy = rospy.Publisher("/mavros/setpoint_position/local",PoseStamped, queue_size=10 )  
                self.pub_move_globaly = rospy.Publisher("/mavros/setpoint_position/global", GeoPoseStamped, queue_size = 10)

  
        def drone_state(self, msg):             # some notification like connected or not and mode also armed or not
                msg = self.copter_state
            

        def global_position(self, msg):
                msg =  self.global_position    
                print("\nglobal position\n")
                print(f"latitude = {msg.latitude} \nlongitude = {msg.longitude} \naltitude = {msg.altitude}")


        def setmode(self, mode):
                print ("\nSetting Mode")
                rospy.wait_for_service('/mavros/set_mode') 
                if True:
                        change_mode = rospy.ServiceProxy('/mavros/set_mode', SetMode)
                        response = change_mode(custom_mode= mode)
                        if response.mode_sent:
                                print("*Mode setted* ")
                else:
                        print("Set mode failed" )


        def arming(self):
                print ("\nArming")
                rospy.wait_for_service('/mavros/cmd/arming')
                if True:
                        arming_cl = rospy.ServiceProxy('/mavros/cmd/arming', CommandBool)
                        response = arming_cl(value = True)
                        if response.success:
                                print("*Drone armed*\n")
                else :
                        print("Arming failed" )


        def takeoff(self, target_alt):
                rospy.wait_for_service('/mavros/cmd/takeoff')
                if True:
                        takeoff_cl = rospy.ServiceProxy('/mavros/cmd/takeoff', CommandTOL)
                        action = takeoff_cl(altitude= target_alt , latitude=0, longitude=0, min_pitch=0, yaw=0)
                        if action.success:
                                print("\n*Drone is flying.....")
                        #rospy.loginfo(action)
                else :
                        print("Takeoff failed" )


        def change_speed( self, speed, speed_type, rel_or_abs):     #look at https://mavlink.io/en/messages/common.html#mav_commands  mav_cmd_178 #
                rospy.wait_for_service('/mavros/cmd/command')
                if True:
                        command_cl = rospy.ServiceProxy('/mavros/cmd/command', CommandLong)
                        response = command_cl(command= 178, param1= speed_type, param2 =speed, param3=-1, param4=rel_or_abs)

                """ 
                        speed = m/s
                        speed_type=   (0=Airspeed, 1=Ground Speed, 2=Climb Speed, 3=Descent Speed) 
                        param4 = 0: absolute, 1: relative      relative to wind speed   
                """

                if response.success:
                        print(f"speed changed to {speed}m/s  = {response.success} ")         
                else:
                        print(f"speed changed to {speed}m/s  = {response.success} ")         


        def change_yaw( self, yaw):               #look at https://mavlink.io/en/messages/common.html#mav_commands  mav_cmd_115 #
                rospy.wait_for_service('/mavros/cmd/command')
                if True:
                        command_cl = rospy.ServiceProxy('/mavros/cmd/command', CommandLong)
                        response = command_cl(command= 115, param1= yaw, param2 =10, param3=1, param4=0)

                """
                yaw = 0 : 360 degree
                param2 = angular speed deg/s
                param3 = (clockwise or anticlockwise) = (1, -1)
                param4 = (0:absolute angle , 1:ralative offset) 
                #rospy.loginfo(f"speed changed to {speed}m/s :{response.success} )         
                #rospy.loginfo("Setting speed to {}m/s".format(str(speed)) )
                """

                if response.success:
                        print(f"yaw changed to {yaw}deg = {response.success} ")               
                else:
                        print(f"yaw changed to {yaw}deg = {response.success} ")         


        def do_parchuting(self):               #look at https://mavlink.io/en/messages/common.html#mav_commands  mav_cmd_115 #
                rospy.wait_for_service('/mavros/cmd/command')
                if True:
                        command_cl = rospy.ServiceProxy('/mavros/cmd/command', CommandLong)
                        response = command_cl(command= 208, param1= 1)

                """
                yaw = 0 : 360 degree
                param2 = angular speed deg/s
                param3 = (clockwise or anticlockwise) = (1, -1)
                param4 = (0:absolute angle , 1:ralative offset) 
                #rospy.loginfo(f"speed changed to {speed}m/s :{response.success} )         
                #rospy.loginfo("Setting speed to {}m/s".format(str(speed)) )
                """

                if response.success:
                        print( response.success )         
                else:
                        print(response.success)         


        def move_localy(self, x, y, z):                                          #  [+y : forward,-y :backward, +x : right, -x: left, +z : up, -z : dowm]  #
                local_position_class = PoseStamped()
                local_position_class.pose.position.x = x    
                local_position_class.pose.position.y= y 
                local_position_class.pose.position.z= z
                pub_move_localy.publish(local_position_class)


        def move_globaly(self, lat, long, alt):                                          #  alt here is WGS84 ellipsoid to convert it AMSL theres a module (prfered use mission planner)
                global_position_class = GeoPoseStamped()
                global_position_class.pose.position.latitude = lat    
                global_position_class.pose.position.longitude= long 
                global_position_class.pose.position.altitude= alt
                pub_move_globaly.publish(global_position_class)

        
        def set_velocity_localy(self ,x, y, z):                                          #  [+y : forward,-y :backward, +x : right, -x: left, +z : up, -z : dowm]  #
                local_velocity_class = TwistStamped()
                local_velocity_class.twist.linear.x = x    
                local_velocity_class.twist.linear.y= y 
                local_velocity_class.twist.linear.z= z
                pub_velocity_localy.publish(local_velocity_class)


        def disarm(self):
                print ("\nDisarming")
                rospy.wait_for_service('/mavros/cmd/arming')
                if True:
                        arming_cl = rospy.ServiceProxy('/mavros/cmd/arming', CommandBool)
                        action = arming_cl(value = False)
                if action.success:
                        print("*Drone disarmed*")
                        rospy.loginfo(action)
                else:
                        print("Disarming failed: %s" )

       
