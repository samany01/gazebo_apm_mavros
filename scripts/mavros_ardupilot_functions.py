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


def drone_state(msg):             # some notification like connected or not and mode also armed or not
    #print(msg)
    rospy.spin()
    
    
def drone_state_listener():
     sub = rospy.Subscriber('/mavros/state', State)


def global_position(msg):     
    print("\nglobal position\n")
    print(f"latitude = {msg.latitude} \nlongitude = {msg.longitude} \naltitude = {msg.altitude}")
    rospy.spin()


def global_position_listner():
    sub = rospy.Subscriber('/mavros/global_position/global', data_class = NavSatFix, callback = global_position)
    
    
def takoff_until_reached_altitude(msg):
    altitude = float(msg.data)
    takeoff(target_alt)
    #print (f"main altitude = {altitude}")	      		
    if altitude >= 0.97*float(target_alt) :
        print("\nAltitude reached")
        rospy.spin()


def takoff_until_reached_altitude_listener():
    sub = rospy.Subscriber("/mavros/global_position/rel_alt", Float64, takoff_until_reached_altitude)


def setmode(mode):
    print ("\nSetting Mode")
    rospy.wait_for_service('/mavros/set_mode') 
    if True:
        change_mode = rospy.ServiceProxy('/mavros/set_mode', SetMode)
        response = change_mode(custom_mode= mode)
        if response.mode_sent:
            print("*Mode setted* ")
    else:
        print("Set mode failed" )


def arming():
    print ("\nArming")
    rospy.wait_for_service('/mavros/cmd/arming')
    if True:
        arming_cl = rospy.ServiceProxy('/mavros/cmd/arming', CommandBool)
        response = arming_cl(value = True)
        if response.success:
            print("*Drone armed*\n")
    else :
        print("Arming failed" )


def takeoff(target_alt):
    rospy.wait_for_service('/mavros/cmd/takeoff')
    if True:
        takeoff_cl = rospy.ServiceProxy('/mavros/cmd/takeoff', CommandTOL)
        action = takeoff_cl(altitude= target_alt , latitude=0, longitude=0, min_pitch=0, yaw=0)
        if action.success:
            print("\n*Drone is flying.....")
        #rospy.loginfo(action)
        #print(action.success)
    else :
        print("Takeoff failed" )


def change_speed( speed, speed_type, rel_or_abs):     #look at https://mavlink.io/en/messages/common.html#mav_commands  mav_cmd_178 #
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


def change_yaw( yaw):               #look at https://mavlink.io/en/messages/common.html#mav_commands  mav_cmd_115 #
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


def do_parchuting():               #look at https://mavlink.io/en/messages/common.html#mav_commands  mav_cmd_115 #
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


def move_localy(x, y, z):                                          #  [+y : forward,-y :backward, +x : right, -x: left, +z : up, -z : dowm]  #
    local_position_class = PoseStamped()
    local_position_class.pose.position.x = x    
    local_position_class.pose.position.y= y 
    local_position_class.pose.position.z= z
    pub_move_localy.publish(local_position_class)
    

def move_globaly(lat, long, alt):                                          #  alt here is WGS84 ellipsoid to convert it AMSL theres a module (prfered use mission planner)
    global_position_class = GeoPoseStamped()
    global_position_class.pose.position.latitude = lat    
    global_position_class.pose.position.longitude= long 
    global_position_class.pose.position.altitude= alt
    pub_move_globaly.publish(global_position_class)
        
        
def set_velocity_localy(x, y, z):                                          #  [+y : forward,-y :backward, +x : right, -x: left, +z : up, -z : dowm]  #
    local_velocity_class = TwistStamped()
    local_velocity_class.twist.linear.x = x    
    local_velocity_class.twist.linear.y= y 
    local_velocity_class.twist.linear.z= z
    pub_velocity_localy.publish(local_velocity_class)

      
def land():
    print ("\nLanding")
    rospy.wait_for_service('/mavros/cmd/land')
    if True:
        takeoff_cl = rospy.ServiceProxy('/mavros/cmd/land', CommandTOL)
        action = takeoff_cl(altitude=0, latitude=0, longitude=0, min_pitch=0, yaw=0)
        if action.success:
            print("*Drone landed*")
    else:
        print("Landing failed: %s" )
        
      
def disarm():
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
        
       
if __name__ == "__main__":

    rospy.init_node('mavros_ardupilot_functions')
    pub_velocity_localy = rospy.Publisher("/mavros/setpoint_velocity/cmd_vel",TwistStamped, queue_size = 10 )  
    pub_move_localy = rospy.Publisher("/mavros/setpoint_position/local",PoseStamped, queue_size = 10 )  
    pub_move_globaly = rospy.Publisher("/mavros/setpoint_position/global", GeoPoseStamped, queue_size = 10)

    rate = rospy.Rate(2)   
    setmode("GUIDED")   
    arming()   
    time.sleep(5) 
    takeoff(2)
    rospy.spin()
