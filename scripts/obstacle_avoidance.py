#! /usr/bin/env python3
import rospy
import time
from mavros_ardupilot_functions import setmode, arming, takeoff, change_yaw
from sensor_msgs.msg import LaserScan
from geometry_msgs.msg import PoseStamped, Point, TwistStamped
from mavros_msgs.srv import CommandBool, CommandLong



def move_localy(x, y, z):                                          #  [-x : forward,+x :backward, +y : right, -y: left, +z : up, -z : dowm]  #
    local_position_class = PoseStamped()
    local_position_class.pose.position.x = x    
    local_position_class.pose.position.y= y 
    local_position_class.pose.position.z= z
    #yaw = change_yaw(yaw)
    pub_move_localy.publish(local_position_class)

def set_velocity_localy(x, y, z):                                          #  [+y : forward,-y :backward, +x : right, -x: left, +z : up, -z : dowm]  #
    local_velocity_class = TwistStamped()
    local_velocity_class.twist.linear.x = x    
    local_velocity_class.twist.linear.y= y 
    local_velocity_class.twist.linear.z= z
    pub_velocity_localy.publish(local_velocity_class)



def avoidance_tfmini(msg):
    tfmini_range= msg.ranges[0]                #for tfmini plus only one point ranges
    #print(f"tfmini_range = {tfmini_range}")
    if  float(.40) <  tfmini_range < float(3):
        print("\n💢️obstacle detected")
        dis_to_avoid = tfmini_range - 3
        set_velocity_localy(0, dis_to_avoid, 0)
        if True:
            print("\n*obstacle avoided successfully*")
      
            
if __name__ == "__main__" :
        rospy.init_node('obstacle_avoidance', anonymous=True)
        #sub = rospy.Subscriber('/spur/laser/scan', LaserScan, avoidance_tfmini,queue_size = 1 )
        pub_move_localy = rospy.Publisher("/mavros/setpoint_position/local",PoseStamped, queue_size = 10 )
        pub_velocity_localy = rospy.Publisher("/mavros/setpoint_velocity/cmd_vel",TwistStamped, queue_size = 10 )  
        

        rate = rospy.Rate(2)                                                          
        setmode("GUIDED")   
        arming()   
        time.sleep(5) 
        takeoff(2)
        time.sleep(10)
        move_localy(0, 0, 2)
        time.sleep(10)        
        sub = rospy.Subscriber('/spur/laser/scan', LaserScan, avoidance_tfmini)
        
        rospy.spin()
        #time.sleep(15)               

