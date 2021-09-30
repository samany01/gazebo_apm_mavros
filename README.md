# gazebo_apm_mavros

it is (gazebo models and worlds + ardupilot + my_mavros_api) in order to communicate and control drone to accomplishe a specifice mssions.
## install gazebo 11 from source
http://gazebosim.org/tutorials?tut=install_from_source&cat=install

## install ardupilot from source
https://ardupilot.org/dev/docs/building-setup-linux.html

## install ros from source
http://wiki.ros.org/noetic/Installation/Ubuntu

## install mavros/mavlink from source
https://github.com/mavlink/mavros/blob/master/mavros/README.md#installation

## download reposity 
1- open catkin_ws/src on terminal

2- clone my reposity 

3- cd catkin_ws

4- catkin build

5- gedit ~/.bashrc

6- at the end add these lines

** ros path 

  export ROS_PACKAGE_PATH=/catkin_ws/src/gazebo_apm_mavros/:$ROS_PACKAGE_PATH
  
  export ROS_PACKAGE_PATH=/catkin_ws/src/mavros/mavros:$ROS_PACKAGE_PATH
  
** gazebo path 

  export GAZEBO_RESOURCE_PATH=~/catkin_ws/src/gazebo_apm_mavros/worlds:${GAZEBO_RESOURCE_PATH}
  
  export GAZEBO_MODEL_PATH=~/catkin_ws/src/gazebo_apm_mavros/models:${GAZEBO_MODEL_PATH}
  
  export GAZEBO_PLUGIN_PATH=~/catkin_ws/src/gazebo_apm_mavros/build:${GAZEBO_PLUGIN_PATH}
  
  export GAZEBO_PLUGIN_PATH=~/catkin_ws/src/gazebo_apm_mavros/src:${GAZEBO_PLUGIN_PATH}
  
7- ctrl+s and close it

8- source ~/.bashrc

9- open scripts file on terminal

10- chmod +x obstacle_avoidance.py             # to give it permission to run you will make this for all python modules #  

11- tip:- open apm.launch file from mavros and change FCU_URL connection to udp like "udp: //: 14550 @ 192.168.1.130@5760"

          or look at http://wiki.ros.org/mavros
          
## Show Time
### open 4 terminals add each add these lines in order and wait until being intiated
- first : roslaunch gazbo obstacle_avoidance.launch

- second : sim_vehicle.py -v ArduCopter -f gazebo-iris -I0

- third : roslaunch mavros  apm.launch

- fourth : rosrun gazbo obstacle_avoidance.py


##                                                obstacle_avoidance onepoint lidar simulation

## Screen shot
![Screenshot 2021-09-26 211112](https://user-images.githubusercontent.com/77525029/134928960-77ba02a6-bdd1-4a8e-936f-2a5d181accfe.png)

## Video link
https://www.youtube.com/watch?v=CG5Po6Zi3hM

# Move_nesw_localy
- first : roslaunch gazbo grass.launch

- second : sim_vehicle.py -v ArduCopter -f gazebo-iris -I0

- third : roslaunch mavros  apm.launch

- fourth : rosrun gazbo move_nesw_localy.py

## Video link
https://www.youtube.com/watch?v=wuW9swpovKI
