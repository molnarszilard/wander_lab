# Gmapping for Pioneer P3-AT

This is a short  description for runnging gmapping/slam/amcl for pioneer 3-at robot, with jetson etc. This is just an overview, you might need to learn more things from elsewhere, but by default it should be possible to slightly modify the code for a different robot setup.
TODO: check the community implementations for ROS2.

## Install

https://github.com/ros-perception/slam_gmapping 

returns errors at catkin_make? 

 

instal with apt: 

```bash
sudo apt install ros-melodic-gmapping 
sudo apt install ros-melodic-slam-gmapping 
sudo apt install ros-melodic-openslam-gmapping 
sudo apt install ros-melodic-map-server 
```
 

## Create a map 

### Session 1 (simplifications in session 4) 

Terminal1:
```bash
roscore 
```

T2 - start the lidar:
```bash
rosrun sicktoolbox_wrapper sicklms _port:=/dev/ttyUSB0 _baud:=38400 _connect_delay:=30
```

T3 - start the robot base:
```bash
rosrun rosaria RosAria _port:=/dev/ttyUSB1
```

T4 - start the remote controller node:
```bash
rosrun teleop_twist_keyboard teleop_twist_keyboard.py /cmd_vel:=/RosAria/cmd_vel
```

(maybe needed before T5: `rosparam set use_sim_time true`) 

T5 - create a tf transform from the robot base to the lidar:
```bash
rosrun tf2_ros static_transform_publisher 0.16 0 0.16 0 0 0 1 base_link laser
```

T6 - start the scanner, this reads the 2D lidar scan and the odometry:
```bash
rosrun gmapping slam_gmapping scan:=/scan  odom_frame:=odom
```

(no _ before parameters) 

T7 - the previous node will publish the map on `/map`, which you need to save as a bag:
```bash
rosbag record /map
``` 

 - Navigate around with the robot (teleop), and leave a little bit of time at the end, when it records but the robot does not move anymore, then stop all the nodes

 - Now the bag contains the map, to get it as image and yaml: 

 
### Session2 

Terminal1:
```bash
roscore 
```

T2 - save the map, run this again near the end of the bag:
```bash
rosrun map_server map_saver 
```

T3 - play the bag:
```bash
rosbag play abc.bag
```

at the end of the bag run the command in T2 again 

 

### Session 3 - amcl localization 

run session 1, with T1-T5, then: 

T6 - publish the map:
```bash
rosrun map_server map_server map.yaml
```

T7 - run the localization server:
```bash
rosrun amcl amcl scan:=/scan map:=/map use_map_topic:=true
```

then in rviz you can see an approximation of the robot pose on the map (as you move the robot, the pose should change -- althought it is very insensitive, pose changes only after larger movements) 

 

### Session4 - Final Navigation stack 

http://wiki.ros.org/navigation 

http://wiki.ros.org/navigation/Tutorials/RobotSetup 

 

Simplify things by creating roslaunch files: 

 

 - pioneer_nav_configuration.launch

```
<launch> 
   <node pkg="sicktoolbox_wrapper" type="sicklms" name="sicklms" output="screen"> 
       <param name="port" value="/dev/ttyUSB0" /> 
       <param name="baud" value="38400" /> 
       <param name="connect_delay" value="30" /> 
   </node> 

   <node pkg="rosaria" type="RosAria" name="RosAria" output="screen"> 
       <param name="port" value="/dev/ttyUSB1" /> 
   </node> 

   <node pkg="tf2_ros" type="static_transform_publisher" name="link1_broadcaster" args="0.16 0 0.16 0 0 0 1 base_link laser" /> 

</launch> 
 ```

 - move_base.launch

```
<launch>
   <master auto="start"/> 

<!-- Run the map server  --> 
   <node name="map_server" pkg="map_server" type="map_server" args="$(find pioneer_nav)/map.yaml"/> 

<!--- Run AMCL --> 
   <include file="$(find amcl)/examples/amcl_omni.launch" /> 

   <node pkg="move_base" type="move_base" respawn="false" name="move_base" output="screen"> 
       <rosparam file="$(find pioneer_nav)/config/costmap_common_params.yaml" command="load" ns="global_costmap" /> 
       <rosparam file="$(find pioneer_nav)/config/costmap_common_params.yaml" command="load" ns="local_costmap" /> 
       <rosparam file="$(find pioneer_nav)/config/local_costmap_params.yaml" command="load" /> 
       <rosparam file="$(find pioneer_nav)/config/global_costmap_params.yaml" command="load" /> 
       <rosparam file="$(find pioneer_nav)/config/base_local_planner_params.yaml" command="load" /> 
       <remap from="/cmd_vel" to="/RosAria/cmd_vel"/> 
   </node> 


</launch> 
```
 

Create some config files (can be inside config folder, otherwise rewrite move_base.launch): 

 - base_local_planner_params.yaml

```
TrajectoryPlannerROS: 
 max_vel_x: 0.25 
 min_vel_x: 0.1 
 max_vel_theta: 1.0 
 min_in_place_vel_theta: 0.4 

 acc_lim_theta: 3.2 
 acc_lim_x: 2.5 
 acc_lim_y: 2.5 

 holonomic_robot: true 
```

 - global_costmap_params.yaml

```
global_costmap: 
 global_frame: map 
 robot_base_frame: base_link 
 update_frequency: 5.0 
 static_map: true 
```
 
 - costmap_common_params.yaml

```
obstacle_range: 4.0 
raytrace_range: 5.0 
footprint: [[0.25, 0.25], [0.25, -0.25], [-0.25, -0.25], [-0.25, 0.25]] 
#robot_radius: ir_of_robot 
inflation_radius: 0.5 
observation_sources: laser_scan_sensor point_cloud_sensor 
laser_scan_sensor: {sensor_frame: laser, data_type: LaserScan, topic: /scan, marking: true, clearing: true} 
# point_cloud_sensor: {sensor_frame: frame_name, data_type: PointCloud, topic: topic_name, marking: true, clearing: true} 
```

 - local_costmap_params.yaml

```
local_costmap: 
 global_frame: odom 
 robot_base_frame: base_link 
 update_frequency: 5.0 
 publish_frequency: 2.0 
 static_map: false 
 rolling_window: true 
 width: 9.0 
 height: 9.0 
 resolution: 0.05 
```

run the code: 

T1 - start the robot base:
```bash
roslaunch pioneer_nav pioneer_nav_configuration.launch
```

T2 - start the nav:
```bash
roslaunch pioneer_nav move_base.launch
```

T3 - open rviz and add map, footprint, plan(TODO: config so you do not need to add every topic by hand):
```bash
rviz
```

Set goal: 

 - RVIZ: to pbar - click on 2D Nav Goal, and click on map where you want to go (while clicking you also decide the orientation, hold the click, and set the vector direction) 

 - publish on topic with a hardcoded position: !Be careful when setting coordinates, they should be on the map! 

```
#!/usr/bin/env python 

import rospy 
from geometry_msgs.msg import PoseStamped 

def talker(): 
   pub = rospy.Publisher('move_base_simple/goal', PoseStamped, queue_size=1) 
   rospy.init_node('talker', anonymous=True) 
   rate = rospy.Rate(5) # 10hz 
   while not rospy.is_shutdown(): 
       goal = PoseStamped() 
       goal.header.seq = 1 
       goal.header.stamp = rospy.Time.now() 
       goal.header.frame_id = "map" 

       goal.pose.position.x = 0.5 
       goal.pose.position.y = -0.5 
       goal.pose.position.z = 0.0 

       goal.pose.orientation.x = 0.0 
       goal.pose.orientation.y = 0.0 
       goal.pose.orientation.z = 0.0 
       goal.pose.orientation.w = 1.0 
       pub.publish(goal) 
       rate.sleep() 

if __name__ == '__main__': 
   try: 
       talker() 
   except rospy.ROSInterruptException: 
       pass 
```
 