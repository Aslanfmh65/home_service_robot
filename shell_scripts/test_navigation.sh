#!/bin/sh
xterm  -e  " roslaunch turtlebot_gazebo turtlebot_world.launch world_file:=/home/robond/Desktop/myrobot/catkin_ws/src/maps/home_service_robot.world " &
sleep 5
xterm  -e  " roslaunch turtlebot_gazebo amcl_demo.launch map_file:=/home/robond/Desktop/myrobot/catkin_ws/src/maps/my_map.yaml 3d_sensor:=kinect " &
sleep 5
xterm  -e  " roslaunch turtlebot_rviz_launchers view_navigation.launch "
