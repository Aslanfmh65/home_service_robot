#!/bin/sh
xterm  -e  " roslaunch turtlebot_gazebo turtlebot_world.launch world_file:=/home/robond/Desktop/myrobot/catkin_ws/src/maps/home_service_robot.world " &
sleep 5
xterm  -e  " rosrun gmapping slam_gmapping " &
sleep 5
xterm  -e  " roslaunch turtlebot_rviz_launchers view_navigation.launch " &
sleep 5
xterm  -e  " roslaunch turtlebot_teleop keyboard_teleop.launch "
