#!/bin/sh
export TURTLEBOT_3D_SENSOR=kinect
xterm -e "source /opt/ros/kinetic/setup.bash; roscore" & 
sleep 5
xterm -e "roslaunch turtlebot_gazebo turtlebot_world.launch world_file:=/home/workspace/catkin_ws/src/test_home_service/world/my_world/test_world.world" &
sleep 5
xterm -e "roslaunch turtlebot_gazebo gmapping_demo.launch" &
sleep 5
xterm -e "roslaunch turtlebot_rviz_launchers view_navigation.launch" &
sleep 5
xterm -e "roslaunch turtlebot_teleop keyboard_teleop.launch" &
