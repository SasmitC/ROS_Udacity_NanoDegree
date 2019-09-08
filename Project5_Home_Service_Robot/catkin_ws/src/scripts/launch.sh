#!/bin/sh
xterm -e "source /opt/ros/kinetic/setup.bash; roscore" & 
sleep 5
xterm  -e  "gazebo" &
sleep 5
xterm  -e  "rosrun rviz rviz"
