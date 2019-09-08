#!/bin/sh
export TURTLEBOT_3D_SENSOR=kinect
xterm -e "source /opt/ros/kinetic/setup.bash; roscore" & 
sleep 5
xterm -e "roslaunch turtlebot_gazebo turtlebot_world.launch world_file:=/home/workspace/catkin_ws/src/home_service/src/worlds/MyOfficeWorld.world" &
sleep 5
xterm -e "roslaunch turtlebot_gazebo amcl_demo.launch map_file:=/home/workspace/catkin_ws/src/home_service/src/maps/map.yaml" & 
sleep 5
xterm -e "roslaunch home_service home_service.launch" &
sleep 5
xterm -e "rosrun home_service_add_markers home_service_add_markers_node" &
sleep 5
xterm -e "rosrun home_service_pick_objects home_service_pick_objects_node" &
