#!/bin/sh
export TURTLEBOT_3D_SENSOR=kinect
xterm -e "source /opt/ros/kinetic/setup.bash; roscore" & 
sleep 5
xterm -e  "roslaunch turtlebot_gazebo turtlebot_world.launch world_file:=/home/workspace/catkin_ws/src/test_home_service/world/my_world/test_world.world" &
sleep 5
xterm -e  "roslaunch turtlebot_gazebo amcl_demo.launch map_file:=/home/workspace/catkin_ws/src/test_home_service/maps/map.yaml" & 
sleep 5
xterm -e "roslaunch test_home_service test_home_service.launch" &
sleep 5
xterm -e "rosrun add_markers add_markers_node" &
sleep 5
xterm -e "rosrun pick_objects pick_objects_node" &

