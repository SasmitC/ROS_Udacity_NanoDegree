[![Udacity - Robotics NanoDegree Program](https://s3-us-west-1.amazonaws.com/udacity-robotics/Extra+Images/RoboND_flag.png)](https://classroom.udacity.com/nanodegrees/nd209/dashboard/overview)

# Project 2 - Go Chase It!
This project introduces the student to write ROS Nodes in C++ and explore the various powerful features ROS and Gazebo has to offer. This project demonstrates the ability to work with ROS nodes in C++. It implements service and client servers and teaches how to write publisher and subscriber nodes. This project is divided into two major tasks - 
1. House own robot in Gazebo world environment: Learn how to build and customize own robot model using Gazebo Model Editor and then house this robot inside the **MyOfficeWorld.world** world environment created in the previous project.

2. Program the robot to chase a white coloured ball inside the world environment: Learn how to write two ROS Nodes
+ **drive_bot.cpp** - provides a service to navigate the robot through the world environment
+ **process_image.cpp** - subscribes to the camera image, searches and detects a white ball in the robot's field of view. When the ball's position is identified, this node will send a request to the service provided by _drive_bot.cpp_ node which will in turn direct the robot towards the white ball.


## Table of Contents

   * [Requirements](#requirements)
   * [How to use](#how-to-use)
   * [Directory Structure](#directory-structure)
   * [Implementation](#implementation)
   * [Future Work](#future-work)
   * [License](#license)
   * [Contribution](#contribution)


### Requirements
1. Linux OS or a Virtual Machine for Windows/Mac OS
2. Github
3. C++
4. ROS kinetic version
5. Gazebo
6. Rviz
7. Understanding of ROS service, client, publisher and subscriber nodes
8. Understanding of Robotics at fundamental level
9. Understanding of image processing at fundamental level

### How to use


### Directory Structure


### Implementation


### Future Work


### License


### Contribution
