# ROS_Udacity_NanoDegree
This repository comprises of the projects one needs to work on while pursuing the Robotics Software Engineering Nano Degree from Udacity. It is a great kick-starter for ROS beginners and introduces you to ROS &amp; Gazebo fundamentals. Furthermore it helps a student build on these fundamentals and advance to challenging problems of localization, navigation, mapping, SLAM and path planning algorithms. The programming language used is C++.

[![HomePage](Robotics_Udacity_ND.jpg)](https://www.youtube.com/watch?v=yl5ca8fDLLY "Robotics Software Engineer Nanodegree Program")

#Contents:-

##Project 1 - Build My World
The endeavour of this initial project is to familiarise the student with ROS and Gazebo simulation environment, one of the most popular simulation engine used by roboticists across the globe. Here, I hace created my custom office gazebo simulation environment, which will be the arena for the future projects.

##Project 2 - Go Chase It!
This project introduces the student to creating thier own ROS Nodes in C++. I have housed my own robot inside my office environment and placed a white ball randomly. The robot has to track, chase and navigate towards the white ball using a camera sensor until the white ball lies in its field of view. If the ball does not lie in the field of view, the robot should stop.

##Project 3 - Where Am I
This project introduces the student to the Adaptive Monte Carlo Localization package and in general the concept of localization used by autonomous robots for perceiving the surroundings with the help of sensor measurements. Gaussian filters are commonly used to estimate the values from the noisy sensor data. This package allows the robot to estimate its position with respect to known map the environment. I have successfully created a map of my office environment.

##Project 4 - Map My World
The goal of the project is to deploy mapping algorithms and localization packages to simultaneously map the surroundings as well as localize the robot in its mapped environment. This project combines the concepts from the previous project on localization. I have mapped my office environment and localized my robot with respect to the generated map.

##Project 5 - Home Service Robot
This is the final challenge of the Nano Degree. This project builds on all the concepts taught in the previous project lessons and combines them to design an autonomously navigating home service robot. This project introduces the navigation algorithms and implements the SLAM algorithms from the previous projects to instruct the robot to pick up and deliver virtual items in the simulated office environment.
