[![Udacity - Robotics NanoDegree Program](https://s3-us-west-1.amazonaws.com/udacity-robotics/Extra+Images/RoboND_flag.png)](https://classroom.udacity.com/nanodegrees/nd209/dashboard/overview)

# Project 3 - Where Am I
Localization is a challenging problem of determining a robot's position and orientation (together known as its pose) in a mapped environment. Usually, this is achieved by implementing a probabilistic algorithm to filter noisy sensor measurements and track the robot's pose. This project introduces two common localization algorithms, namely, the Extended Kalman Filter (EKF) localization and the Monte Carlo localization (MCL), also known as the particle filter algorithm. The multidimensional EKF helps in estimating the state variables comprising of the kinematic parameters for non-linear motion models. Whereas the particle filter uses randomly sampled values of the state known as particles to estimate the robot's state with respect to its environment. Due to the prevailing uncertainty in the sensor measurements, identifying the pose of the robot relative to its environment is prone to errors. This makes localization a non-trivial task. Here, a static environment is used for the purpose of ground truth measurement.

The project starts with a C++ implementatoion of the Extended Kalman Filter (EKF) and introduces the EKF package in ROS known as the _robot_pose_ekf_. This package combines all the noisy measurements, compares the data generated from the robot's onboard sensors such as the inertial measurement unit (IMU), rotary encoder and vision sensors viz. camera and lidar, filters the noise and removes the uncertainties. It further applies sensor fusion by combining the data from all these sensors in order to better estimate the robot's pose as it moves around. This project also implements the _turtlebot_gazebo_, _odom_to_trajectory_, _turtlebot_teleop_ and the _rviz_ package. It integrates all nodes of the above five packages into a single _launch_ file.

Next, the project implements the Monte Carlo Localization (MCL) algorithm for localizing the robot in its environment. With the particle filter, this algorithm offers potential advantages over EKF algorithm. THE MCL algorithm involves the motion and sensor update followed by the resampling step. As the MCL is an iterative process, the algorithm takes the previous belief obtained from randomly generated particles, the actuation input and the sensor measurements resulting into a new belief based on the new measurements from the sensors and the predection of the motion model. This project provides a glimpse into C++ code of the MCL algorithm. Further, the project implements the Adaptive Monte Carlo Localization (AMCL) package in ROS. Finally, the robot should be able to localize itself in the simulated environment.

## Table of Contents

   * [Requirements](#requirements)
   * [How to use](#how-to-use)
   * [Directory Structure](#directory-structure)
   * [Implementation](#implementation)
   * [Future Work](#future-work)
   * [License](#license)
   * [Contribution](#contribution)

### Requirements

### How to use

### Directory Structure

### Implementation

### Future Work

### License

### Contribution
