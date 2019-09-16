[![Udacity - Robotics NanoDegree Program](https://s3-us-west-1.amazonaws.com/udacity-robotics/Extra+Images/RoboND_flag.png)](https://classroom.udacity.com/nanodegrees/nd209/dashboard/overview)

# Project 4 - Map My World
Welcome to the Mapping and SLAM module. Here, you will learn how the robots build the map of their surrounding environment and perform Simultaneous Localization and Mapping (SLAM). As you recall from the previous project, Localization is the task for the robot to estimate its pose given the map of the environment. However, in many real circumstances, the map may not be known prior either because the area is unexplored or the surroundings change often and the map may not be up to date. That leads us to Mapping, which involves the task of producing the map of the environment given the robot's pose and trajectory. But here's the catch - often in practical and real world problems, you neither have a map nor know the robot's pose. This is when SLAM comes to the rescue!

With limited access only to robot's own movement (odometry) data and sensor measurements, the robot must build the map of its environment while simultaneously localizing itself relative to the map. Although robot mapping sounds similar to localization, it assumes an old path and estimates the configuration of the environment as opposed to a given known map and estimating the environment, as we saw in the previous localization project. While the robot's pose has a finite-dimensional space, namely x and y coordinates and orientation angle theta, the map generally lies in a continuous space and as a result, there are infinitely many variables to describe it. Along with uncertainty in the perception using sensors and the nature of the configuration space, mapping becomes a very challenging task. Here, you will be introduced to the **Occupancy Grid Mapping**.

In SLAM, the robot must build a map of its environment while simultaneously localizing itself relative to this generated map. This task is more challenging than localization and mapping because neither the map nor the robot's pose is provided. Due to the noise present in the sensor measurements, the map and the robot's pose will be uncertain and the errors in the robot's pose estimates and the map will be correlated. SLAM is often called as the _Chicken or the Egg?_ problem! The accuracy of the map depends on the accuracy of the localization and vice versa. But the map is required for localization and the robot's pose is needed for mapping! However, SLAM is fundamental to robotics. Some of the potential applications such as self-driving vehicles, a rover exploring on the surface of Mars, a vacuum cleaner robot etc. all need an accurate SLAM algorithm to perform tasks in an unknown environment.

SLAM algorithms can be categorized as -
    + Extended Kalman Filter SLAM (EKF)
    + Sparse Extended Information Filter (SEIF)
    + Extended Information Form (EIF)
    + **FastSLAM**
    + **GraphSLAM**

In this module, you will be dealing with the **FastSLAM** algorithm, which is based on the particle filter approach combined with a low dimensional EKF to solve the SLAM problem. You will adopt this to _Grid Maps_, resulting in **Grid Based fastSLAM** algorithm. Moreover, you will learn **GraphSLAM**, which uses constraints to represent the relationships between the robot poses and the environment to generate a most likely map given the measurement data. Another flavour of _GraphSLAM_, known as the **Real Time Appearance Based Mapping** or **RTABMAP** will be used later in the project.


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
1. Code the Occupancy Grid Mapping Algorithm in C++:
    + 

### Directory Structure

### Implementation

### Future Work

### License

### Contribution
