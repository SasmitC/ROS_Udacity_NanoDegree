[![Udacity - Robotics NanoDegree Program](https://s3-us-west-1.amazonaws.com/udacity-robotics/Extra+Images/RoboND_flag.png)](https://classroom.udacity.com/nanodegrees/nd209/dashboard/overview)

# Project 3 - Where Am I
Localization is a challenging problem of determining a robot's position and orientation (together known as its pose) in a mapped environment. Usually, this is achieved by implementing a probabilistic algorithm to filter noisy sensor measurements and track the robot's pose. This project introduces two common localization algorithms, namely, the Extended Kalman Filter (EKF) localization and the Monte Carlo localization (MCL), also known as the particle filter algorithm. The multidimensional EKF helps in estimating the state variables comprising of the kinematic parameters for non-linear motion models. Whereas the particle filter uses randomly sampled values of the state known as particles to estimate the robot's state with respect to its environment. Due to the prevailing uncertainty in the sensor measurements, identifying the pose of the robot relative to its environment is prone to errors. This makes localization a non-trivial task. Here, a static environment is used for the purpose of ground truth measurement.

The project starts with a C++ implementation of the Extended Kalman Filter (EKF) and introduces the EKF package in ROS known as the _robot_pose_ekf_. This package combines all the noisy measurements, compares the data generated from the robot's onboard sensors such as the inertial measurement unit (IMU), rotary encoder and vision sensors viz. camera and lidar, filters the noise and removes the uncertainties. It further applies sensor fusion by combining the data from all these sensors in order to better estimate the robot's pose as it moves around. This project also implements the _turtlebot_gazebo_, _odom_to_trajectory_, _turtlebot_teleop_ and the _rviz_ package. It integrates all nodes of the above five packages into a single _launch_ file.

Next, the project implements the Monte Carlo Localization (MCL) algorithm for localizing the robot in its environment. With the particle filter, this algorithm offers potential advantages over the EKF algorithm. THE MCL algorithm involves the motion and sensor update followed by the resampling step. Since the MCL is an iterative process, the algorithm takes the previous belief obtained from randomly generated particles, the actuation input and the sensor measurements and yields a new belief based on the new measurements from the sensors and the prediction step involving the motion model. This project provides an overview of the MCL algorithm in C++. Furthermore, the project implements the Adaptive Monte Carlo Localization (AMCL) package in ROS. Finally, the robot should now be able to localize itself in the simulated environment.


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
6. Understanding of [conditional probability](https://en.wikipedia.org/wiki/Conditional_probability), [law of total probability](https://en.wikipedia.org/wiki/Law_of_total_probability), [Bayesian statistics](https://en.wikipedia.org/wiki/Bayesian_statistics) and [Bayes' rule](https://en.wikipedia.org/wiki/Bayes%27_theorem).
7. Understanding of [Robotics](https://en.wikibooks.org/wiki/Robotics), state variables and motion model
8. Understanding of [Kalman Filter](https://en.wikipedia.org/wiki/Kalman_filter), [Extended Kalman Filter](https://en.wikipedia.org/wiki/Extended_Kalman_filter) and [Sensor Fusion](https://en.wikipedia.org/wiki/Sensor_fusion)
9. Understanding of [Monte Carlo localization](https://en.wikipedia.org/wiki/Monte_Carlo_localization) and [particle filters](https://en.wikipedia.org/wiki/Particle_filter)


### How to use
1. Set up the ```catkin_ws```: Create the Catkin Workspace - 
```sh
$ mkdir -p /home/workspace/catkin_ws/src
$ cd ~/catkin_ws/src
$ catkin_init_workspace
$ cd ..
$ catkin_make
```

2. Clone, build and launch the ```turtlebot_gazebo``` package:
+ Clone the ```turtlebot_gazebo``` package in to the ```src``` directory created in step #1. Go through [this link](http://wiki.ros.org/turtlebot_gazebo) and its tutorials to get familiar with the package -
```sh
$ cd ~/catkin_ws/src
$ git clone https://github.com/turtlebot/turtlebot_simulator
```
+ Install Dependencies -
```sh
$ cd ~/catkin_ws
$ source devel/setup.bash
$ rosdep -i install turtlebot_gazebo
```
+ Build Package -
```sh
$ catkin_make
$ source devel/setup.bash
```
+ Launch Nodes - 
```sh
$ roslaunch turtlebot_gazebo turtlebot_world.launch
```
+ Topics -
```sh
$ rostopic list
Or
$ rosrun rqt_graph rqt_graph
```

2. Clone, build and launch the ```robot_pose_ekf``` package:
+ Install the package -
```sh
$ cd ~/catkin_ws/src/
$ git clone http://wiki.ros.org/robot_pose_ekf
```
+ Edit the ```robot_pose_ekf.launch``` file as below -
```xml
<launch>

<node pkg="robot_pose_ekf" type="robot_pose_ekf" name="robot_pose_ekf">
  <param name="output_frame" value="odom_combined"/>
  <param name="base_footprint_frame" value="base_footprint"/>
  <param name="freq" value="30.0"/>
  <param name="sensor_timeout" value="1.0"/>  
  <param name="odom_used" value="true"/>
  <param name="imu_used" value="true"/>
  <param name="vo_used" value="false"/>

  <remap from="imu_data" to="/mobile_base/sensors/imu_data" />    

</node>

</launch>
```


### Directory Structure

### Implementation

### Future Work

### License

### Contribution
