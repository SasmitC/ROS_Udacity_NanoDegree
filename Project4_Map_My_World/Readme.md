[![Udacity - Robotics NanoDegree Program](https://s3-us-west-1.amazonaws.com/udacity-robotics/Extra+Images/RoboND_flag.png)](https://classroom.udacity.com/nanodegrees/nd209/dashboard/overview)

# Project 4 - Map My World
Welcome to the Mapping and SLAM module! Here, you will learn how the robots build the map of their surrounding environment and perform Simultaneous Localization and Mapping (SLAM). As you recall from the previous project, Localization is the task for the robot to estimate its pose given the map of the environment. However, in many real circumstances, the map may not be known prior either because the area is unexplored or the surroundings change often and the map may not be up to date. That leads us to Mapping, which involves the task of producing the map of the environment given the robot's pose and trajectory. But here's the catch - often in practical and real world problems, you neither have a map nor know the robot's pose. This is when SLAM comes to the rescue!

With limited access only to robot's own movement (odometry) data and sensor measurements, the robot must build the map of its environment while simultaneously localizing itself relative to the map. Although robot mapping sounds similar to localization, it assumes an old path and estimates the configuration of the environment as opposed to a given known map and estimating the environment, as we saw in the previous localization project. While the robot's pose has a finite-dimensional space, namely x and y coordinates and orientation angle theta, the map generally lies in a continuous space and as a result, there are infinitely many variables to describe it. Along with uncertainty in the perception using sensors and the nature of the configuration space, mapping becomes a very challenging task. Here, you will be introduced to the **Occupancy Grid Mapping**.

In SLAM, the robot must build a map of its environment while simultaneously localizing itself relative to this generated map. This task is more challenging than localization and mapping because neither the map nor the robot's pose is provided. Due to the noise present in the sensor measurements, the map and the robot's pose will be uncertain and the errors in the robot's pose estimates and the map will be correlated. SLAM is often called as the _Chicken or the Egg?_ problem! The accuracy of the map depends on the accuracy of the localization and vice versa. But the map is required for localization and the robot's pose is needed for mapping! However, SLAM is fundamental to robotics. Some of the potential applications such as self-driving vehicles, a rover exploring on the surface of Mars, a vacuum cleaner robot etc. all need an accurate SLAM algorithm to perform tasks in an unknown environment.

SLAM algorithms can be categorized as:
+ Extended Kalman Filter SLAM (EKF)
+ Sparse Extended Information Filter (SEIF)
+ Extended Information Form (EIF)
+ **FastSLAM**
+ **GraphSLAM**

In this module, you will be dealing with the **FastSLAM** algorithm, which is based on the particle filter approach combined with a low dimensional EKF to solve the SLAM problem. You will adopt this to _Grid Maps_, resulting in **Grid Based fastSLAM** algorithm. Moreover, you will learn **GraphSLAM**, which uses constraints to represent the relationships between the robot poses and the environment to generate a most likely map given the measurement data. Another flavour of _GraphSLAM_, known as the **Real Time Appearance Based Mapping** or **RTABMAP** will be used later in the project.

For viewing the output, jump directly to [Results](#results)

## Table of Contents

   * [Requirements](#requirements)
   * [How to use](#how-to-use)
   * [Directory Structure](#directory-structure)
   * [Project Implementation](#project-implementation)
   * [Results](#results)
   * [Future Work](#future-work)
   * [License](#license)
   * [Contribution](#contribution)


### Requirements
1. Linux OS or a Virtual Machine for Windows/Mac OS
2. Github
3. C++
4. ROS kinetic version
5. Gazebo
6. Understanding of [probabilistic robotics](http://www.probabilistic-robotics.org/), [posterior probability](https://en.wikipedia.org/wiki/Posterior_probability) and [log probability](https://en.wikipedia.org/wiki/Log_probability)
7. Understanding of [SLAM](https://en.wikipedia.org/wiki/Simultaneous_localization_and_mapping) and [robot navigation](https://en.wikipedia.org/wiki/Robot_navigation)
8. Understanding to 2D and 3D Mapping, [Occupance Grid Mapping](https://en.wikipedia.org/wiki/Occupancy_grid_mapping), its [ROS packages](http://wiki.ros.org/octomap) for eg. [Octomap](http://octomap.github.io/)
9. Understanding of different SLAM algorithms like [FastSLAM](http://ais.informatik.uni-freiburg.de/teaching/ws12/mapping/pdf/slam10-fastslam.pdf), [GraphSLAM](http://www2.informatik.uni-freiburg.de/~stachnis/pdf/grisetti10titsmag.pdf), [Grid based FastSLAM](http://ais.informatik.uni-freiburg.de/teaching/ws12/mapping/pdf/slam13-gridfastslam.pdf), [gmapping ROS package](http://wiki.ros.org/gmapping)
10. Understanding of [Real-Time Appearance-Based Mapping (RTAB-Map)](https://arxiv.org/pdf/1809.02989), [RTABMap in ROS](http://www.theconstructsim.com/construct-learn-develop-robots-using-ros/robotigniteacademy_learnros/ros-courses-library/rtab-map-in-ros-101/), [ROS package rtabmap](http://wiki.ros.org/rtabmap_ros)


### How to use
1. Code the Occupancy Grid Mapping Algorithm in C++: The robot is equipped with **eight sonar rangefinder sensors**. It navigates around in the environment to generate its map. This mini-prpject has been organized as follows -
    + Data
        - ```measurement.txt```: The measurements from the sonar rangefinder sensors attached to the robot at each time stamp recorded over a period of 413 seconds (timestamp, measurement 1:8).
        - ```poses.txt```: The exact robot poses at each timestamp recorded over a period of 413 seconds (timestamp, x, y, ϴ).
    + Global Functions
        - ```inverseSensorModel()```: To compute ```r``` and ```phi``` and evaluate the three different cases of the algorithm.
        
          ![inverseSensorModel](inverseSensorModel.png)
          
        - ```occupancyGridMapping()```: The core algorithm
        
          ![occupancy_grid_algorithm](occupancygridalgorithm.png)
          
    + Main Function
        - ```File Scan```: To scan both the measurement and poses files to retrieve the values. At each time stamp, the values are passed to the occupancy grid mapping function.
        - ```Display Map```: To display the generated map after processing all the measurements and poses.
        
    
```cpp
#include <iostream>
#include <math.h>
#include <vector>
#include "src/matplotlibcpp.h" //Graph Library

using namespace std;
namespace plt = matplotlibcpp;

// Sensor characteristic: Min and Max ranges of the beams
double Zmax = 5000, Zmin = 170;
// Defining free cells(lfree), occupied cells(locc), unknown cells(l0) log odds values
double l0 = 0, locc = 0.4, lfree = -0.4;
// Grid dimensions
double gridWidth = 100, gridHeight = 100;
// Map dimensions
double mapWidth = 30000, mapHeight = 15000;
// Robot size with respect to the map 
double robotXOffset = mapWidth / 5, robotYOffset = mapHeight / 3;
// Defining an l vector to store the log odds values of each cell
vector< vector<double> > l(mapWidth/gridWidth, vector<double>(mapHeight/gridHeight));

double inverseSensorModel(double x, double y, double theta, double xi, double yi, double sensorData[])
{
    // Defining Sensor Characteristics
    double Zk, thetaK, sensorTheta;
    double minDelta = -1;
    double alpha = 200, beta = 20;

    //******************Compute r and phi**********************//
    double r = sqrt(pow(xi - x, 2) + pow(yi - y, 2));
    double phi = atan2(yi - y, xi - x) - theta;

    //Scaling Measurement to [-90 -37.5 -22.5 -7.5 7.5 22.5 37.5 90]
    for (int i = 0; i < 8; i++) {
        if (i == 0) {
            sensorTheta = -90 * (M_PI / 180);
        }
        else if (i == 1) {
            sensorTheta = -37.5 * (M_PI / 180);
        }
        else if (i == 6) {
            sensorTheta = 37.5 * (M_PI / 180);
        }
        else if (i == 7) {
            sensorTheta = 90 * (M_PI / 180);
        }
        else {
            sensorTheta = (-37.5 + (i - 1) * 15) * (M_PI / 180);
        }

        if (fabs(phi - sensorTheta) < minDelta || minDelta == -1) {
            Zk = sensorData[i];
            thetaK = sensorTheta;
            minDelta = fabs(phi - sensorTheta);
        }
    }

    //******************Evaluate the three cases**********************//
    if (r > min((double)Zmax, Zk + alpha / 2) || fabs(phi - thetaK) > beta / 2 || Zk > Zmax || Zk < Zmin) {
        return l0;
    }
    else if (Zk < Zmax && fabs(r - Zk) < alpha / 2) {
        return locc;
    }
    else if (r <= Zk) {
        return lfree;
    }
}

void occupancyGridMapping(double Robotx, double Roboty, double Robottheta, double sensorData[])
{
    //******************Code the Occupancy Grid Mapping Algorithm**********************//
    for (int x = 0; x < mapWidth / gridWidth; x++) {
        for (int y = 0; y < mapHeight / gridHeight; y++) {
            double xi = x * gridWidth + gridWidth / 2 - robotXOffset;
            double yi = -(y * gridHeight + gridHeight / 2) + robotYOffset;
            if (sqrt(pow(xi - Robotx, 2) + pow(yi - Roboty, 2)) <= Zmax) {
                l[x][y] = l[x][y] + inverseSensorModel(Robotx, Roboty, Robottheta, xi, yi, sensorData) - l0;
            }
        }
    }
}

void visualization()
{
    //Graph Format
    plt::title("Map");
    plt::xlim(0, (int)(mapWidth / gridWidth));
    plt::ylim(0, (int)(mapHeight / gridHeight));

    // Draw every grid of the map:
    for (double x = 0; x < mapWidth / gridWidth; x++) {
        cout << "Remaining Rows= " << mapWidth / gridWidth - x << endl;
        for (double y = 0; y < mapHeight / gridHeight; y++) {
            if (l[x][y] == 0) { //Green unkown state
                plt::plot({ x }, { y }, "g.");
            }
            else if (l[x][y] > 0) { //Black occupied state
                plt::plot({ x }, { y }, "k.");
            }
            else { //Red free state
                plt::plot({ x }, { y }, "r.");
            }
        }
    }

    //Save the image and close the plot
    plt::save("./Images/Map.png");
    plt::clf();
}

int main()
{
    double timeStamp;
    double measurementData[8];
    double robotX, robotY, robotTheta;

    FILE* posesFile = fopen("Data/poses.txt", "r");
    FILE* measurementFile = fopen("Data/measurement.txt", "r");

    // Scanning the files and retrieving measurement and poses at each timestamp
    while (fscanf(posesFile, "%lf %lf %lf %lf", &timeStamp, &robotX, &robotY, &robotTheta) != EOF) {
        fscanf(measurementFile, "%lf", &timeStamp);
        for (int i = 0; i < 8; i++) {
            fscanf(measurementFile, "%lf", &measurementData[i]);
        }
        occupancyGridMapping(robotX, robotY, (robotTheta / 10) * (M_PI / 180), measurementData);
    }

    // Visualize the map at the final step
    cout << "Wait for the image to generate" << endl;
    visualization();
    cout << "Done!" << endl;

    return 0;
}
```

2. Generate the map: 
    After coding the _Occupancy Grid Mapping algorithm_ in C++ and generating a map 2D vector, its time to visualize the generated map! Basically, you'll plot different cells - occupied, free and unknown cells on a graph to generate the map. Check out this [link](https://github.com/lava/matplotlib-cpp) for more information on the matplotlib C++ library. For information regarding the plot color and shape refer to the LineSpec and LineColor section of the [MATLAB documentation](https://www.mathworks.com/help/matlab/ref/plot.html?ue).
    + Compile the program -
    ```sh
    $ cd Project4_Map_My_World/OccupancyGridMapping/
    $ rm -rf Images/* #Delete the folder content and not the folder itself!
    $ g++ main.cpp -o map_app -std=c++11 -I/usr/include/python2.7 -lpython2.7
    ```
    + Finally run the program -
    ```sh
    $ ./map_app
    ```
    + Now, wait for the program to generate the map and store it in the Project4_Map_My_World/OccupancyGridMapping/Images directory. The generated map should look something like the following -
    + Map Legend:
        - ![#f03c15](https://placehold.it/15/008000/000000?text=+ "Green - Unknown/Undiscovered Zone") `Green - Unknown/Undiscovered Zone`
        - ![#f03c15](https://placehold.it/15/ff0000/000000?text=+ "Red - Free Zone") `Red - Free Zone`
        - ![#f03c15](https://placehold.it/15/000000/000000?text=+ "Black - Occupied Zone") `Black - Occupied Zone`
    
    ![generated_map](OccupancyGridMapping/Images/map.png)

    For more information on **Occupancy Grid Mapping** refer [this paper](Knowledge_Portal/InTech-Occupancy_grid_maps_for_localization_and_mapping.pdf).

    Mapping with a mobile robot equipped with more than one perception sensor such as LIDAR, RGBD Camera, Radar or Ultrasonic Sensor leads to an enormous size of the  map. But it becomes necessary to combine the data from more than sources into a single map. An intuitive approach maybe to implement Occupancy Grid Mapping Algorithm for individual sensor models. However, this will fail since each sensor has different characteristics and they respond differently to different type of obstacles. The best approach to the _multi sensor fusion_ problem is to build separate maps for each sensor independently of each other and then integrate them. By implementing _De Morgan's Law_ you can combine these individual maps with each cell denoting the combined estimated occupancy values. To obtain the most likely map, the maximum value of each cell in resultant map can be computed. Another approach is perform an OR operation between values of each cell. An illustration of _multi sensor fusion_ to generate a map is shown below -

    ```cpp
    #include <iostream>
    #include <math.h>
    using namespace std;

    const int mapWidth =  2;
    const int mapHeight = 2;

    void sensorFusion(double m1[][mapWidth], double m2[][mapWidth])
    {
        for (int x = 0; x < mapHeight; x++) {
            for (int y = 0; y < mapWidth; y++) {
                double p = 1 - (1 - m1[x][y]) * (1 - m2[x][y]);
                cout << p << " ";
            }
            cout << endl;
        }
    }

    int main()
    {

        double m1[mapHeight][mapWidth] = { { 0.9, 0.6 }, { 0.1, 0.5 } };
        double m2[mapHeight][mapWidth] = { { 0.3, 0.4 }, { 0.4, 0.3 } };
        sensorFusion(m1, m2);

        return 0;
    }
    ```

3. Implement Grid-based FastSLAM Algorithm:
    For a quick read, please refer this concise explanation on [Grid-based FastSLAM Algorithm](https://fjp.at/posts/slam/fastslam/). After going through the nitty-gritty of the algorithm, let us put it to test and simulate the results. Here, you will implement the [```gmapping``` ROS package](http://wiki.ros.org/gmapping) which is based on the **Grid-based FastSLAM Algorithm** to map the environment. ```gmapping``` provides laser and odometry based SLAM and 2D occupancy grid map of the environment. The map gets updated as the robot moves and gathers sensor data. You will learn to deploy a turtlebot inside a willow garage environment.
    
    + Create a catkin_ws in your project directory -
    ```sh
    $ mkdir -p ~/catkin_ws/src
    $ cd ~/catkin_ws/src
    $ catkin_init_workspace
    $ cd ..
    $ catkin_make
    ```
    
    + Clone the ```turtlebot_gazebo``` and ```turtlebot_teleop``` in src directory -
    ```sh
    $ cd ~/catkin_ws/src
    $ git clone https://github.com/turtlebot/turtlebot_simulator
    $ git clone https://github.com/turtlebot/turtlebot
    ```
    
    + Install the package dependencies -
    ```sh
    $ cd ..
    $ source devel/setup.bash
    $ rosdep -i install turtlebot_gazebo
    $ rosdep -i install turtlebot_teleop
    ```
    
    + Build the packages -
    ```sh
    $ catkin_make
    $ source devel/setup.bash
    ```
    
    + Clone the ```gmapping``` package, install its dependencies and build catkin workspace -
    ```sh
    $ cd ~/catkin_ws/src
    $ git clone https://github.com/ros-perception/slam_gmapping
    $ rosdep install gmapping
    $ cd..
    $ catkin_make
    ```
    
    + In terminal #1, deploy the ```turtlebot``` in a willow garage envrironment -
    ```sh
    $ cd ~/catkin_ws
    $ source devel/setup.bash
    $ roslaunch turtlebot_gazebo turtlebot_world.launch world_file:=worlds/willowgarage.world
    ```
    
    + In terminal #2, launch the ```teleop``` node -
    ```sh
    $ cd ~/catkin_ws
    $ source devel/setup.bash
    $ roslaunch turtlebot_teleop keyboard_teleop.launch
    ```
    
    + In terminal #3, run the ```slam_gmapping``` node -
    ```sh
    $ cd ~/catkin_ws
    $ source devel/setup.bash
    $ rosrun gmapping slam_gmapping
    ```
    
    + Run rviz and subscribe to the various published topics to visualize the map-
    ```sh
    $ rosrun rviz rviz
    ```
    Edit the rviz configuration as follows:
        * Change the **Fixed Frame** to ```map```
        * Keep **Reference Frame** as default
        * Add a **RobotModel**
        * Add a **camera** and select the ```/camera/rgb/image_raw``` topic
        * Add a **map** and select the ```/map``` topic
    
    + Save the map of the environment(map.pgm and map.yaml will be generated) -
    ```sh
    $ map_server map_saver -f myMap
    ```
    With the [map_saver](http://wiki.ros.org/map_server), you can load and save the maps.
    
    ![willow-garage-map](willow-garage-map.png.jpg)
    
    ![rvizOutput](Output.png)
    
    
### Directory Structure
```bash
.Project4_Map_My_World
├── catkin_ws
│   ├── Knowledge_Portal
│   │   ├── ...
│   ├── OccupancyGridMapping
│   │   ├── Data
│   │   │   ├── measurement.txt
│   │   │   ├── poses.txt
│   │   ├── Images
│   │   │   ├── map.png
│   │   ├── src
│   │   │   ├── matplotlibcpp.h
│   │   ├── main.cpp
│   ├── build
│   │   ├── ...
│   ├── devel
│   │   ├── ...
│   ├── src
│   │   ├── my_robot
│   │   │   ├── config
│   │   │   │   ├── base_local_planner_params.yaml
│   │   │   │   ├── costmap_common_params.yaml
│   │   │   │   ├── global_costmap_params.yaml
│   │   │   │   ├── local_costmap_params.yaml
│   │   │   ├── launch
│   │   │   │   ├── localization.launch
│   │   │   │   ├── mapping.launch
│   │   │   │   ├── robot_description.launch
│   │   │   │   ├── teleop.launch
│   │   │   │   ├── world.launch
│   │   │   ├── maps
│   │   │   │   ├── map.pgm
│   │   │   │   ├── map.yaml
│   │   │   ├── meshes
│   │   │   │   ├── hokuyo.dae
│   │   │   ├── rviz
│   │   │   │   ├── mapMyWorldRviz.rviz
│   │   │   ├── urdf
│   │   │   │   ├── my_robot.gazebo
│   │   │   │   ├── my_robot.xacro
│   │   │   ├── worlds
│   │   │   │   ├── MyOfficeWorld.world
│   │   │   │   ├── empty.world
│   │   │   ├── CMakeLists.txt
│   │   │   ├── package.xml
│   │   ├── teleop_twist_keyboard
│   │   │   ├── teleop_twist_keyboard.py
│   │   │   ├── README.md
│   │   │   ├── CMakeLists.txt
│   │   │   ├── package.xml
└── Readme.md
```


### Project Implementation
Welcome to the project Map My World !
    In this project, you will create a 2D occupancy grid and 3D octotmap of a simulated environment using your own robot with the _RTAB-Map_ package. [Real-Time Appearance-Based Mapping](http://wiki.ros.org/rtabmap_ros) or **RTAB-Map** is a popular solution for robotic SLAM to map the unknown environment. It has adequately high speed and memory management, moreoever, it provides GUIs for information analysis. For this project, you will be using the ```rtabmap_ros``` package, which is a ROS wrapper (API) for interacting with RTAB-Map.
    The project flow will be as follows:
    + You will develop your own package to interface with the rtabmap_ros package.
    + You will build upon your localization project to make the necessary changes to interface the robot with RTAB-Map. An example of this is the addition of an RGB-D camera.
    + You will ensure that all files are in the appropriate places, all links are properly connected, naming is properly setup and topics are correctly mapped. Furthermore you will need to generate the appropriate launch files to launch the robot and map its surrounding environment.
    + When your robot is launched you will teleop around the room to generate a proper map of the environment.

1. Simulation Setup: Similar to the previous project, you need to set up the simulation environment and the robot.
    + Grab the code from the [previous project](https://github.com/SasmitC/ROS_Udacity_NanoDegree/tree/master/Project2_Go_Chase_It/catkin_ws/src/my_robot). Create a catkin package by the same name my_robot and copy the contents of above repository into the newly created package on your local machine.

    + Do a quick ```$catkin_make``` and ```$source the devel/setup.bash``` script. Launch the world to verify if the system is good to go!
    ```sh
	  $ roslaunch my_robot world.launch
	  ```
2. Senosr Upgrade: Add an ```optical``` camera link - For RGB-D camera in URDF file, we need to add an extra link and an extra joint to the camera link in order to align the camera image in Gazebo properly with the robot. Note that the parent link of ```camera_optical_joint``` should be properly configured to the original camera link.

    + Add the following joint and link to the robot's ```.xacro``` file -
    ```xml
	  <joint name="camera_optical_joint" type="fixed">
      <origin xyz="0 0 0" rpy="-1.5707 0 -1.5707"/>
      <parent link="camera_link"/>
      <child link="camera_link_optical"/>
    </joint>

    <link name="camera_link_optical">
    </link>
	  ```
    
    + Configure the RGBD-D Camera - To do this we will need to replace the existing camera and its shared object file, namely the ```libgazebo_ros_camera.so```, to that of the Kinect shared object file, ```libgazebo_ros_openni_kinect.so```. Also, update the ```<frameName>``` to be the ```camera_link_optical``` link you created just now. Substitute the following code  in your robot's ```.gazebo``` file -
     ```xml
     <!-- RGBD Camera -->
     <gazebo reference="camera_link">
       <sensor type="depth" name="camera1">
        <always_on>1</always_on>
        <update_rate>20.0</update_rate>
        <visualize>true</visualize>             
        <camera>
            <horizontal_fov>1.047</horizontal_fov>  
            <image>
                <width>640</width>
                <height>480</height>
                <format>R8G8B8</format>
            </image>
            <depth_camera>

            </depth_camera>
            <clip>
                <near>0.1</near>
                <far>20</far>
            </clip>
        </camera>
         <plugin name="camera_controller" filename="libgazebo_ros_openni_kinect.so">
            <alwaysOn>true</alwaysOn>
            <updateRate>10.0</updateRate>
            <cameraName>camera</cameraName>
            <frameName>camera_link_optical</frameName>                   
            <imageTopicName>rgb/image_raw</imageTopicName>
            <depthImageTopicName>depth/image_raw</depthImageTopicName>
            <pointCloudTopicName>depth/points</pointCloudTopicName>
            <cameraInfoTopicName>rgb/camera_info</cameraInfoTopicName>              
            <depthImageCameraInfoTopicName>depth/camera_info</depthImageCameraInfoTopicName>            
            <pointCloudCutoff>0.4</pointCloudCutoff>                
                <hackBaseline>0.07</hackBaseline>
                <distortionK1>0.0</distortionK1>
                <distortionK2>0.0</distortionK2>
                <distortionK3>0.0</distortionK3>
                <distortionT1>0.0</distortionT1>
                <distortionT2>0.0</distortionT2>
            <CxPrime>0.0</CxPrime>
            <Cx>0.0</Cx>
            <Cy>0.0</Cy>
            <focalLength>0.0</focalLength>
            </plugin>
       </sensor>
      </gazebo>
     ```

3. Launch File: Create ```mapping.launch``` file in the ```launch``` folder. For more information on advanced parameter tuning of rtabmap_ros package refer [this Wiki page](http://wiki.ros.org/rtabmap_ros/Tutorials/Advanced%20Parameter%20Tuning) Add the following to the launch file - 
    ```xml
    <?xml version="1.0" encoding="UTF-8"?>

    <launch>
      <!-- Arguments for launch file with defaults provided -->
      <arg name="database_path"     default="/home/workspace/catkin_ws/src/database/rtabmap.db"/>
      <arg name="rgb_topic"   default="/camera/rgb/image_raw"/>
      <arg name="depth_topic" default="/camera/depth/image_raw"/>
      <arg name="camera_info_topic" default="/camera/rgb/camera_info"/>  


      <!-- Mapping Node -->
      <group ns="rtabmap">
        <node name="rtabmap" pkg="rtabmap_ros" type="rtabmap" output="screen" args="--delete_db_on_start">

          <!-- Basic RTAB-Map Parameters -->
          <param name="database_path"       type="string" value="$(arg database_path)"/>
          <param name="frame_id"            type="string" value="robot_footprint"/>
          <param name="odom_frame_id"       type="string" value="odom"/>
          <param name="subscribe_depth"     type="bool"   value="true"/>
          <param name="subscribe_scan"      type="bool"   value="true"/>
          <param name="odom_tf_angular_variance" type="double" value="0.0001"/>
          <param name="odom_tf_linear_variance" type="double" value="0.0001"/>

          <!-- RTAB-Map Inputs -->
          <remap from="scan" to="/scan"/>
          <remap from="rgb/image" to="$(arg rgb_topic)"/>
          <remap from="depth/image" to="$(arg depth_topic)"/>
          <remap from="rgb/camera_info" to="$(arg camera_info_topic)"/>

          <!-- RTAB-Map Output -->
          <remap from="grid_map" to="/map"/>

          <!-- Rate (Hz) at which new nodes are added to map -->
          <param name="Rtabmap/DetectionRate" type="string" value="1"/>

          <!-- 2D SLAM -->
          <param name="Reg/Force3DoF" type="string" value="true"/>

          <!-- Loop Closure Detection -->
          <!-- 0=SURF 1=SIFT 2=ORB 3=FAST/FREAK 4=FAST/BRIEF 5=GFTT/FREAK 6=GFTT/BRIEF 7=BRISK 8=GFTT/ORB 9=KAZE -->
          <param name="Kp/DetectorStrategy" type="string" value="0"/>
          <param name="Vis/FeatureType" type="string" value="0"/>

          <!-- Maximum visual words per image (bag-of-words) -->
          <param name="Kp/MaxFeatures" type="string" value="200"/>

          <!-- Used to extract more or less SURF features -->
          <param name="SURF/HessianThreshold" type="string" value="1000"/>

          <!-- Loop Closure Constraint -->
          <!-- 0=Visual, 1=ICP (1 requires scan)-->
          <param name="Reg/Strategy" type="string" value="0"/>

          <!-- Minimum visual inliers to accept loop closure -->
          <param name="Vis/MinInliers" type="string" value="10"/>

          <!-- Set to false to avoid saving data when robot is not moving -->
          <param name="Mem/NotLinkedNodesKept" type="string" value="false"/>
          <param name="Grid/FromDepth"         type="string" value="false"/>
        </node>
      </group>

    <!-- visualization with rtabmapviz -->
      <node pkg="rtabmap_ros" type="rtabmapviz" name="rtabmapviz" args="-d $(find rtabmap_ros)/launch/config/rgbd_gui.ini" output="screen">
        <param name="subscribe_depth"             type="bool" value="true"/>
        <param name="subscribe_scan"              type="bool" value="true"/>
        <param name="frame_id"                    type="string" value="robot_footprint"/>
        <param name="queue_size"                  type="int" value="10"/>

        <remap from="rgb/image"       to="$(arg rgb_topic)"/>
        <remap from="depth/image"     to="$(arg depth_topic)"/>
        <remap from="rgb/camera_info" to="$(arg camera_info_topic)"/>
        <remap from="scan"            to="/scan"/>
        <remap from="odom"            to="/odom"/>
      </node>

    </launch>
    ```
    + It is possible to achieve real time visualization using the ```rtabmapviz``` node. ```rtabmapviz``` is great to deploy on a real robot during live mapping to ensure that you are getting the necessary features to complete loop closures. Enabling this will launch the _rtabmapviz GUI_ and provide you with realtime feature detection, loop closures, and other relevant information to the mapping process.
    
4. Clone, install dependencies, build and launch the ```turtlebot_teleop``` package:
    + Clone the package. To know more visit [this link](http://wiki.ros.org/turtlebot_teleop) -
    ```sh
    $ cd ~/catkin_ws/src
    $ git clone https://github.com/turtlebot/turtlebot
    ```

    + Install the package -
    ```sh
    $ cd ~/catkin_ws
    $ source devel/setup.bash
    $ rosdep -i install turtlebot_teleop
    ```
    
    + Build the package -
    ```sh
    $ catkin_make
    $ source devel/setup.bash
    ```
    
    + Launch the node -
    ```sh
    $ roslaunch turtlebot_teleop keyboard_teleop.launch 
    ```
    
5. Map the World!: Everything is set up, now launch all the nodes -
    + Launch the Gazebo world and Rviz configuration -
    ```sh
    $ roslaunch my_robot world.launch
    ```
    
    + Launch the ```teleop``` node -
    ```sh
    $ rosrun teleop_twist_keyboard teleop_twist_keyboard.py
    ```    
    
    + Launch the mapping node -
    ```sh
    $ roslaunch my_robot mapping.launch
    ```
    
    + Navigate the robot in the environment to create the map of it. When the mapping is completed for the entire environment, you can find the map ```.db``` file at the path specified in the ```launch``` file, otherwise, by default it will be located in ```/root/.ros``` folder.
    
    + Best Practices -
        - Start with lower velocity. Our goal is to create a great map with the least amount of passes as possible.
        - You can maximize your loop closures by going over similar paths two or three times. Getting 3 loop closures will be sufficient for mapping the entire environment. This allows for the maximization of feature detection, facilitating faster loop closures.
        - When you are done mapping, be sure to copy or move your database before moving on to map a new environment. Remember, relaunching the mapping node deletes any database in place on launch start up.

6. Database Analysis: The ```rtabmap-databaseViewer``` is a great tool for exploring your database when you are done generating it. It is isolated from ROS and allows for complete analysis of your mapping session. This is how you will check for loop closures, generate 3D maps for viewing, extract images, check feature mapping rich zones, and much more.

    ![rtab-dbviewer](MappedWorld1.png)

    + You can open the mapping database by entering the following command in a new terminal -
    ```sh
    $ rtabmap-databaseViewer ~/.ros/rtabmap.db
    ```
    
    + You need to add certain features for better visualization -
        - Say yes to using the database parameters
        - View -> Constraint View
        - View -> Graph View
        
    + Some pointers on what is observed in the GUI
    
        ![rtab-dbviewer-panel](rtab-dbviewer-panel.png)
    
        - On the left, you have your 2D grid map in all of its updated iterations and the path of your robot.
        - In the middle you have different images from the mapping process. Here you can scrub through images to see all of the features from your detection algorithm. These features are in yellow. The pink indicates where two images have features in common and this information is being used to create neighboring links and loop closures
        - On the right you can see the constraint view. This is where you can identify where and how the neighboring links and loop closures were created.
        - You can see the number of loop closures in the bottom left. The codes stand for the following: Neighbor, Neighbor Merged, Global Loop closure, Local loop closure by space, Local loop closure by time, User loop closure, and Prior link.
	
        
7. RTAB-Map Localization: If you desire to perform localization using the map you created, there are only a few changes you need to make. You can start by duplicating your ```mapping.launch``` file and renaming the duplicated file to ```localization.launch```. This is another method for localization you can keep in mind when working on your next robotics project. The following changes also need to be made to the localization.launch file -

    + Remove the ```args="--delete_db_on_start"``` from your node launcher since you will need your database to localize too.
    + Remove the ```Mem/NotLinkedNodesKept``` parameter
    + Add the ```Mem/IncrementalMemory``` parameter of type ```string``` and set it to ```false``` to finalize the changes needed to put the robot into localization mode.
	

### Results	
	
![rtab-map-rviz](MappedWorld2.png)
	
	
![rtab-map-3D-1](MappedWorld3.png)
	
	
![rtab-map-3D-2](MappedWorld4.png)
	
	    
### Future Work
Try to build different simulation environments and evaluate the performance between them.

### License
MIT License

### Contribution
You may contribute to this project by forking this GitHub repository, creating pull requests or by raising issues.
