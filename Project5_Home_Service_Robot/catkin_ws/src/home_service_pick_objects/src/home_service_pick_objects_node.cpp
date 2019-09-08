#include <cmath>
#include <ros/ros.h>
#include <move_base_msgs/MoveBaseAction.h>
#include <actionlib/client/simple_action_client.h>

// Define a client for to send goal requests to the move_base server through a SimpleActionClient
typedef actionlib::SimpleActionClient<move_base_msgs::MoveBaseAction> MoveBaseClient;

// Create a MoveBaseGoal message which accepts the user input(main())
move_base_msgs::MoveBaseGoal setGoal(double pos_x, double pos_y, double rot_w)
{
   move_base_msgs::MoveBaseGoal goal;

  // set up the frame parameters
  goal.target_pose.header.frame_id = "map";
  goal.target_pose.header.stamp = ros::Time::now();

  // Define a position and orientation for the robot to reach
  goal.target_pose.pose.position.x = pos_x;
  goal.target_pose.pose.position.y = pos_y; 
  goal.target_pose.pose.orientation.w = rot_w;
  ROS_INFO("Goal set!!");

  return goal;
}

void navigateToGoal(MoveBaseClient& ac, move_base_msgs::MoveBaseGoal goal, std::string goalInfo, std::string successMsg, std::string failureMsg)
{
  // Send the goal position and orientation for the robot to reach
  ROS_INFO("%s",goalInfo.c_str());
  ac.sendGoal(goal);

  // Wait an infinite time for the results
  ac.waitForResult();

  // Check if the robot reached its goal

  if(ac.getState() == actionlib::SimpleClientGoalState::SUCCEEDED)
    ROS_INFO("%s",successMsg.c_str());
  else
    ROS_INFO("%s",failureMsg.c_str());

  //return goalReached;
}


int main(int argc, char** argv){
  // Initialize the pick_objects node
  ros::init(argc, argv, "home_service_pick_objects");

  //tell the action client that we want to spin a thread by default
  MoveBaseClient ac("move_base", true);

  // Wait 5 sec for move_base action server to come up
  while(!ac.waitForServer(ros::Duration(5.0))){
    ROS_INFO("Waiting for the move_base action server to come up");
  }

  move_base_msgs::MoveBaseGoal pickupGoal = setGoal(-10.0, 3.0, 1.0);
  navigateToGoal(ac, pickupGoal, "Sending pickup goal...", "SUCCESS!! The turtlebot has reached the pickup zone", "FAILURE!! The turtlebot failed to reach the pickup zone for some reason");

  ros::Duration(5.0).sleep();
  move_base_msgs::MoveBaseGoal dropoffGoal = setGoal(10.0, -4.0, 1.0);
  navigateToGoal(ac, dropoffGoal, "Sending dropoff goal...", "SUCCESS!! The turtlebot has reached the dropoff zone", "FAILURE!! The turtlebot failed to reach the dropoff zone for some reason");

  ros::Duration(5.0).sleep();

  return 0;
}
