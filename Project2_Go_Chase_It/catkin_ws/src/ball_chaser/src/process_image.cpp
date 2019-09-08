#include "ros/ros.h"
#include "ball_chaser/DriveToTarget.h"
#include <sensor_msgs/Image.h>

// Define a global client that can request services
ros::ServiceClient client;

// This function calls the command_robot service to drive the robot in the specified direction
void drive_robot(float lin_x, float ang_z)
{
  ROS_INFO_STREAM("Moving the robot towards the ball if the ball lies in the FOV...");
  ball_chaser::DriveToTarget srv;
  srv.request.linear_x = lin_x;
  srv.request.angular_z = ang_z;
  // TODO: Request a service and pass the velocities to it to drive the robot
  if (!client.call(srv))
    ROS_ERROR("Failed to call service drive_robot");
}

// This callback function continuously executes and reads the image data
void process_image_callback(const sensor_msgs::Image img)
{
	int white_pixel = 255;
	int h = img.height, s = img.step;  //image height, that is, number of rows
	int w = img.width;	// image width, that is, number of columns;
	int img_size = h * s;
	int pos;
	bool left = false, center = false, right = false, stop = true;
	float lin_x, ang_z;
  
	for (int i = 0; i < h; i++)
  {
    for(int j = 0; j < s; j +=3)
    {
      if (img.data[i*s + j] == white_pixel && img.data[i*s + j+1] == white_pixel && img.data[i*s + j+2] == white_pixel)
      {
        ROS_INFO_STREAM("Ball detected! Moving towrads the ball...");
			  if(j <= s/3)
        {
          left = true;
          stop = false;
          ROS_INFO_STREAM("Ball detected towards left...");
        }
        else if(j <= 2*s/3)
        {
          center = true;
          stop = false;
          ROS_INFO_STREAM("Ball detected in the middle...");
        }
        else if(j <= s)
        {
          right = true;
          stop = false;
          ROS_INFO_STREAM("Ball detected towards right...");        
        }
        else
        {
          stop = true;
          ROS_INFO_STREAM("Ball not in the FOV...");  
        }
      }
		}
  }
  	
  if (left | center | right)
  {
    if(left)
    {
      lin_x = 0.2;
      ang_z = 0.4;
      drive_robot(lin_x,ang_z);
    }
    else if(right)
    {
      lin_x = 0.2;
      ang_z = -0.4;
      drive_robot(lin_x,ang_z);
    }
    else if(center)
    {
      lin_x = 0.4;
      ang_z = 0.0;
      drive_robot(lin_x,ang_z);
    }  
  }
  else if(stop)
  {
    lin_x = 0.0;
    ang_z = 0.0;
    drive_robot(lin_x,ang_z);
  }
}      
    // TODO: Loop through each pixel in the image and check if there's a bright white one
    // Then, identify if this pixel falls in the left, mid, or right side of the image
    // Depending on the white ball position, call the drive_bot function and pass velocities to it
    // Request a stop when there's no white ball seen by the camera



int main(int argc, char** argv)
{
  // Initialize the process_image node and create a handle to it
  ros::init(argc, argv, "process_image");
  ros::NodeHandle n;

  // Define a client service capable of requesting services from command_robot
  client = n.serviceClient<ball_chaser::DriveToTarget>("/ball_chaser/command_robot");

  // Subscribe to /camera/rgb/image_raw topic to read the image data inside the process_image_callback function
  ros::Subscriber sub1 = n.subscribe("/camera/rgb/image_raw", 10, process_image_callback);

  // Handle ROS communication events
  ros::spin();

  return 0;
}