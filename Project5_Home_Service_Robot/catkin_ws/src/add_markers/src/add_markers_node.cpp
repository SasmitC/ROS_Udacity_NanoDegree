#include <ros/ros.h>
#include <nav_msgs/Odometry.h>
#include <visualization_msgs/Marker.h>
#include <vector>

ros::Publisher publisher_marker;
visualization_msgs::Marker visual_marker;

bool isInPickUpZone = false;
bool isInDropOffZone = false;
bool pickedUp = false;
bool droppedOff = false;

std::vector<float> pickup_location = {4.0, 4.0, 0.0};
std::vector<float> dropoff_location = {-4.0, 4.0, 0.0};


void odom_callback(const nav_msgs::Odometry::ConstPtr& msg)
{
    //Get the current position and orientation of the robot from odom topic
    float x = msg->pose.pose.position.x;
    float y = msg->pose.pose.position.y;
    //Define a threshold distance
    float pickUpDist = 0.0;
    float dropOffDist = 0.0;
    //Define an allowed error tolerance
    const float delta_err = 0.5;

    //Robot is considered to be inside Pick-Up Zone if it lies within a sphere of radius delta_err 

    if(!isInPickUpZone && !isInDropOffZone)
    {
        pickUpDist = sqrt(pow((pickup_location[0] - x), 2) + pow((pickup_location[1] - y), 2));
        ROS_INFO("Distance away from the Pick-Up Area : %f", pickUpDist);

        if(pickUpDist <= delta_err)
        {
            //After reaching the Pick-Up Zone, create an illusion that the item has been picked up by the robot and make it disappear
            ROS_INFO("Robot has arrived in the Pick-Up Zone and has picked up the item!! Deleting the Visual Marker at this location...");
            isInPickUpZone = true;
            pickedUp = true;
        }
    }

    else if(isInPickUpZone && !isInDropOffZone)
    {
        dropOffDist = sqrt(pow((dropoff_location[0] - x), 2) + pow((dropoff_location[1] - y), 2));
        ROS_INFO("Distance away from the Drop-Off : %f", dropOffDist);

        if(dropOffDist <= delta_err)
        {
            ROS_INFO("Robot has arrived in the Drop-Off Zone and has delivered the item!!! Task COMPLETED!!");
            isInDropOffZone = true;
            isInPickUpZone = false;
            droppedOff = true;
        }
    }

}

int main(int argc, char** argv)
{
    ros::init(argc, argv, "add_markers");
    ros::NodeHandle n;

    //Create a publisher for Visual Marker
    publisher_marker = n.advertise<visualization_msgs::Marker>("visualization_marker", 1);

    //Create a subscriber to the odom topic
    ros::Subscriber subscriber_odom = n.subscribe("/odom", 1000, odom_callback);

    //Set up Visual Marker features
    visual_marker.header.frame_id = "map";
    visual_marker.header.stamp = ros::Time::now();
    visual_marker.ns = "add_markers";
    visual_marker.id = 0;
    visual_marker.type = visualization_msgs::Marker::SPHERE;

    //Set the orientation of the Visual Marker
    visual_marker.pose.orientation.x = 0.0;
    visual_marker.pose.orientation.y = 0.0;
    visual_marker.pose.orientation.z = 0.0;
    visual_marker.pose.orientation.w = 1.0;

    //Set the scale
    visual_marker.scale.x = 0.5;
    visual_marker.scale.y = 0.5;
    visual_marker.scale.z = 0.5;

    //Set the colour
    visual_marker.color.r = 1.0f;
    visual_marker.color.g = 0.0f;
    visual_marker.color.b = 0.0f;
    visual_marker.color.a = 1.0f;

    //Create a Pick-Up Location
    visual_marker.pose.position.x = pickup_location[0];
    visual_marker.pose.position.y = pickup_location[1];
    visual_marker.pose.position.z = pickup_location[2];

    visual_marker.lifetime = ros::Duration();

    while(publisher_marker.getNumSubscribers() < 1)
    {
        if(!ros::ok())
        {
            return 0;
        }
        ROS_WARN_ONCE("First create a subscriber to the Visual Marker! Add Marker in rviz config window");
        sleep(1);
    }

    ros::Rate r(1);

    //Wait for odometry messages
    ROS_INFO_STREAM("Subscribed to Visual Marker. Listening to odom messages...");
    ros::Duration(2.0).sleep();

    //Publish the Visual Marker at the Pick-Up Location
    visual_marker.action = visualization_msgs::Marker::ADD;

    publisher_marker.publish(visual_marker);
    ROS_INFO("Published a Visual Marker at Pick-Up Location!!");

    while (ros::ok()) 
    {
        if(isInPickUpZone)
        {           
            
            if(!pickedUp) ROS_INFO("Allowing the robot to reach the Pick-Up Zone...");
            visual_marker.action = visualization_msgs::Marker::DELETE;

            // publisher_marker.publish(visual_marker);
            //Wait for 3s
            sleep(3);            
        }

        else if(isInDropOffZone)
        {
            //Create a new Visual Marker at the Drop-Off Location
            visual_marker.pose.position.x = dropoff_location[0];
            visual_marker.pose.position.y = dropoff_location[1];
            visual_marker.pose.position.z = dropoff_location[2];

            if(!droppedOff) ROS_INFO("Allowing the robot to reach the Drop-Off Area...");
            visual_marker.action = visualization_msgs::Marker::ADD;

            // publisher_marker.publish(visual_marker);
            //Wait for 3s
            sleep(3);
        }
        publisher_marker.publish(visual_marker);
        ros::spinOnce();
    }
    r.sleep();
    return 0;

}