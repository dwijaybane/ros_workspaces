#include <ros/ros.h>
#include "simple_arm/GoToPosition"
#include "sensor_msgs/JointState.h"
#include "sensor_msgs/Image.h"


ros::ServiceClient client;


// This callback function continuously executes and reads the arm joint angles position
void joint_states_callback(const sensor_msgs::JointState js)
{

}


// This callback function continuously executes and reads the image data
void look_away_callback(const sensor_msgs::Image img)
{

}


int main(int argc, char *argv[])
{
    // Initialize the look_away node and create a handle to it
    ros::init(argc, argv, "look_away");
    ros::NodeHandle n;

    // Define a client service capable of requesting services from safe_move
    client = n.serviceClient<simple_arm::GoToPosition>("/arm_mover/safe_move");

    // Subscribe to /simple_arm/joint_states topic to read the arm joints position inside the joint_states_callback function
    ros::Subscriber sub1 = n.subscribe("/simple_arm/joint_states", 10, joint_states_callback);

    // Subscribe to rgb_camera/image_raw topic to read the image data inside the look_away_callback function
    ros::Subscriber sub2 = n.subscribe("/rgb_camera/image_raw", 10, look_away_callback);

    // Handle ROS communication events
    ros::spin();

    return 0;
}
