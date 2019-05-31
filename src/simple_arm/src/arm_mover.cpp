#include "ros/ros.h"
#include <std_msgs/Float64.h>
#include "simple_arm/GoToPosition.h"


// Global joint publisher variable
ros::Publisher joint_1, joint_2;


bool handle_safe_move_request(simple_arm::GoToPosition::Request& req,
        simple_arm::GoToPosition::Response& res)
{
    ROS_INFO("GoToPositionRequest Received - j1:%1.2f, j2:%1.2f", (float)req.joint_1, (float)req.joint_2);
    


}


int main(int argc, char *argv[])
{
    // Initialize ROS node arm_mover with handle 
    ros::init(argc, argv, "arm_mover");
    ros::NodeHandle n;

    // Define two publishers to publish std_msgs::Float64 messages on joints respective topics 
    joint_1 = n.advertise<std_msgs::Float64>("/simple_arm/joint_1_position_controller/command",10);
    joint_2 = n.advertise<std_msgs::Float64>("/simple_arm/joint_2_position_controller/command",10);

    // Define a safe_move service with a handle_safe_move_request callback function
    ros::ServiceServer service = n.advertiseService("/arm_mover/safe_move", handle_safe_move_request);
    ROS_INFO("Ready to send joint commands");

    // Handle ROS communication events
    ros::spin();


    return 0;
}
