#include <ros/ros.h>
#include "geometry_msgs/Twist.h"
#include "ball_chaser/DriveToTarget.h"

// Global Drive Publisher Variable
ros::Publisher motor_command_publisher;

// This callback function executes whenever a command_robot service is requested
bool handle_drive_request(ball_chaser::DriveToTarget::Request& req, ball_chaser::DriveToTarget::Response& res)
{
    ROS_INFO("Drive Bot request received lin.x %1.2f  ang.z %1.2f", req.linear_x, req.angular_z);

    // Use msg datatype for publishing
    geometry_msgs::Twist motor_command;

    motor_command.linear.x = req.linear_x;
    motor_command.linear.y = 0.0;
    motor_command.linear.z = 0.0;
    motor_command.angular.x = 0.0;
    motor_command.angular.y = 0.0;
    motor_command.angular.z = req.angular_z;

    // Publish requested velocity and angle
    motor_command_publisher.publish(motor_command);

    ros::Duration(1).sleep();

    // Return a response message
    res.msg_feedback = "Linear Velocity set - lin.x: " + std::to_string(req.linear_x) + "  Angular Angle Set - ang.z: " + std::to_string(req.angular_z);
    ROS_INFO_STREAM(res.msg_feedback);

    return true;
}


int main(int argc, char *argv[])
{
    // Initialize drive_bot node with NodeHandle
    ros::init(argc, argv, "drive_bot");
    ros::NodeHandle n;

    // Create Publisher for geometry_msgs to publish on /cmd_vel topic
    motor_command_publisher = n.advertise<geometry_msgs::Twist>("/cmd_vel", 10);

    // Define a drive /ball_chaser/command_robot service with a handle_drive_request callback function
    ros::ServiceServer motor_service = n.advertiseService("/ball_chaser/command_robot", handle_drive_request);

    ROS_INFO("Ready to send commands to drive bot");

    // Handle ROS Communication events
    ros::spin();

    return 0;
}
