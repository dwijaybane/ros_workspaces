#include <ros/ros.h>
#include "geometry_msgs/Twist.h"
#include "ball_chaser/DriveToTarget.h"


ros::Publisher motor_command_publisher;


bool command_robot_service_callback(ball_chaser::DriveToTarget::Request& req, ball_chaser::DriveToTarget::Response& res)
{
    ROS_INFO("Drive Bot service called lin.x %1.2f  ang.z", req.linear.x, req.angular.z);
}


int main(int argc, char *argv[])
{
    ros::init(argc, argv, "drive_bot");
    
    ros::NodeHandle n;

    motor_command_publisher = n.advertise<geometry_msgs::Twist>("/cmd_vel", 10);
    
    geometry_msgs::Twist motor_command;

    motor_command.linear.x = 0.5;
    motor_command.angular.z = 0.0;

    motor_command_publisher.publish(motor_command);
    
    ros::ServiceServer motor_service = n.advertiseService("/ball_chaser/command_robot", command_robot_service_callback);

    ros::spin();

    return 0;
}
