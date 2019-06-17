#include <ros/ros.h>
#include "geometry_msgs/Twist.h"


int main(int argc, char *argv[])
{
    // Initialize the drive_bot_simple node with NodeHandle
    ros::init(argc, argv, "drive_bot");
    ros::NodeHandle n;

    // Create a publisher that can publish geometry_msgs/Twist msg on the /cmd_vel topic
    ros::Publisher motor_command_publisher = n.advertise<geometry_msgs::Twist>("/cmd_vel", 10);

    // Set loop frequency to 10Hz
    ros::Rate loop_rate(10);

    // Get ROS Clock Start Time
    int start_time, elapsed;

    while(not start_time){
        start_time = ros::Time::now().toSec();
    }

    // Get ROS Elapsed Time
    elapsed = ros::Time::now().toSec() - start_time;

    // Init msg type
    geometry_msgs::Twist msg_vel;

    msg_vel.linear.x = 0.1;
    msg_vel.linear.y = 0.0;
    msg_vel.linear.z = 0.0;
    msg_vel.angular.x = 0.0;
    msg_vel.angular.y = 0.0;
    msg_vel.angular.z = 0.2;

    ROS_INFO("Publishing lin.x: %1.2f ang.z: %1.2f",msg_vel.linear.x, msg_vel.angular.z);

    // Publish msg using publisher to /cmd_vel
    motor_command_publisher.publish(msg_vel);

    return 0;
}
