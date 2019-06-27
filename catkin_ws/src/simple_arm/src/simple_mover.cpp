#include "ros/ros.h"
#include "std_msgs/Float64.h"

int main(int argc, char *argv[])
{
    // Initialize the arm_mover node
    ros::init(argc, argv, "arm_mover");

    // Create a handle to arm_mover node
    ros::NodeHandle n;

    // Create a publisher that can publish std_msgs/Float64 message on the joint_x_position_controller topic
    ros::Publisher pub1 = n.advertise<std_msgs::Float64>("/simple_arm/joint_1_position_controller/command" ,10);
    ros::Publisher pub2 = n.advertise<std_msgs::Float64>("/simple_arm/joint_2_position_controller/command" ,10);

    // Set loop frequency to 10Hz
    ros::Rate loop_rate(10);

    int start_time, elapsed;

    // Get ROS Start time
    while (not start_time) {
        start_time = ros::Time::now().toSec();
    }

    while (ros::ok()) {
        // Get ROS elapsed time
        elapsed = ros::Time::now().toSec() - start_time;

        // Set the arm joint angles
        std_msgs::Float64 joint1_angle, joint2_angle;

        joint1_angle.data = sin(2 * M_PI * 0.1 * elapsed) * (M_PI / 2);
        joint2_angle.data = sin(2 * M_PI * 0.1 * elapsed) * (M_PI / 2);

        ROS_INFO("%f",joint1_angle.data);
        ROS_INFO("%f",joint1_angle.data);

        // Publish the arm joint angles
        pub1.publish(joint1_angle);
        pub2.publish(joint2_angle);
    
        loop_rate.sleep();
    }

    return 0;
}
