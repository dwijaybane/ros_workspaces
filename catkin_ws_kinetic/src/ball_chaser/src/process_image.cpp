#include <iostream>
#include <vector>
#include <algorithm>
#include <ros/ros.h>
#include <sensor_msgs/Image.h>
#include "ball_chaser/DriveToTarget.h"


// Define a global client that can request services
ros::ServiceClient client;


// This function calls the command_robot service to drive the robot in the specified direction
void drive_robot(float lin_x, float ang_z)
{
    // Request a service and pass the velocities to it to drive the robot
    ball_chaser::DriveToTarget srv;
    srv.request.linear_x = lin_x;
    srv.request.angular_z = ang_z;

    // Call service with passed parameters
    if(!client.call(srv))
        ROS_INFO("Failed to call command_robot service");
}


// This callback function continuously executes and reads the image data
void process_image_callback(const sensor_msgs::Image img)
{

    int white_pixel = 255;
    int left_counter = 0;
    int front_counter = 0;
    int right_counter = 0;

    // TODO: Loop through each pixel in the image and check if there's a bright white one
    for (int i = 0; i < img.height*img.step; i += 3) {
        // locate pixel position
        int position_index = (i % (img.width*3)) / 3;

        if(img.data[i]==white_pixel && img.data[i+1]==white_pixel && img.data[i+2]==white_pixel) {
            if(position_index <= 265) {
                left_counter += 1;
            }
            if(position_index > 265 && position_index <= 533) {
                front_counter += 1;
            }
            if(position_index > 533) {
                right_counter += 1;
            }
        }
    }

    // Then, identify if this pixel falls in the left, mid, or right side of the image
    std::vector<int> position_vote{left_counter, front_counter, right_counter};
    int where_to_move = *std::max_element(position_vote.begin(), position_vote.end());

    // Depending on the white ball position, call the drive_bot function and pass velocities to it
    // Request a stop when there's no white ball seen by the camera
    if(where_to_move == 0) {
        drive_robot(0.0, 0.0);
    }
    else if(where_to_move == left_counter) {
        drive_robot(0.0, 0.2);
    }
    else if(where_to_move == front_counter) {
        drive_robot(0.5, 0.0);
    }
    else if(where_to_move == right_counter) {
        drive_robot(0.0, -0.2);
    }
}


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
