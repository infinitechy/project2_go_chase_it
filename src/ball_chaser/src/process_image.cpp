#include "ros/ros.h"
#include "ball_chaser/DriveToTarget.h"
#include <sensor_msgs/Image.h>

// Define a global client that can request services
ros::ServiceClient client;

// This function calls the command_robot service to drive the robot in the specified direction
void drive_robot(float lin_x, float ang_z)
{
    // TODO: Request a service and pass the velocities to it to drive the robot

    ball_chaser::DriveToTarget req;
    req.request.angular_z = ang_z;
    req.request.linear_x = lin_x;
    client.call(req);

    ROS_INFO("message feedback: %s", req.response.msg_feedback);
}

// This callback function continuously executes and reads the image data
void process_image_callback(const sensor_msgs::Image& img)
{
    int white_pixel = 255;
    int row{-1};
    int col{-1};

    // left, mid and right regions in the image
    std::pair<int, int> left = std::make_pair(0, img.width/4);
    std::pair<int, int> center = std::make_pair(img.width/4+1, img.width/2 + img.width/4);
    std::pair<int, int> right = std::make_pair(img.width/2 + img.width/4, img.width);

    for(int i = 0; i < img.height; ++i)
    {
        for(int j = 0; j < img.width; ++j)
        {
            //in this logic, comparision is done for one value, because for the white
            //ball R=255, G=255, B= 255
            auto extracted_data = img.data[i*img.step + j*3];

            if(extracted_data == white_pixel)
            {
                row = i;
                col = j;
                break;
            }
        }
    }

    if((row != -1) && (col != -1))
    {
        if(col < left.second && col >= left.first)
        {
            drive_robot(0.5, 0.5);
            ROS_INFO("left movement --- coordinates: %d   %d", row, col);
        }
        else if(col < center.second && col >= center.first)
        {
            drive_robot(0.5, 0);
            ROS_INFO("center movement --- coordinates: %d   %d", row, col);
        }
        else
        {
            drive_robot(0.5, -0.5);
            ROS_INFO("right movement --- coordinates: %d   %d", row, col);
        }        
    }
    else
    {
        drive_robot(0.0, 0.0);
        ROS_INFO("no movement --- coordinates: %d   %d", row, col);
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