#include "ros/ros.h"
#include "geometry_msgs/Twist.h"
#include "ball_chaser/DriveToTarget.h"
//TODO: Include the ball_chaser "DriveToTarget" header file

// ROS::Publisher motor commands;
ros::Publisher motor_command_publisher;

bool handle_drive_request(ball_chaser::DriveToTarget::Request& request1, ball_chaser::DriveToTarget::Response& response1)
{
    geometry_msgs::Twist msg_for_robot{};

    msg_for_robot.linear.x = request1.linear_x;
    msg_for_robot.angular.z = request1.angular_z;

    motor_command_publisher.publish(msg_for_robot);

    // Wait 3 seconds for arm to settle
    ros::Duration(3).sleep();

    // Return a response message
    response1.msg_feedback = "message sent to robot";
    ROS_INFO_STREAM(response1.msg_feedback);

    return true;
}

int main(int argc, char** argv)
{
    // Initialize a ROS node
    ros::init(argc, argv, "drive_bot");

    // Create a ROS NodeHandle object
    ros::NodeHandle n;

    // Inform ROS master that we will be publishing a message of type geometry_msgs::Twist on the robot actuation topic with a publishing queue size of 10
    motor_command_publisher = n.advertise<geometry_msgs::Twist>("/cmd_vel", 10);

    // TODO: Define a drive /ball_chaser/command_robot service with a handle_drive_request callback function
    ros::ServiceServer service = n.advertiseService("ball_chaser/command_robot", handle_drive_request);

    // TODO: Handle ROS communication events
    ros::spin();

    return 0;
}