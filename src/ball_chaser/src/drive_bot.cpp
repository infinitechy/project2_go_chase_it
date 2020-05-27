#include "ros/ros.h"
#include "geometry_msgs/Twist.h"
#include "ball_chaser/DriveToTarget.h"
//TODO: Include the ball_chaser "DriveToTarget" header file

class DriveBot
{
    public:
      DriveBot()
        {
            motor_command_publisher_ =  n_.advertise<geometry_msgs::Twist>("/cmd_vel", 10);
            service_ = n_.advertiseService("ball_chaser/command_robot", &DriveBot::handle_drive_request, this);
        }

        bool handle_drive_request(ball_chaser::DriveToTarget::Request& request1, ball_chaser::DriveToTarget::Response& response1)
        {
            geometry_msgs::Twist msg_for_robot{};

            msg_for_robot.linear.x = request1.linear_x;
            msg_for_robot.angular.z = request1.angular_z;

            motor_command_publisher_.publish(msg_for_robot);

            // Return a response message
            response1.msg_feedback = "message sent to robot";
            ROS_INFO_STREAM(response1.msg_feedback);

            return true;
        }

    private:
        ros::Publisher motor_command_publisher_{};
        ros::ServiceServer service_{};
        ros::NodeHandle n_{};
};

int main(int argc, char** argv)
{
    // Initialize a ROS node
    ros::init(argc, argv, "drive_bot");

    DriveBot dr{};

    ros::spin();

    return 0;
}
