#include "ros/ros.h"
#include "ball_chaser/DriveToTarget.h"
#include <sensor_msgs/Image.h>

class ProcessImage
{
    public: 
        ProcessImage()
        {
            sub1_ = n_.subscribe("/camera/rgb/image_raw", 10, &ProcessImage::process_image_callback, this);
            client_ = n_.serviceClient<ball_chaser::DriveToTarget>("/ball_chaser/command_robot");
        }

        // This callback function continuously executes and reads the image data
        void process_image_callback(const sensor_msgs::Image& img)
        {
            auto res = process_image(img);

            if(res[0] == 0 && res[1] == 0 && res[2] == 0)
            {
                drive_robot(0, 0);
                return;
            }

            auto array_pointer = std::max_element(res.begin(), res.end());
            int index = std::distance(res.begin(), array_pointer);
            if(index == 0)
            {
                drive_robot(0, 0.5);
            }
            else if(index == 1)
            {
                drive_robot(0.5, 0);
            }
            else
            {
                drive_robot(0.0, -0.5);
            }
        }

    private:
        std::vector<int> process_image(const sensor_msgs::Image& img)
        {
            int white_pixel = 255;
            // left, mid and right regions in the image
            std::pair<int, int> left = std::make_pair(0, img.width/4);
            std::pair<int, int> center = std::make_pair(img.width/4+1, img.width/2 + img.width/4);
            std::pair<int, int> right = std::make_pair(img.width/2 + img.width/4, img.width);
            std::vector<int> segment_count(3,0);

            for(int i = 0; i < img.height; ++i)
            {
                for(int j = 0; j < img.width; ++j)
                {
                    //in this logic, comparision is done for one value, because for the white
                    //ball R=255, G=255, B= 255
                    auto extracted_data_r = img.data[i*img.step + j*3];
                    auto extracted_data_g = img.data[i*img.step + j*3 + 1];
                    auto extracted_data_b = img.data[i*img.step + j*3 + 2];

                    if((extracted_data_r == white_pixel) && (extracted_data_g == white_pixel) &&
                        (extracted_data_b == white_pixel))
                    {
                        if(j < left.second && j >= left.first)
                        {
                            segment_count[0]++;
                        }
                        else if(j < center.second && j >= center.first)
                        {
                            segment_count[1]++;
                        }
                        else
                        {
                            segment_count[2]++;
                        }        
                    }
                }
            }

            return segment_count;
        }

        // This function calls the command_robot service to drive the robot in the specified direction
        void drive_robot(float lin_x, float ang_z)
        {
            ball_chaser::DriveToTarget req;
            req.request.angular_z = ang_z;
            req.request.linear_x = lin_x;
            client_.call(req);

            ROS_INFO("message feedback: %s", req.response.msg_feedback);
        }

        ros::ServiceClient client_;
        ros::Subscriber sub1_{};
        ros::NodeHandle n_;
};

int main(int argc, char** argv)
{
    // Initialize the process_image node and create a handle to it
    ros::init(argc, argv, "process_image");
    ProcessImage pr{};

    // Handle ROS communication events
    ros::spin();

    return 0;
}

