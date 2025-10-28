#ifndef __DesiredImageManager_H__
#define __DesiredImageManager_H__

#include "rclcpp/rclcpp.hpp"
#include "image_transport/image_transport.hpp"
#include "sensor_msgs/msg/image.hpp"
#include "visp_bridge/image.h"
#include <visp/vpIoTools.h>
#include <visp/vpImageIo.h>
#include <visp3/core/vpImageTools.h>
#include <std_msgs/msg/float64_multi_array.hpp>

using namespace std;

class DesiredImageManager:public rclcpp::Node
{
    public:
        DesiredImageManager();
        ~DesiredImageManager();
        void init();
        
        void imageCallback(const sensor_msgs::msg::Image::ConstSharedPtr &msg);
        std::string image_topic;
        std::string filename_write_image;
        
    private:
        bool INDICATOR;
        image_transport::Subscriber sub_;
        std::string joint_topic_;
        std_msgs::msg::Float64MultiArray j_init;
        rclcpp::Publisher<std_msgs::msg::Float64MultiArray>::SharedPtr joint_pub_;
        vpImage<unsigned char> desired_image_temp;
        vpImage<unsigned char> m_desired_image;
        std::stringstream ss_desired_image;
        std::ofstream positions;
 
};
#endif