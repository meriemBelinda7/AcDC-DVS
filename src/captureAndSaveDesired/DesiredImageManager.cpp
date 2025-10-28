#include "DesiredImageManager.h"

DesiredImageManager::DesiredImageManager():Node("CaptureAndSaveDesiredImage")
{
    this->declare_parameter<std::string>("Image_topic", "currentImage");
    this->declare_parameter<std::string>("Log_folder", "/home/belinda-naamani/.ros/ros_dvs_bridge/Id.png");
    this->declare_parameter<std::string>("joint_topic", "cmd/joint");

    this->declare_parameter<double>("J1", double(52.30));
    this->declare_parameter<double>("J2", double(-72.92));
    this->declare_parameter<double>("J3", double(79.45));
    this->declare_parameter<double>("J4", double(-96.57));
    this->declare_parameter<double>("J5", double(-90.43));
    this->declare_parameter<double>("J6", double(54.13));
    this->declare_parameter<bool>("INDICATOR", true);

    INDICATOR = this->get_parameter("INDICATOR").as_bool();
    
    
    image_topic=this->get_parameter("Image_topic").as_string();
    filename_write_image= this->get_parameter("Log_folder").as_string();
    joint_topic_ = this->get_parameter("joint_topic").as_string();
    j_init.data.resize(6);
    
    j_init.data[0]=(vpMath::rad(this->get_parameter("J1").as_double()));
    j_init.data[1]=(vpMath::rad(this->get_parameter("J2").as_double()));
    j_init.data[2]=(vpMath::rad(this->get_parameter("J3").as_double())) ;
    j_init.data[3]=(vpMath::rad(this->get_parameter("J4").as_double()));
    j_init.data[4]=(vpMath::rad(this->get_parameter("J5").as_double()))  ;
    j_init.data[5]=(vpMath::rad(this->get_parameter("J6").as_double()));

}
DesiredImageManager::~DesiredImageManager()
{
    
}
void DesiredImageManager::init()
{

    image_transport::ImageTransport it(this->shared_from_this());
    sub_ = it.subscribe(image_topic, 1, std::bind(&DesiredImageManager::imageCallback, this,  std::placeholders::_1));
    joint_pub_ = this->create_publisher<std_msgs::msg::Float64MultiArray>(joint_topic_,1);

    joint_pub_->publish(j_init);
    vpTime::wait(5000);
    
    
}

void DesiredImageManager::imageCallback(const sensor_msgs::msg::Image::ConstSharedPtr &msg)
{


    try
    {
        //m_desired_image = visp_bridge::toVispImage(*msg);
        m_desired_image = visp_bridge::toVispImage(*msg);

        ss_desired_image << filename_write_image<<"/Id.png";
        std::string file_path = ss_desired_image.str();
        filename_write_image = vpIoTools::path(file_path.c_str());
        vpImageIo::write(m_desired_image, filename_write_image);
    }
    catch (const std::exception &e)
    {
        // Error handling in case conversion fails
        RCLCPP_ERROR(this->get_logger(), "Failed to process image: %s", e.what());
    }

}
