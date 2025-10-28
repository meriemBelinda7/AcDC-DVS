#ifndef __ddvsDefocusBasedVisualServoingFocusControl_H__
#define __ddvsDefocusBasedVisualServoingFocusControl_H__

#include "rclcpp/rclcpp.hpp"
#include "image_transport/image_transport.hpp"
#include "sensor_msgs/msg/image.hpp"
#include "visp_bridge/image.h"
#include <visp/vpIoTools.h>
#include <visp/vpImageIo.h>
#include <tf2_msgs/msg/tf_message.hpp>
#include <visp3/core/vpCameraParameters.h>
#include <visp3/visual_features/vpFeatureLuminance.h>
#include <visp3/vs/vpServo.h>
#include "include/CCameraThinLensParameters.h"
#include "include/CFeatureDefocusedLuminanceFC.h"
#include <geometry_msgs/msg/twist.hpp>
#include <std_msgs/msg/float64.hpp>
#include <visp3/core/vpImageTools.h>
#include <std_msgs/msg/float64_multi_array.hpp>


using namespace std;

class ddvsDefocusBasedVisualServoingFocusControl:public rclcpp::Node
{
    public:
        ddvsDefocusBasedVisualServoingFocusControl();
        ~ddvsDefocusBasedVisualServoingFocusControl();
        void init();
        void go_to_initial_position(vpColVector& p_init);
        void imageCallback(const sensor_msgs::msg::Image::ConstSharedPtr &msg);
        void initVisualServoingTask();
        void toolPoseCallback(const tf2_msgs::msg::TFMessage& tf);
        vpHomogeneousMatrix toVispHomogeneousMatrix(const tf2_msgs::msg::TFMessage& trans);
        int errorToImage(vpColVector &e, vpImage<unsigned char> &diff_image);
        //void binning(const vpImage<unsigned char>& src, vpImage<unsigned char>& dst, int binningFactor);
        void stopRobot();
        vpColVector getError(CFeatureDefocusedLuminanceFC &sI, CFeatureDefocusedLuminanceFC &sId);

    private:

        //Later to be converted to launch parameters
        double mu;
        double lambda_;
        double px_;
        double py_;
        double u0_,v0_;
        double FNumber_;
        double Zf_;
        double f_;
        double ku_;
        bool tx_,ty_,tz_,rx_,ry_,rz_;
        int binning_val_;
        string cameraTopic_;
        string robotTopic_;
        string cameraPoseTopic_;
        string diffTopic_;
        string errTopic_;
        string logs_path_;
        string data_path_;
        string focusTopic_;
        string vel_topic_;
        string rel_pos_topic_;
        std::string twist_topic_;
        double sceneDepth_;
        vpMatrix L;
        rclcpp::TimerBase::SharedPtr timer_;
        void timerCallback();


        unsigned int height_, width_;
        image_transport::Subscriber sub_;
        image_transport::Publisher diff_pub_;
        image_transport::Publisher id_pub_;
        rclcpp::Subscription<tf2_msgs::msg::TFMessage>::SharedPtr camPose_sub_;
        //rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr velocity_pub_;
        rclcpp::Publisher<std_msgs::msg::Float64>::SharedPtr residual_pub_;
        rclcpp::Publisher<std_msgs::msg::Float64>::SharedPtr focus_pub_;
        rclcpp::Publisher<std_msgs::msg::Float64MultiArray>::SharedPtr vel_pub_;
        rclcpp::Publisher<std_msgs::msg::Float64MultiArray>::SharedPtr rel_pos_pub_;
        rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr twist_pub_;
        vpVelocityTwistMatrix bVc_;

        geometry_msgs::msg::Twist twist_msg_;
        sensor_msgs::msg::Image diff_;
        sensor_msgs::msg::Image desired_;
        std_msgs::msg::Float64 residuals_msg_;
        std_msgs::msg::Float64 focus_msg_;
    
        vpImage<unsigned char> desired_image_;
        vpImage<unsigned char> current_image_;
        vpImage<unsigned char> current_image_temp_;
        vpImage<unsigned char> difference_image_;
        double clamped_result, c_result1;
        vpMatrix dof_;
        std_msgs::msg::Float64MultiArray p_init_;
        std_msgs::msg::Float64MultiArray v_;
        CCameraThinLensParameters cam_;
        CFeatureDefocusedLuminanceFC sId;
        CFeatureDefocusedLuminanceFC sI;

        vpServo Servo;
        
        double mu_;

        double norm_error_;

        int iter_;
        bool vsInitialized_;
        bool controlInBaseFrame_;
        bool pub_diffImage_;
        int nDof_;
        int dim_s_;
        vpMatrix L_;
        vpColVector v_temp_;
        std::ofstream log_file_;
		std::ofstream residuals_;
		std::ofstream times_;
        double dZf_;
        int Zf_step_;
        double new_zf_;
        bool flag = false;

        
};
#endif