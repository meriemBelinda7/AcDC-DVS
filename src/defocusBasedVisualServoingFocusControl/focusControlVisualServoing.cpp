#include"ddvsDefocusBasedVisualServoingFocusControl.h"



int main(int argc, char ** argv)
{
    rclcpp::init(argc, argv);

    auto node = std::make_shared<ddvsDefocusBasedVisualServoingFocusControl>();
   
    node->init();

    rclcpp::spin(node);

    rclcpp::shutdown();

  return 0;
}