#include "DesiredImageManager.h"




int main(int argc, char ** argv)
{
    rclcpp::init(argc, argv);

    auto node = std::make_shared<DesiredImageManager>();
    node->init();
    rclcpp::spin(node);

    rclcpp::shutdown();

  return 0;
}
