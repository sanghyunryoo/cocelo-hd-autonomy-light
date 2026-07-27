#include <memory>

#include <rclcpp/rclcpp.hpp>

#include "autonomy_light/localization_node.hpp"

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<autonomy_light::LocalizationNode>());
  rclcpp::shutdown();
  return 0;
}
