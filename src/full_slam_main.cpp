#include "autonomy_light/full_slam_node.hpp"

#include <rclcpp/rclcpp.hpp>

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<autonomy_light::FullSlamNode>());
  rclcpp::shutdown();
  return 0;
}
