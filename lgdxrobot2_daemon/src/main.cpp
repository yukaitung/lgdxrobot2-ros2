#include "DaemonNode.hpp"

int main(int argc, char **argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<DaemonNode>());
  rclcpp::shutdown();
  return 0;
}