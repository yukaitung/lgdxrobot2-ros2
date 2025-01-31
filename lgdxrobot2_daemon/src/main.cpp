#include "lgdxrobot2_daemon/DaemonNode.hpp"

int main(int argc, char **argv)
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<DaemonNode>();
  rclcpp::spin(node);
  node->shutdown();
  rclcpp::shutdown();
  return 0;
}