#include "lgdxrobot2_agent/Agent.hpp"

int main(int argc, char **argv)
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<Agent>();
  node->Initalise();
  rclcpp::spin(node);
  node->Shutdown();
  return 0;
}