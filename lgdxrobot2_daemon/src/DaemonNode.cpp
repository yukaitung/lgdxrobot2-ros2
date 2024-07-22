#include "DaemonNode.hpp"

using namespace std::chrono_literals;
#include <iostream>

DaemonNode::DaemonNode() : Node("lgdxrobot2_daemon_node")
{
  cloud = std::make_unique<CloudAdapter>("192.168.1.10:5162", 
    "/home/user/key/rootCA.crt", 
    "/home/user/key/c1.crt", 
    "/home/user/key/c1.key",
    [](const RpcRespond *respond){
      std::cout << "test" << std::endl;
    },
    [this](const char *message, int level){
      logCallback(message, level);
    });

  //autoTaskPublisher = this->create_publisher<lgdxrobot2_daemon::msg::AutoTask>("/daemon/autotask", rclcpp::SensorDataQoS().reliable());
  //autoTaskPublisherTimer = this->create_wall_timer(20ms, std::bind(&DaemonNode::autoTaskPublisherTimerCallback, this));
}

void DaemonNode::logCallback(const char *msg, int level)
{
  switch(level)
  {
    case RCLCPP_LOG_MIN_SEVERITY_INFO:
      RCLCPP_INFO(this->get_logger(), "%s", msg);
      break;
    case RCLCPP_LOG_MIN_SEVERITY_ERROR:
      RCLCPP_ERROR(this->get_logger(), "%s", msg);
      break;
  }
}

void DaemonNode::autoTaskPublisherTimerCallback()
{
  autoTaskPublisher->publish(currentTask);
}