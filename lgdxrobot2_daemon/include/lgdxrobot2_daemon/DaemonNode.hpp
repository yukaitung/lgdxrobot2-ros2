#ifndef DAEMON_NODE_HPP
#define DAEMON_NODE_HPP

#include "rclcpp/rclcpp.hpp"
#include "lgdxrobot2_daemon/msg/auto_task.hpp"
#include "proto/RobotClientService.grpc.pb.h"

class DaemonNode : public rclcpp::Node
{
  private:
    lgdxrobot2_daemon::msg::AutoTask currentTask;
    rclcpp::Publisher<lgdxrobot2_daemon::msg::AutoTask>::SharedPtr autoTaskPublisher;
    rclcpp::TimerBase::SharedPtr autoTaskPublisherTimer;

    void autoTaskPublisherTimerCallback();

  public:
    DaemonNode();
};

#endif // DAEMON_NODE_HPP