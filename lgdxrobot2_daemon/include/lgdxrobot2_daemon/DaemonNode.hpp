#ifndef DAEMON_NODE_HPP
#define DAEMON_NODE_HPP

#include "CloudAdapter.hpp"

#include "rclcpp/rclcpp.hpp"
#include "lgdxrobot2_daemon/msg/auto_task.hpp"

class DaemonNode : public rclcpp::Node
{
  private:
    std::unique_ptr<CloudAdapter> cloud;

    // Auto Task
    lgdxrobot2_daemon::msg::AutoTask currentTask;
    rclcpp::Publisher<lgdxrobot2_daemon::msg::AutoTask>::SharedPtr autoTaskPublisher;
    rclcpp::TimerBase::SharedPtr autoTaskPublisherTimer;

    void autoTaskPublisherTimerCallback();
    
  public:
    DaemonNode();
};

#endif // DAEMON_NODE_HPP