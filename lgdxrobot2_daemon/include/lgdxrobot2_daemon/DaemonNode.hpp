#ifndef DAEMON_NODE_HPP
#define DAEMON_NODE_HPP

#include <queue>

#include "CloudAdapter.hpp"

#include "rclcpp/rclcpp.hpp"
#include "lgdxrobot2_daemon/msg/auto_task.hpp"

class DaemonNode : public rclcpp::Node
{
  private:
    // Cloud
    std::unique_ptr<CloudAdapter> cloud;
    std::queue<CloudFunctions> cloudErrorQueue;
    rclcpp::TimerBase::SharedPtr cloudRetryTimer;

    // Auto Task
    lgdxrobot2_daemon::msg::AutoTask currentTask;
    rclcpp::Publisher<lgdxrobot2_daemon::msg::AutoTask>::SharedPtr autoTaskPublisher;
    rclcpp::TimerBase::SharedPtr autoTaskPublisherTimer;

    void logCallback(const char *msg, int level);
    void autoTaskPublisherTimerCallback();
    
    void cloudRetry();
    void cloudGreet();
    void cloudExchange();
    void cloudAutoTaskNext();
    void cloudautoTaskAbort();
  public:
    DaemonNode();
};

#endif // DAEMON_NODE_HPP