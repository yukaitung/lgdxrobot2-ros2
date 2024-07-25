#ifndef DAEMON_NODE_HPP
#define DAEMON_NODE_HPP

#include <queue>

#include "CloudAdapter.hpp"

#include "rclcpp/rclcpp.hpp"
#include "lgdxrobot2_daemon/msg/auto_task.hpp"
#include "lgdxrobot2_daemon/srv/auto_task_next.hpp"
#include "lgdxrobot2_daemon/srv/auto_task_abort.hpp"

class DaemonNode : public rclcpp::Node
{
  private:
    // Cloud
    bool robotIdle = true;
    bool robotStopped = false;
    lgdxrobot2_daemon::msg::AutoTask currentTask;
    std::unique_ptr<CloudAdapter> cloud;
    std::queue<CloudFunctions> cloudErrorQueue;
    rclcpp::TimerBase::SharedPtr cloudRetryTimer;
    rclcpp::TimerBase::SharedPtr cloudExchangeTimer;
    rclcpp::TimerBase::SharedPtr autoTaskPublisherTimer;
    rclcpp::Publisher<lgdxrobot2_daemon::msg::AutoTask>::SharedPtr autoTaskPublisher;
    rclcpp::Service<lgdxrobot2_daemon::srv::AutoTaskNext>::SharedPtr autoTaskNextService;
    rclcpp::Service<lgdxrobot2_daemon::srv::AutoTaskAbort>::SharedPtr autoTaskAbortService;

    void logCallback(const char *msg, int level);

    void cloudRetry();
    void cloudGreet();
    void cloudExchange();
    void cloudAutoTaskNext();
    void cloudautoTaskAbort();
    
  public:
    DaemonNode();
};

#endif // DAEMON_NODE_HPP