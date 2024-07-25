#include "DaemonNode.hpp"

#include <random>

DaemonNode::DaemonNode() : Node("lgdxrobot2_daemon_node")
{
  cloud = std::make_unique<CloudAdapter>("192.168.1.10:5162", 
    "/home/user/key/rootCA.crt", 
    "/home/user/key/c1.crt", 
    "/home/user/key/c1.key",
    [](const RpcRespond *respond){
    },
    [this](const char *message, int level){
      logCallback(message, level);
    },
    [this](CloudFunctions function){
      cloudErrorQueue.push(function);
      if (cloudRetryTimer->is_canceled()) {
        cloudRetryTimer->reset();
      }
    });
  
  std::random_device rd;
  std::mt19937 gen(rd());
  std::uniform_int_distribution<> dis(1000, 6000);
  int cloudRetryWait = dis(gen);
  RCLCPP_INFO(this->get_logger(), "The break for reconnection to the cloud is %d ms", cloudRetryWait);
  cloudRetryTimer = this->create_wall_timer(std::chrono::milliseconds(cloudRetryWait), 
    std::bind(&DaemonNode::cloudRetry, this));
  cloudRetryTimer->cancel();
  
  //autoTaskPublisher = this->create_publisher<lgdxrobot2_daemon::msg::AutoTask>("/daemon/autotask", rclcpp::SensorDataQoS().reliable());
  //autoTaskPublisherTimer = this->create_wall_timer(20ms, std::bind(&DaemonNode::autoTaskPublisherTimerCallback, this));

  // Initialised everything
  cloudGreet();
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

void DaemonNode::cloudRetry()
{
  cloudRetryTimer->cancel();
  // Avoid inf loop if queue increase
  for (int i = 0, size = cloudErrorQueue.size(); i < size; i++)
  {
    CloudFunctions function = cloudErrorQueue.front();
    switch (function)
    {
      case CloudFunctions::Greet:
        cloudGreet();
        break;
      case CloudFunctions::Exchange:
        cloudExchange();
        break;
      case CloudFunctions::AutoTaskNext:
        cloudAutoTaskNext();
        break;
      case CloudFunctions::AutoTaskAbort:
        cloudautoTaskAbort();
        break;
    }
    cloudErrorQueue.pop();
  }
}

void DaemonNode::cloudGreet()
{
  cloud->greet();
}

void DaemonNode::cloudExchange()
{

}

void DaemonNode::cloudAutoTaskNext()
{

}

void DaemonNode::cloudautoTaskAbort()
{

}