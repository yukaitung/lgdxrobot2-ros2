#include <random>

#include "DaemonNode.hpp"

DaemonNode::DaemonNode() : Node("lgdxrobot2_daemon_node")
{
  cloud = std::make_unique<CloudAdapter>("192.168.1.10:5162", 
    "/home/user/key/rootCA.crt", 
    "/home/user/key/c1.crt", 
    "/home/user/key/c1.key",
    [this](){
      cloudExchangeTimer->reset();
    },
    [this](const RpcRespond *respond){
			if (respond->has_task())
			{
				RpcAutoTask task = respond->task();
				currentTask.task_id = task.taskid();
				currentTask.task_name = task.taskname();
				currentTask.task_progress_id = task.taskprogressid();
				currentTask.task_progress_name = task.taskprogressname();
				currentTask.complete_token = task.completetoken();
			}
    },
    [this](const char *message, int level){
      logCallback(message, level);
    },
    [this](CloudFunctions function){
      cloudErrorQueue.push(function);
      if (cloudRetryTimer->is_canceled())
        cloudRetryTimer->reset();
    });
  
  std::random_device rd;
  std::mt19937 gen(rd());
  std::uniform_int_distribution<> dis(1000, 6000);
  int cloudRetryWait = dis(gen);
  RCLCPP_INFO(this->get_logger(), "The break for reconnection to the cloud is %d ms", cloudRetryWait);
  cloudRetryTimer = this->create_wall_timer(std::chrono::milliseconds(cloudRetryWait), 
    std::bind(&DaemonNode::cloudRetry, this));
  cloudRetryTimer->cancel();

  cloudExchangeTimer = this->create_wall_timer(std::chrono::milliseconds(500), 
    std::bind(&DaemonNode::cloudExchange, this));
  cloudExchangeTimer->cancel();
  
  autoTaskPublisher = this->create_publisher<lgdxrobot2_daemon::msg::AutoTask>("/daemon/autotask", 
		rclcpp::SensorDataQoS().reliable());
  autoTaskPublisherTimer = this->create_wall_timer(std::chrono::milliseconds(100), 
		[this](){
			autoTaskPublisher->publish(currentTask);
		});

  autoTaskNextService = this->create_service<lgdxrobot2_daemon::srv::AutoTaskNext>("auto_task_next",
    [this](const std::shared_ptr<lgdxrobot2_daemon::srv::AutoTaskNext::Request> request,
      std::shared_ptr<lgdxrobot2_daemon::srv::AutoTaskNext::Response> response) 
    {
      if (!currentTask.complete_token.empty() && (request->next_token == currentTask.complete_token))
      {
        cloudAutoTaskNext();
        response->success = true;
      }
      else
      {
        response->success = false;
      }
    });
  autoTaskAbortService = this->create_service<lgdxrobot2_daemon::srv::AutoTaskAbort>("auto_task_abort",
    [this](const std::shared_ptr<lgdxrobot2_daemon::srv::AutoTaskAbort::Request> request,
      std::shared_ptr<lgdxrobot2_daemon::srv::AutoTaskAbort::Response> response)
    {
      if (!currentTask.complete_token.empty() && (request->next_token == currentTask.complete_token))
      {
        cloudautoTaskAbort();
        response->success = true;
      }
      else
      {
        response->success = false;
      }
    });

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

void DaemonNode::cloudRetry()
{
  cloudRetryTimer->cancel();
  // Avoid inf loop if the queue increases
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
  if (!cloudExchangeTimer->is_canceled())
    cloudExchangeTimer->cancel();
  cloud->exchange();
  // Don't reset the cloudExchangeTimer here
}

void DaemonNode::cloudAutoTaskNext()
{
  if (!currentTask.complete_token.empty())
  {
    RpcCompleteToken token;
    token.set_taskid(currentTask.task_id);
    token.set_token(currentTask.complete_token);
    cloud->autoTaskNext(token);
  }
}

void DaemonNode::cloudautoTaskAbort()
{
  if (!currentTask.complete_token.empty())
  {
    RpcCompleteToken token;
    token.set_taskid(currentTask.task_id);
    token.set_token(currentTask.complete_token);
    cloud->autoTaskAbort(token);
  }
}