#include <random>

#include "DaemonNode.hpp"

DaemonNode::DaemonNode() : Node("lgdxrobot2_daemon_node")
{
  /*
   * Parameters
   */
  // Cloud
  auto cloud_enable_desc = rcl_interfaces::msg::ParameterDescriptor{};
  cloud_enable_desc.description = "Enable LGDXRobot2 Cloud";
  this->declare_parameter("cloud_enable", false, cloud_enable_desc);
  auto cloud_addr_port_desc = rcl_interfaces::msg::ParameterDescriptor{};
  cloud_addr_port_desc.description = "Address for LGDXRobot2 Cloud";
  this->declare_parameter("cloud_address", "", cloud_addr_port_desc);
  auto cloud_root_cert_desc = rcl_interfaces::msg::ParameterDescriptor{};
  cloud_root_cert_desc.description = "Path to server root certificate, required in LGDXRobot2 Cloud";
  this->declare_parameter("cloud_root_cert", "", cloud_root_cert_desc);
  auto cloud_client_key_desc = rcl_interfaces::msg::ParameterDescriptor{};
  cloud_client_key_desc.description = "Path to client's private key, required in LGDXRobot2 Cloud";
  this->declare_parameter("cloud_client_key", "", cloud_client_key_desc);
  auto cloud_client_cert_desc = rcl_interfaces::msg::ParameterDescriptor{};
  cloud_client_cert_desc.description = "Path to client's certificate chain, required in LGDXRobot2 Cloud";
  this->declare_parameter("cloud_client_cert", "", cloud_client_cert_desc);

  // Serial Port
  auto serial_param_desc = rcl_interfaces::msg::ParameterDescriptor{};
  serial_param_desc.description = "Default serial port name or (Linux only) perform automated search if the port name is unspecified.";
  this->declare_parameter("serial_port", "", serial_param_desc);
  auto control_param_desc = rcl_interfaces::msg::ParameterDescriptor{};
  control_param_desc.description = "Robot control mode, using `joy` for joystick or `cmd_vel` for ROS nav stack.";
  this->declare_parameter("control_mode", "joy", control_param_desc);
  auto use_external_imu_param_desc = rcl_interfaces::msg::ParameterDescriptor{};
  use_external_imu_param_desc.description = "Using external IMU for odometry calcuation.";
  this->declare_parameter("use_external_imu", false, use_external_imu_param_desc);
  auto reset_transform_param_desc = rcl_interfaces::msg::ParameterDescriptor{};
  reset_transform_param_desc.description = "Reset robot transform on start up.";
  this->declare_parameter("reset_transform", false, reset_transform_param_desc);

  // ROS
  auto odom_param_desc = rcl_interfaces::msg::ParameterDescriptor{};
  odom_param_desc.description = "Publishing odometry information from the robot.";
  this->declare_parameter("publish_odom", false, odom_param_desc);
  auto tf_param_desc = rcl_interfaces::msg::ParameterDescriptor{};
  tf_param_desc.description = "Publishing tf information from the robot.";
  this->declare_parameter("publish_tf", false, tf_param_desc);
  auto base_link_frame_param_desc = rcl_interfaces::msg::ParameterDescriptor{};
  base_link_frame_param_desc.description = "Custom base_link name.";
  this->declare_parameter("base_link_frame", "base_link", base_link_frame_param_desc);
  
  /*
   * Object Init
   */
  bool cloudEnable = this->get_parameter("cloud_enable").as_bool();
  if (cloudEnable)
  {
    std::string cloudAddress = this->get_parameter("cloud_address").as_string();
    std::string root = this->get_parameter("cloud_root_cert").as_string();
    std::string key = this->get_parameter("cloud_client_key").as_string();
    std::string cert = this->get_parameter("cloud_client_cert").as_string();
    cloud = std::make_unique<CloudAdapter>(cloudAddress.c_str(), 
      root.c_str(), 
      key.c_str(),
      cert.c_str(), 
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
          currentTask.next_token = task.nexttoken();
          robotIdle = false;
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
    RCLCPP_INFO(this->get_logger(), "LGDXRobot2 Cloud is enabled, the break for reconnection to the cloud is %d ms", cloudRetryWait);
    cloudRetryTimer = this->create_wall_timer(std::chrono::milliseconds(cloudRetryWait), 
      std::bind(&DaemonNode::cloudRetry, this));
    cloudRetryTimer->cancel();
    
    cloudExchangeTimer = this->create_wall_timer(std::chrono::milliseconds(500), 
      std::bind(&DaemonNode::cloudExchange, this));
    cloudExchangeTimer->cancel();
    
    autoTaskPublisher = this->create_publisher<lgdxrobot2_daemon::msg::AutoTask>("/daemon/auto_task", 
      rclcpp::SensorDataQoS().reliable());
    autoTaskPublisherTimer = this->create_wall_timer(std::chrono::milliseconds(100), 
      [this](){
        autoTaskPublisher->publish(currentTask);
      });

    autoTaskNextService = this->create_service<lgdxrobot2_daemon::srv::AutoTaskNext>("auto_task_next",
      [this](const std::shared_ptr<lgdxrobot2_daemon::srv::AutoTaskNext::Request> request,
        std::shared_ptr<lgdxrobot2_daemon::srv::AutoTaskNext::Response> response) 
      {
        if (!currentTask.next_token.empty() && (request->next_token == currentTask.next_token))
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
        if (!currentTask.next_token.empty() && (request->next_token == currentTask.next_token))
        {
          cloudautoTaskAbort();
          response->success = true;
        }
        else
        {
          response->success = false;
        }
      });
  }
  
  // Initialised everything
  if (cloud != nullptr)
  {
    cloudGreet();
  }
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

  bool getTask = robotIdle == true && robotStopped == false;

  cloud->exchange(getTask);
  // Don't reset the cloudExchangeTimer here
}

void DaemonNode::cloudAutoTaskNext()
{
  if (!currentTask.next_token.empty())
  {
    RpcNextToken token;
    token.set_taskid(currentTask.task_id);
    token.set_token(currentTask.next_token);
    cloud->autoTaskNext(token);
    robotIdle = true;
  }
}

void DaemonNode::cloudautoTaskAbort()
{
  if (!currentTask.next_token.empty())
  {
    RpcNextToken token;
    token.set_taskid(currentTask.task_id);
    token.set_token(currentTask.next_token);
    cloud->autoTaskAbort(token);
    robotIdle = true;
  }
}