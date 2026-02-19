#include "lgdxrobot2_agent/Cloud.hpp"

Cloud::Cloud(rclcpp::Node::SharedPtr node, std::shared_ptr<CloudSignals> cloudSignalsPtr) :
  _node(node),
  _logger(node->get_logger())
{
  cloudSignals = cloudSignalsPtr;

  // Publisher
  robotDataPublisher = node->create_publisher<lgdxrobot_cloud_msgs::msg::RobotData>("cloud/robot_data", 
    rclcpp::SensorDataQoS().reliable());

  // Subscription
  softwareEmergencyStopSubscription = node->create_subscription<std_msgs::msg::Bool>(
    "cloud/software_emergency_stop", rclcpp::SensorDataQoS().reliable(),
    [this](const std_msgs::msg::Bool::SharedPtr msg) {
      cloudSignals->SetEstop(msg->data);
    });

  // Client
  mcuSnClient = node->create_client<lgdxrobot_cloud_msgs::srv::McuSn>("mcu_sn");
}

void Cloud::PublishRobotData(const McuData &mcuData)
{
  lgdxrobot_cloud_msgs::msg::RobotData robotData;
  robotData.hardware_emergency_stop_enabled = mcuData.hardware_emergency_stop_enabled | mcuData.bettery_low_emergency_stop_enabled;
  robotData.batteries_voltage = {mcuData.battery1.voltage, mcuData.battery2.voltage};
  robotDataPublisher->publish(robotData);
}

void Cloud::PublishMcuSn(const std::string &sn)
{
  RCLCPP_INFO(_logger, "MCU Serial Number: %s, awaiting LGDXRobot Cloud Adaptor...", sn.c_str());
  while (!mcuSnClient->wait_for_service()) {} // Wait for service to become available
  auto request = std::make_shared<lgdxrobot_cloud_msgs::srv::McuSn::Request>();
  request->mcu_sn = sn;
  auto result = mcuSnClient->async_send_request(request);
  RCLCPP_INFO(_logger, "MCU Serial Number sent.");
}