#ifndef CLOUD_HPP
#define CLOUD_HPP

#include "CloudSignals.hpp"
#include "lgdxrobot_cloud_msgs/msg/robot_data.hpp"
#include "lgdxrobot_cloud_msgs/srv/mcu_sn.hpp"
#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/bool.hpp"
#include "lgdxrobot2.h"

class Cloud
{
  private:
    rclcpp::Node::SharedPtr _node;
    rclcpp::Logger _logger;

    rclcpp::Publisher<lgdxrobot_cloud_msgs::msg::RobotData>::SharedPtr robotDataPublisher;
    rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr softwareEmergencyStopSubscription;
    rclcpp::Client<lgdxrobot_cloud_msgs::srv::McuSn>::SharedPtr mcuSnClient;

    std::shared_ptr<CloudSignals> cloudSignals;
    
  public:
    Cloud(rclcpp::Node::SharedPtr node, std::shared_ptr<CloudSignals> cloudSignalsPtr);
    void PublishRobotData(const McuData &mcuData);
    void PublishMcuSn(const std::string &sn);
};

#endif // CLOUD_HPP