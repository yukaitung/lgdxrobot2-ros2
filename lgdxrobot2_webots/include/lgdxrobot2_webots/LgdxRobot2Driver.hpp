#ifndef LGDXROBOT2DRIVER_HPP
#define LGDXROBOT2DRIVER_HPP

#include "geometry_msgs/msg/twist.hpp"
#include "lgdxrobot2_agent/msg/robot_data.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include "rclcpp/macros.hpp"
#include "rclcpp/rclcpp.hpp"
#include "webots_ros2_driver/PluginInterface.hpp"
#include "webots_ros2_driver/WebotsNode.hpp"

// Odom
#include "tf2_ros/transform_broadcaster.h"

#define CHASSIS_LX 0.24
#define CHASSIS_LY 0.24
#define WHEEL_RADIUS 0.0375

namespace LgdxRobot2 
{
class LgdxRobot2Driver : public webots_ros2_driver::PluginInterface
{
  private:
    webots_ros2_driver::WebotsNode *rosNode;

    WbDeviceTag wheels[4];
    double wheelsVelocity[4] = {0};
    rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr cmdVelSubscription;

    bool isCrticialStatus = false;
    rclcpp::Subscription<lgdxrobot2_agent::msg::RobotData>::SharedPtr robotStatusSubscription;
    
    double lastSimTime = 0;
    double motorLastPosition[4] = {0};
    double robotTransform[3] = {0};
    WbDeviceTag positionSensors[4];
    rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr odomPublisher;
    std::shared_ptr<tf2_ros::TransformBroadcaster> tfBroadcaster;

    WbDeviceTag inertialUnit;

    void cmdVelCallback(const geometry_msgs::msg::Twist &msg);

  public:
    void init(webots_ros2_driver::WebotsNode *node, std::unordered_map<std::string, std::string> &parameters) override;
    void step() override;
};
}

#endif