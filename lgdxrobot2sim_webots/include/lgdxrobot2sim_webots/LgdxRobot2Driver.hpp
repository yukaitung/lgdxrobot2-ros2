#ifndef LGDXROBOT2DRIVER_HPP
#define LGDXROBOT2DRIVER_HPP

#include "geometry_msgs/msg/twist.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include "rclcpp/macros.hpp"
#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/joint_state.hpp"
#include "webots_ros2_driver/PluginInterface.hpp"
#include "webots_ros2_driver/WebotsNode.hpp"

// Odom
#include "tf2_ros/transform_broadcaster.h"
#include <string>

#define CHASSIS_LX 0.082
#define CHASSIS_LY 0.104
#define WHEEL_RADIUS 0.0375

namespace LgdxRobot2 
{
class LgdxRobot2Driver : public webots_ros2_driver::PluginInterface
{
  private:
    webots_ros2_driver::WebotsNode *rosNode;

    WbDeviceTag wheels[4]; // Wheel1 - Wheel4
    double wheelsVelocity[4] = {0}; // Wheel1 - Wheel4
    rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr cmdVelSubscription;

    bool isCrticialStatus = false;
    
    double lastSimTime = 0;
    double motorLastPosition[4] = {0}; // Wheel1 - Wheel4
    double robotTransform[3] = {0}; // x, y, rotation
    WbDeviceTag positionSensors[4]; // Wheel1 - Wheel4
    rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr odomPublisher;
    std::shared_ptr<tf2_ros::TransformBroadcaster> tfBroadcaster;
    rclcpp::Publisher<sensor_msgs::msg::JointState>::SharedPtr jointStatePublisher;

    WbDeviceTag inertialUnit;

    void cmdVelCallback(const geometry_msgs::msg::Twist &msg);

  public:
    void init(webots_ros2_driver::WebotsNode *node, std::unordered_map<std::string, std::string> &parameters) override;
    void step() override;
};
}

#endif