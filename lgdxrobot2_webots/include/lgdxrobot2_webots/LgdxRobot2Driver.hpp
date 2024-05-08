#ifndef LGDXROBOT2DRIVER_HPP
#define LGDXROBOT2DRIVER_HPP

#include "rclcpp/rclcpp.hpp"
#include "rclcpp/macros.hpp"
#include "geometry_msgs/msg/twist.hpp"
#include "webots_ros2_driver/PluginInterface.hpp"
#include "webots_ros2_driver/WebotsNode.hpp"

#define CHASSIS_LX 0.237
#define CHASSIS_LY 0.287
#define WHEEL_RADIUS 0.0375

namespace LgdxRobot2 
{
class LgdxRobot2Driver : public webots_ros2_driver::PluginInterface
{
  private:
    void cmdVelCallback(const geometry_msgs::msg::Twist &msg);
    rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr cmdVelSubscription;
    WbDeviceTag wheels[4];
    float wheelsVelocity[4] = {0};

  public:
    void init(webots_ros2_driver::WebotsNode *node, std::unordered_map<std::string, std::string> &parameters) override;
    void step() override;
};
}

#endif