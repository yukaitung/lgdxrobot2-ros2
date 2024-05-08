#include "LgdxRobot2Driver.hpp"

#include <webots/motor.h>
#include <webots/robot.h>

namespace LgdxRobot2 {
void LgdxRobot2Driver::cmdVelCallback(const geometry_msgs::msg::Twist &msg) 
{
  float x = msg.linear.x;
  float y = msg.linear.y;
  float w = msg.angular.z;
  wheelsVelocity[0] = (1 / WHEEL_RADIUS) * (x - y - (CHASSIS_LX + CHASSIS_LY) * w);
  wheelsVelocity[1] = (1 / WHEEL_RADIUS) * (x + y + (CHASSIS_LX + CHASSIS_LY) * w);
  wheelsVelocity[2] = (1 / WHEEL_RADIUS) * (x + y - (CHASSIS_LX + CHASSIS_LY) * w);
  wheelsVelocity[3] = (1 / WHEEL_RADIUS) * (x - y + (CHASSIS_LX + CHASSIS_LY) * w);
}

void LgdxRobot2Driver::init(webots_ros2_driver::WebotsNode *node, std::unordered_map<std::string, std::string> &parameters) 
{
  char wheelsNames[4][8] = {"wheel1", "wheel2", "wheel3", "wheel4"};
  for (int i = 0; i < 4; i++) 
  {
    wheels[i] = wb_robot_get_device(wheelsNames[i]);
    wb_motor_set_position(wheels[i], INFINITY);
    wb_motor_set_velocity(wheels[i], 0.0);
  }

  cmdVelSubscription = node->create_subscription<geometry_msgs::msg::Twist>(
    "cmd_vel", 
    rclcpp::SensorDataQoS().reliable(),
    std::bind(&LgdxRobot2Driver::cmdVelCallback, this, std::placeholders::_1)
  );
}

void LgdxRobot2Driver::step() 
{
  for (int i = 0; i < 4; i++) 
  {
    wb_motor_set_velocity(wheels[i], wheelsVelocity[i]);
  }
}
}

#include "pluginlib/class_list_macros.hpp"
PLUGINLIB_EXPORT_CLASS(LgdxRobot2::LgdxRobot2Driver, webots_ros2_driver::PluginInterface)