#include "McuNode.hpp"

void McuNode::serialDebugCallback(const std::string &msg, int level)
{
  switch(level)
  {
    case RCLCPP_LOG_MIN_SEVERITY_INFO:
      RCLCPP_INFO(this->get_logger(), "%s", msg.c_str());
      break;
    case RCLCPP_LOG_MIN_SEVERITY_ERROR:
      RCLCPP_ERROR(this->get_logger(), "%s", msg.c_str());
      break;
  }
}

void McuNode::serialReadCallback(const McuData& data)
{
  RCLCPP_INFO(this->get_logger(), "%f", data.measuredWheelVelocity[0]);
}

void McuNode::joyCallback(const sensor_msgs::msg::Joy &msg)
{
  // E-Stop
  if(lastEstopButton[0] == 0 && msg.buttons[0] == 1)
  {
    // A = disable software E-Stop
    serial.setEstop(false);
    RCLCPP_INFO(this->get_logger(), "Software E-Stop Enabled");
  }
  lastEstopButton[0] = msg.buttons[0];
  if(lastEstopButton[1] == 0 && msg.buttons[1] == 1)
  {
    // B = enable software E-Stop
    serial.setEstop(true);
    RCLCPP_INFO(this->get_logger(), "Software E-Stop Disabled");
  }
  lastEstopButton[1] = msg.buttons[1];
  // Velocity Change
  if(lastVelocityChangeButton[0] == 0 && msg.buttons[6] == 1)
  {
    // LB = decrease max velocity
    if(maximumVelocity >= 0.2) 
    {
      maximumVelocity -= 0.1;
      RCLCPP_INFO(this->get_logger(), "Maximum velocity decreased to %.1f m/s", maximumVelocity);
    }
  }
  lastVelocityChangeButton[0] = msg.buttons[6];
  if(lastVelocityChangeButton[1] == 0 && msg.buttons[7] == 1)
  {
    // RB = increase max velocity
    if(maximumVelocity < 1.0)
    {
      maximumVelocity += 0.1;
      RCLCPP_INFO(this->get_logger(), "Maximum velocity increased to %.1f m/s", maximumVelocity);
    } 
  }
  lastVelocityChangeButton[1] = msg.buttons[7];
  // Control IK
  // Left Stick = XY
  float x = msg.axes[1] * maximumVelocity;
  float y = msg.axes[0] * maximumVelocity;
  if(msg.axes[1] == 0 && msg.axes[0] == 0)
  {
    // Use D-pad = XY if Left Stick no input
    x = msg.axes[7] * maximumVelocity;
    y = msg.axes[6] * maximumVelocity;
  }
  // LT = w left, RT = w right
  float w = (((msg.axes[4] - 1) / 2) - ((msg.axes[5] - 1) / 2)) * maximumVelocity;
  serial.setInverseKinematics(x, y, w);
}

McuNode::McuNode() : Node("mcu_node"), serial(std::bind(&McuNode::serialReadCallback, this, std::placeholders::_1), std::bind(&McuNode::serialDebugCallback, this, std::placeholders::_1, std::placeholders::_2))
{
  joySubscription = this->create_subscription<sensor_msgs::msg::Joy>("joy", 10, std::bind(&McuNode::joyCallback, this, std::placeholders::_1));
}