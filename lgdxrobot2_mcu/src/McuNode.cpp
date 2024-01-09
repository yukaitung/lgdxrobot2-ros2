#include "McuNode.hpp"


void McuNode::serialPortReadDone(const McuData& data)
{
}

McuNode::McuNode(std::shared_ptr<SerialPort> s) : Node("mcu_node"), serial(s)
{
  serial->setReadCallback(std::bind(&McuNode::serialPortReadDone, this, std::placeholders::_1));
  joySubscription = this->create_subscription<sensor_msgs::msg::Joy>("joy", 10, std::bind(&McuNode::joyCallback, this, std::placeholders::_1));
}

void McuNode::joyCallback(const sensor_msgs::msg::Joy &msg)
{
  // Change max velocity, left, decrease
  if(lastSecondButton[0] == 0 && msg.buttons[6] == 1)
  {
    if(maximumVelocity >= 0.2) 
    {
      maximumVelocity -= 0.1;
      RCLCPP_INFO(this->get_logger(), "Maximum velocity decreased to %.1f m/s", maximumVelocity);
    }
  }
  lastSecondButton[0] = msg.buttons[6];
  // Change max velocity, right, increase
  if(lastSecondButton[1] == 0 && msg.buttons[7] == 1)
  {
    if(maximumVelocity < 1.0)
    {
      maximumVelocity += 0.1;
      RCLCPP_INFO(this->get_logger(), "Maximum velocity increased to %.1f m/s", maximumVelocity);
    } 
  }
  lastSecondButton[1] = msg.buttons[7];
  float x = msg.axes[1] * maximumVelocity;
  if(x == 0)
    x = msg.axes[7] * maximumVelocity;
  float y = msg.axes[0] * maximumVelocity;
  if(y == 0)
    y = msg.axes[6] * maximumVelocity;
  float w = (((msg.axes[4] - 1) / 2) - ((msg.axes[5] - 1) / 2)) * maximumVelocity;
  serial->setInverseKinematics(x, y, w);
}