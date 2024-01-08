#include "McuNode.hpp"

void McuNode::serialPortReadDone(const McuData& data)
{
  //RCLCPP_INFO(this->get_logger(), "w1t: %f, w2t: %f, w3t: %f, w4t: %f", data.targetWheelVelocity[0], data.targetWheelVelocity[1], data.targetWheelVelocity[2], data.targetWheelVelocity[3]);
}

McuNode::McuNode(std::shared_ptr<SerialPort> s) : Node("mcu_node"), serial(s)
{
  serial->setReadCallback(std::bind(&McuNode::serialPortReadDone, this, std::placeholders::_1));
  joySubscription = this->create_subscription<sensor_msgs::msg::Joy>("joy", 10, std::bind(&McuNode::joyCallback, this, std::placeholders::_1));
}

void McuNode::joyCallback(const sensor_msgs::msg::Joy &msg) const
{
  float x = msg.axes[1] * 0.2;
  float y = msg.axes[0] * 0.2;
  float w = (-((msg.axes[4] - 1) / 2) + ((msg.axes[5] - 1) / 2)) * 0.2;
  //RCLCPP_INFO(this->get_logger(), "x: %f, y: %f, w: %f", x, y ,w);
  serial->setInverseKinematics(x, y, w);
}