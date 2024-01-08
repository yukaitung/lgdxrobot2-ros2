#include "McuNode.hpp"

void McuNode::serialPortReadDone(const McuData& data)
{
  RCLCPP_INFO(this->get_logger(), "Received data b1 = %f", data.battery[0]);
}

McuNode::McuNode(std::shared_ptr<SerialPort> s) : Node("mcu_node"), serial(s)
{
  serial->setReadCallback(std::bind(&McuNode::serialPortReadDone, this, std::placeholders::_1));
  RCLCPP_INFO(this->get_logger(), "WELCOME");
}