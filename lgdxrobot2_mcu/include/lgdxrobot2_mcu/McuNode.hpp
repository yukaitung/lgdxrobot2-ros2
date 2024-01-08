#ifndef MCUNODE_HPP
#define MCUNODE_HPP

#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/joy.hpp"

#include "SerialPort.hpp"

class McuNode : public rclcpp::Node
{
  private:
    std::shared_ptr<SerialPort> serial;
    rclcpp::Subscription<sensor_msgs::msg::Joy>::SharedPtr joySubscription;

    void serialPortReadDone(const McuData &data);
    void joyCallback(const sensor_msgs::msg::Joy &msg) const;
    
  public:
    McuNode(std::shared_ptr<SerialPort> s);
};

#endif // MCUNODE_HPP
