#ifndef MCUNODE_HPP
#define MCUNODE_HPP

#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/joy.hpp"

#include "SerialPort.hpp"

class McuNode : public rclcpp::Node
{
  private:
    // Object
    std::shared_ptr<SerialPort> serial;
    rclcpp::Subscription<sensor_msgs::msg::Joy>::SharedPtr joySubscription;

    float maximumVelocity = 0.1; // m/s
    int lastSecondButton[2] = {0};

    void serialPortReadDone(const McuData &data);
    void joyCallback(const sensor_msgs::msg::Joy &msg);
    
  public:
    McuNode(std::shared_ptr<SerialPort> s);
};

#endif // MCUNODE_HPP
