#ifndef MCUNODE_HPP
#define MCUNODE_HPP

#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/joy.hpp"

#include "SerialPort.hpp"

class McuNode : public rclcpp::Node
{
  private:
    // Object
    SerialPort serial;
    rclcpp::Subscription<sensor_msgs::msg::Joy>::SharedPtr joySubscription;

    // Joy
    float maximumVelocity = 0.1; // m/s
    int lastVelocityChangeButton[2] = {0}; // 0: LB, 1: RB
    int lastEstopButton[2] = {0}; // 0: A, 1: B

    void serialDebugCallback(const std::string &msg, int level);
    void serialReadCallback(const McuData &data);
    void joyCallback(const sensor_msgs::msg::Joy &msg);
    
  public:
    McuNode();
};

#endif // MCUNODE_HPP
