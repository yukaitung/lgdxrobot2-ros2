#ifndef MCUNODE_HPP
#define MCUNODE_HPP

#include "rclcpp/rclcpp.hpp"

#include "SerialPort.hpp"

class McuNode : public rclcpp::Node
{
  private:
    std::shared_ptr<SerialPort> serial;

    void serialPortReadDone(const McuData& data);
    
  public:
    McuNode(std::shared_ptr<SerialPort> s);
};

#endif // MCUNODE_HPP
