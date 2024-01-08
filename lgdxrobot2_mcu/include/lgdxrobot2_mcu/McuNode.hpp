#ifndef MCUNODE_HPP
#define MCUNODE_HPP

#include "rclcpp/rclcpp.hpp"

class McuNode : public rclcpp::Node
{
  private:
    int a = 0;

  public:
    McuNode();
    
};

#endif // MCUNODE_HPP