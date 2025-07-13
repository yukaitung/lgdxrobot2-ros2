#ifndef AGENT_HPP
#define AGENT_HPP

#include "rclcpp/rclcpp.hpp"

#include "Mcu.hpp"

class Agent : public rclcpp::Node
{
  private:
    std::unique_ptr<Mcu> mcu;

  public:
    Agent();
    void Initalise();
};

#endif // AGENT_HPP