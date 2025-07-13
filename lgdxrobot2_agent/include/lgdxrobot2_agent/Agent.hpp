#ifndef AGENT_HPP
#define AGENT_HPP

#include "rclcpp/rclcpp.hpp"

#include "Cloud.hpp"
#include "Mcu.hpp"
#include "Navigation.hpp"
#include "RobotStatus.hpp"
#include "Sensors.hpp"

class Agent : public rclcpp::Node
{
  private:
    std::shared_ptr<RobotStatus> robotStatus;
    std::unique_ptr<Cloud> cloud;
    std::unique_ptr<Mcu> mcu;
    std::unique_ptr<Navigation> navigation;
    std::unique_ptr<Sensors> sensors;

  public:
    Agent();
    void Initalise();
};

#endif // AGENT_HPP