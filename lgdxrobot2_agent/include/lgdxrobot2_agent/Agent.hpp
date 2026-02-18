#ifndef AGENT_HPP
#define AGENT_HPP

#include "rclcpp/rclcpp.hpp"

#include "Mcu.hpp"
#include "Sensors.hpp"
#include "Cloud.hpp"

namespace LgdxRobot2 
{

class Agent : public rclcpp::Node
{
  private:
    rclcpp::TimerBase::SharedPtr timer;

    std::unique_ptr<Mcu> mcu;
    std::unique_ptr<Sensors> sensors;
    std::unique_ptr<Cloud> cloud;

    std::shared_ptr<McuSignals> mcuSignals;
    std::shared_ptr<SensorSignals> sensorSignals;
    std::shared_ptr<CloudSignals> cloudSignals;

  public:
    Agent(const rclcpp::NodeOptions &options);
    void Initalise();
};

}

#endif // AGENT_HPP