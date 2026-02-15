#ifndef AGENT_HPP
#define AGENT_HPP

#include "rclcpp/rclcpp.hpp"

#include "Mcu.hpp"
#include "Sensors.hpp"

class Agent : public rclcpp::Node
{
  private:
    std::unique_ptr<Mcu> mcu;
    std::unique_ptr<Sensors> sensors;

    std::shared_ptr<McuSignals> mcuSignals;
    std::shared_ptr<SensorSignals> sensorSignals;

  public:
    Agent();
    void Initalise();
    void Shutdown();
};

#endif // AGENT_HPP