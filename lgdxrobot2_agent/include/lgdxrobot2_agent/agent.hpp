#ifndef AGENT_HPP
#define AGENT_HPP

#include "rclcpp/rclcpp.hpp"

#include "Mcu.hpp"
#include "sensors.hpp"

namespace LgdxRobot2 
{

class Agent : public rclcpp::Node
{
  public:
    Agent(const rclcpp::NodeOptions &options);
    void Initalise();

  private:
    rclcpp::TimerBase::SharedPtr timer_;

    std::unique_ptr<Mcu> mcu_;
    std::unique_ptr<Sensors> sensors_;

    std::shared_ptr<McuSignals> mcu_signals_;
    std::shared_ptr<SensorSignals> sensor_signals_;
};

}

#endif // AGENT_HPP