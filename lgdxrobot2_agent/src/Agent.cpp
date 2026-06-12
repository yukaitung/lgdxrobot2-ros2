#include <chrono>
#include <rclcpp_components/register_node_macro.hpp>
#include "lgdxrobot2_agent/Agent.hpp"

namespace LgdxRobot2 
{
  
Agent::Agent(const rclcpp::NodeOptions &options) : Node("lgdxrobot2_agent_node", options)
{
  timer = this->create_wall_timer(std::chrono::microseconds(1), [this]() {this->Initalise();});
}

void Agent::Initalise()
{
  timer->cancel();

  mcuSignals = std::make_shared<McuSignals>();
  sensorSignals = std::make_shared<SensorSignals>();

  mcu = std::make_unique<Mcu>(shared_from_this(), mcuSignals);
  sensors = std::make_unique<Sensors>(shared_from_this(), sensorSignals);

  mcuSignals->UpdateMcuData.connect(boost::bind(&Sensors::Publish, sensors.get(), boost::placeholders::_1));

  sensorSignals->SetEstop.connect(boost::bind(&Mcu::SetEstop, mcu.get(), boost::placeholders::_1));
  sensorSignals->SetInverseKinematics.connect(boost::bind(&Mcu::SetInverseKinematics, mcu.get(), 
    boost::placeholders::_1, boost::placeholders::_2, boost::placeholders::_3));
}

}

RCLCPP_COMPONENTS_REGISTER_NODE(LgdxRobot2::Agent)
