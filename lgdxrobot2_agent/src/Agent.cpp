#include "lgdxrobot2_agent/Agent.hpp"

Agent::Agent() : Node("lgdxrobot2_agent_node")
{}

void Agent::Initalise()
{
  auto mcuEnableParam = rcl_interfaces::msg::ParameterDescriptor{};
  mcuEnableParam.description = "Enable LGDXRobot2 MCU.";
  this->declare_parameter("mcu_enable", false, mcuEnableParam);

  // MCU
  bool mcuEnable = this->get_parameter("mcu_enable").as_bool();
  if (1)
  {
    mcu = std::make_unique<Mcu>(shared_from_this());
    sensors = std::make_unique<Sensors>(shared_from_this());
  }
}