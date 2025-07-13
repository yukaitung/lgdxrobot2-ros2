#include "lgdxrobot2_agent/Agent.hpp"

Agent::Agent() : Node("lgdxrobot2_agent_node")
{}

void Agent::Initalise()
{
  auto cloudEnableParam = rcl_interfaces::msg::ParameterDescriptor{};
  cloudEnableParam.description = "Enable LGDXRobot Cloud.";
  this->declare_parameter("cloud_enable", false, cloudEnableParam);
  auto mcuEnableParam = rcl_interfaces::msg::ParameterDescriptor{};
  mcuEnableParam.description = "Enable LGDXRobot2 MCU.";
  this->declare_parameter("mcu_enable", false, mcuEnableParam);

  // Cloud
  bool cloudEnable = this->get_parameter("cloud_enable").as_bool();
  if (cloudEnable)
  {
    robotStatus = std::make_shared<RobotStatus>();
    cloud = std::make_unique<Cloud>(shared_from_this(), robotStatus);
    navigation = std::make_unique<Navigation>(shared_from_this(), robotStatus);
  }

  // MCU
  bool mcuEnable = this->get_parameter("mcu_enable").as_bool();
  if (mcuEnable)
  {
    mcu = std::make_unique<Mcu>(shared_from_this());
    sensors = std::make_unique<Sensors>(shared_from_this());
  }
}