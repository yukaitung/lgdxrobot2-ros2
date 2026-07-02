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

  // Parameters
  // MCU
  auto mcuPortNameParam = rcl_interfaces::msg::ParameterDescriptor{};
  mcuPortNameParam.description = "Serial port name for the LGDXRobot2 or default to /dev/lgdxrobot2.";
  this->declare_parameter("serial_port_name", "/dev/lgdxrobot2", mcuPortNameParam);
  auto mcuResetTransformParam = rcl_interfaces::msg::ParameterDescriptor{};
  mcuResetTransformParam.description = "Reset robot transform on start up.";
  this->declare_parameter("reset_transform", false, mcuResetTransformParam);

  // Sensors
  auto tfParam = rcl_interfaces::msg::ParameterDescriptor{};
  tfParam.description = "Publishing tf information from the robot.";
  this->declare_parameter("publish_tf", false, tfParam);
  auto baseLinkParam = rcl_interfaces::msg::ParameterDescriptor{};
  baseLinkParam.description = "Custom `base_link` name when publishing tf information.";
  this->declare_parameter("base_link_name", "base_link", baseLinkParam);
  auto useJoyParam = rcl_interfaces::msg::ParameterDescriptor{};
  useJoyParam.description = "Control robot using `joy_node`.";
  this->declare_parameter("use_joy", false, useJoyParam);
  auto useKeyboard = rcl_interfaces::msg::ParameterDescriptor{};
  useKeyboard.description = "Control the robot using `teleop_twist_keyboard`.";
  this->declare_parameter("use_keyboard", false, useJoyParam);


  mcuSignals = std::make_shared<McuSignals>();
  sensorSignals = std::make_shared<SensorSignals>();

  mcu = std::make_unique<Mcu>(shared_from_this(), mcuSignals);
  sensors = std::make_unique<Sensors>(shared_from_this(), sensorSignals);

  mcuSignals->UpdateMcuData.connect(boost::bind(&Sensors::Publish, sensors.get(), boost::placeholders::_1));

  sensorSignals->SetEstop.connect(boost::bind(&Mcu::SetEstop, mcu.get(), boost::placeholders::_1));
  sensorSignals->SetInverseKinematics.connect(boost::bind(&Mcu::SetInverseKinematics, mcu.get(), 
    boost::placeholders::_1, boost::placeholders::_2, boost::placeholders::_3));

  rclcpp::on_shutdown([this]()
  {
    if (this->mcu)
    {
      this->mcu->Shutdown();
    }
  });
}

}

RCLCPP_COMPONENTS_REGISTER_NODE(LgdxRobot2::Agent)
