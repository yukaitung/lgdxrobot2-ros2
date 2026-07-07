#include <chrono>
#include <rclcpp_components/register_node_macro.hpp>

#include "lgdxrobot2_agent/agent.hpp"

namespace LgdxRobot2 
{
  
Agent::Agent(const rclcpp::NodeOptions &options) : Node("lgdxrobot2_agent_node", options)
{
  timer_ = this->create_wall_timer(std::chrono::microseconds(1), [this]() {this->Initalise();});
}

void Agent::Initalise()
{
  timer_->cancel();

  // Parameters
  // MCU
  auto mcu_port_name_param = rcl_interfaces::msg::ParameterDescriptor{};
  mcu_port_name_param.description = "Serial port name for the LGDXRobot2 or default to /dev/lgdxrobot2.";
  this->declare_parameter("serial_port_name", "/dev/lgdxrobot2", mcu_port_name_param);
  auto mcu_reset_transform_param = rcl_interfaces::msg::ParameterDescriptor{};
  mcu_reset_transform_param.description = "Reset robot transform on start up.";
  this->declare_parameter("reset_transform", false, mcu_reset_transform_param);

  // Sensors
  auto tf_param = rcl_interfaces::msg::ParameterDescriptor{};
  tf_param.description = "Publishing tf information from the robot.";
  this->declare_parameter("publish_tf", false, tf_param);
  auto base_link_param = rcl_interfaces::msg::ParameterDescriptor{};
  base_link_param.description = "Custom `base_link` name when publishing tf information.";
  this->declare_parameter("base_link_name", "base_link", base_link_param);
  auto use_joy_param = rcl_interfaces::msg::ParameterDescriptor{};
  use_joy_param.description = "Control robot using `joy_node`.";
  this->declare_parameter("use_joy", false, use_joy_param);
  auto use_keyboard_param = rcl_interfaces::msg::ParameterDescriptor{};
  use_keyboard_param.description = "Control the robot using `teleop_twist_keyboard`.";
  this->declare_parameter("use_keyboard", false, use_keyboard_param);

  mcu_signals_ = std::make_shared<McuSignals>();
  sensor_signals_ = std::make_shared<SensorSignals>();

  mcu_ = std::make_unique<Mcu>(shared_from_this(), mcu_signals_);
  sensors_ = std::make_unique<Sensors>(shared_from_this(), sensor_signals_);

  mcu_signals_->update_mcu_data.connect(boost::bind(&Sensors::Publish, sensors_.get(), boost::placeholders::_1));

  sensor_signals_->set_estop.connect(boost::bind(&Mcu::SetEstop, mcu_.get(), boost::placeholders::_1));
  sensor_signals_->set_inverse_kinematics.connect(boost::bind(&Mcu::SetInverseKinematics, mcu_.get(), 
    boost::placeholders::_1, boost::placeholders::_2, boost::placeholders::_3));
}

}

RCLCPP_COMPONENTS_REGISTER_NODE(LgdxRobot2::Agent)
