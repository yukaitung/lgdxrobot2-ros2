#include "lgdxrobot2_agent/Agent.hpp"

Agent::Agent() : Node("lgdxrobot2_agent_node")
{}

void Agent::Initalise()
{
  // Signals
  mcuSignals = std::make_shared<McuSignals>();
  sensorSignals = std::make_shared<SensorSignals>();

  mcu = std::make_unique<Mcu>(shared_from_this(), mcuSignals);
  sensors = std::make_unique<Sensors>(shared_from_this(), sensorSignals);

  mcuSignals->UpdateRobotData.connect(boost::bind(&Sensors::PublishOdom, sensors.get(), boost::placeholders::_1));

  sensorSignals->SetEstop.connect(boost::bind(&Mcu::SetEstop, mcu.get(), boost::placeholders::_1));
  sensorSignals->SetInverseKinematics.connect(boost::bind(&Mcu::SetInverseKinematics, mcu.get(), 
    boost::placeholders::_1, boost::placeholders::_2, boost::placeholders::_3));
}

void Agent::Shutdown()
{
  RCLCPP_INFO(this->get_logger(), "Shutting down agent");
  rclcpp::shutdown();
  exit(0);
}