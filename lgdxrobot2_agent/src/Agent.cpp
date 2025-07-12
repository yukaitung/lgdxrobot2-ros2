#include "lgdxrobot2_agent/Agent.hpp"

Agent::Agent() : Node("lgdxrobot2_agent_node")
{
  RCLCPP_INFO(this->get_logger(), "LGDXRobot2 Agent is running.");
}