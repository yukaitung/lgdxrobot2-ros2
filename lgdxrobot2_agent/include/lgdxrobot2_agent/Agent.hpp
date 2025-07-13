#ifndef AGENT_HPP
#define AGENT_HPP

#include "rclcpp/rclcpp.hpp"
#include "tf2_ros/buffer.h"
#include "tf2_ros/transform_listener.h"

#include "Cloud.hpp"
#include "Mcu.hpp"
#include "Navigation.hpp"
#include "RobotStatus.hpp"
#include "Sensors.hpp"

class Agent : public rclcpp::Node
{
  private:
    std::unique_ptr<Cloud> cloud;
    std::unique_ptr<Mcu> mcu;
    std::unique_ptr<Navigation> navigation;
    std::unique_ptr<Sensors> sensors;

    std::shared_ptr<McuSignals> mcuSignals;
    std::shared_ptr<SensorSignals> sensorSignals;

    rclcpp::TimerBase::SharedPtr cloudExchangeTimer;

    std::shared_ptr<tf2_ros::TransformListener> tfListener{nullptr};
    std::unique_ptr<tf2_ros::Buffer> tfBuffer;
    RobotClientsDof robotPosition;

    std::shared_ptr<RobotStatus> robotStatus;
    RobotClientsRobotCriticalStatus criticalStatus;

  public:
    Agent();
    void Initalise();
    void CloudExchangeReady();
};

#endif // AGENT_HPP