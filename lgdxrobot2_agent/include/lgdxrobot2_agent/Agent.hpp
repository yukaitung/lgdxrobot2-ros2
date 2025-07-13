#ifndef AGENT_HPP
#define AGENT_HPP

#include "lgdxrobot2_agent/msg/robot_data.hpp"
#include "rclcpp/rclcpp.hpp"
#include "tf2_ros/buffer.h"
#include "tf2_ros/transform_listener.h"

#include "Cloud.hpp"
#include "Mcu.hpp"
#include "Navigation.hpp"
#include "RobotController.hpp"
#include "RobotStatus.hpp"
#include "Sensors.hpp"

class Agent : public rclcpp::Node
{
  private:
    std::unique_ptr<Cloud> cloud;
    std::unique_ptr<Mcu> mcu;
    std::unique_ptr<Navigation> navigation;
    std::unique_ptr<RobotController> robotController;
    std::unique_ptr<Sensors> sensors;

    std::shared_ptr<CloudSignals> cloudSignals;
    std::shared_ptr<McuSignals> mcuSignals;
    std::shared_ptr<NavigationSignals> navigationSignals;
    std::shared_ptr<RobotControllerSignals> robotControllerSignals;
    std::shared_ptr<SensorSignals> sensorSignals;

    rclcpp::TimerBase::SharedPtr cloudExchangeTimer;
    rclcpp::TimerBase::SharedPtr robotDataPublisherTimer;
    rclcpp::Publisher<lgdxrobot2_agent::msg::RobotData>::SharedPtr robotDataPublisher;
    
    std::shared_ptr<tf2_ros::TransformListener> tfListener{nullptr};
    std::unique_ptr<tf2_ros::Buffer> tfBuffer;

    RobotClientsDof robotPosition;
    std::shared_ptr<RobotStatus> robotStatus;
    lgdxrobot2_agent::msg::RobotData robotData;
    std::vector<double> batteries = {0.0, 0.0};

  private:
    void OnRobotDataReceived(const RobotData &rd);
    void CloudExchange();

  public:
    Agent();
    void Initalise();
    void Shutdown();
};

#endif // AGENT_HPP