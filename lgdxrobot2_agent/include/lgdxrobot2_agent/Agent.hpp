#ifndef AGENT_HPP
#define AGENT_HPP

#include "rclcpp/rclcpp.hpp"

#include "Cloud.hpp"
#include "Mcu.hpp"
#include "Navigation.hpp"
#include "RobotController.hpp"
#include "RobotStatus.hpp"
#include "Sensors.hpp"
#include "SlamController.hpp"

class Agent : public rclcpp::Node
{
  private:
    std::unique_ptr<Cloud> cloud;
    std::unique_ptr<Mcu> mcu;
    std::unique_ptr<Navigation> navigation;
    std::unique_ptr<RobotController> robotController;
    std::unique_ptr<Sensors> sensors;
    std::unique_ptr<SlamController> slamController;

    std::shared_ptr<CloudSignals> cloudSignals;
    std::shared_ptr<McuSignals> mcuSignals;
    std::shared_ptr<NavigationSignals> navigationSignals;
    std::shared_ptr<RobotControllerSignals> robotControllerSignals;
    std::shared_ptr<SensorSignals> sensorSignals;
    std::shared_ptr<SlamControllerSignals> slamControllerSignals;

    std::shared_ptr<RobotStatus> robotStatus;
    std::shared_ptr<RobotClientsAutoTaskNavProgress> navProgress;

  public:
    Agent();
    void Initalise();
    void Shutdown();
};

#endif // AGENT_HPP