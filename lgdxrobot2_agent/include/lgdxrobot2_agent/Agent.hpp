#ifndef AGENT_HPP
#define AGENT_HPP

#include "lgdxrobot2_agent/msg/auto_task.hpp"
#include "lgdxrobot2_agent/msg/robot_data.hpp"
#include "lgdxrobot2_agent/srv/auto_task_abort.hpp"
#include "lgdxrobot2_agent/srv/auto_task_next.hpp"
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

    std::shared_ptr<CloudSignals> cloudSignals;
    std::shared_ptr<McuSignals> mcuSignals;
    std::shared_ptr<SensorSignals> sensorSignals;

    rclcpp::TimerBase::SharedPtr cloudExchangeTimer;
    rclcpp::TimerBase::SharedPtr autoTaskPublisherTimer;
    rclcpp::Publisher<lgdxrobot2_agent::msg::RobotData>::SharedPtr robotDataPublisher;
    rclcpp::Publisher<lgdxrobot2_agent::msg::AutoTask>::SharedPtr autoTaskPublisher;
    rclcpp::Service<lgdxrobot2_agent::srv::AutoTaskNext>::SharedPtr autoTaskNextService;
    rclcpp::Service<lgdxrobot2_agent::srv::AutoTaskAbort>::SharedPtr autoTaskAbortService;
  
    std::shared_ptr<tf2_ros::TransformListener> tfListener{nullptr};
    std::unique_ptr<tf2_ros::Buffer> tfBuffer;

    lgdxrobot2_agent::msg::AutoTask currentTask;
    std::vector<RobotClientsPath> navigationPaths;
    std::size_t navigationProgress = 0;
    RobotClientsDof robotPosition;
    RobotClientsRobotCommands currentCommands;
    std::shared_ptr<RobotStatus> robotStatus;
    lgdxrobot2_agent::msg::RobotData robotData;
    std::vector<double> batteries = {0.0, 0.0};

    RobotClientsRobotCriticalStatus criticalStatus;

  private:
    void OnRobotDataReceived(const RobotData &rd);
    void CloudExchange();
    void OnCloudExchangeDone(const RobotClientsRespond *respond);
    void HandleNavigation();

    void CloudAutoTaskNext();
    void CloudAutoTaskAbort(RobotClientsAbortReason reason);

  public:
    Agent();
    void Initalise();
    void Shutdown();
};

#endif // AGENT_HPP