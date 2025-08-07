#ifndef ROBOT_HPP
#define ROBOT_HPP

#include "lgdxrobot2_agent/msg/auto_task.hpp"
#include "lgdxrobot2_agent/msg/robot_data.hpp"
#include "lgdxrobot2_agent/srv/auto_task_abort.hpp"
#include "lgdxrobot2_agent/srv/auto_task_next.hpp"
#include "proto/RobotClientsService.grpc.pb.h"
#include "RobotStatus.hpp"
#include "Structs/RobotControllerSignals.hpp"
#include "Structs/RobotData.hpp"

#include "rclcpp/rclcpp.hpp"
#include "tf2_ros/buffer.h"
#include "tf2_ros/transform_listener.h"

class RobotController
{
  private:
    rclcpp::Logger logger_;

    rclcpp::TimerBase::SharedPtr autoTaskPublisherTimer;
    rclcpp::TimerBase::SharedPtr cloudExchangeTimer;
    rclcpp::TimerBase::SharedPtr robotDataPublisherTimer;

    rclcpp::Publisher<lgdxrobot2_agent::msg::AutoTask>::SharedPtr autoTaskPublisher;
    rclcpp::Publisher<lgdxrobot2_agent::msg::RobotData>::SharedPtr robotDataPublisher;

    rclcpp::Service<lgdxrobot2_agent::srv::AutoTaskNext>::SharedPtr autoTaskNextService;
    rclcpp::Service<lgdxrobot2_agent::srv::AutoTaskAbort>::SharedPtr autoTaskAbortService;
    
    std::shared_ptr<tf2_ros::TransformListener> tfListener{nullptr};
    std::unique_ptr<tf2_ros::Buffer> tfBuffer;

    std::shared_ptr<RobotControllerSignals> robotControllerSignals;

    // Robot Data
    std::shared_ptr<RobotStatus> robotStatus;
    lgdxrobot2_agent::msg::RobotData robotData;
    RobotClientsRobotCommands currentCommands;
    std::shared_ptr<RobotClientsAutoTaskNavProgress> navProgress;

    // Exchange
    RobotClientsExchange exchange;
    RobotClientsDof robotPosition;
    std::vector<double> batteries = {0.0, 0.0};
    RobotClientsRobotCriticalStatus criticalStatus;

    // AutoTask
    lgdxrobot2_agent::msg::AutoTask currentTask;
    std::vector<RobotClientsPath> navigationPaths;
    std::size_t navigationProgress = 0;

    void UpdateExchange();
    void CloudExchange();

  public:
    RobotController(rclcpp::Node::SharedPtr node,
      std::shared_ptr<RobotControllerSignals> robotControllerSignalsPtr,
      std::shared_ptr<RobotStatus> robotStatusPtr,
      std::shared_ptr<RobotClientsAutoTaskNavProgress> navProgressPtr);

    void OnRobotDataReceived(const RobotData &rd);
    
    void CloudAutoTaskNext();
    void CloudAutoTaskAbort(RobotClientsAbortReason reason);
    void OnNextCloudChange();
    void OnHandleClouldExchange(const RobotClientsRespond *respond);

    void OnNavigationStart();
    void OnNavigationStuck();
    void OnNavigationCleared();
    
    void Shutdown();
};

#endif // ROBOT_HPP