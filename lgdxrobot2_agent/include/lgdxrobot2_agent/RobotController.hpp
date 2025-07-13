#ifndef ROBOT_HPP
#define ROBOT_HPP

#include "lgdxrobot2_agent/msg/auto_task.hpp"
#include "lgdxrobot2_agent/srv/auto_task_abort.hpp"
#include "lgdxrobot2_agent/srv/auto_task_next.hpp"
#include "proto/RobotClientsService.grpc.pb.h"
#include "RobotStatus.hpp"
#include "Structs/RobotControllerSignals.hpp"

#include "rclcpp/rclcpp.hpp"

class RobotController
{
  private:
    rclcpp::Logger logger_;

    rclcpp::TimerBase::SharedPtr autoTaskPublisherTimer;
    rclcpp::Publisher<lgdxrobot2_agent::msg::AutoTask>::SharedPtr autoTaskPublisher;
    rclcpp::Service<lgdxrobot2_agent::srv::AutoTaskNext>::SharedPtr autoTaskNextService;
    rclcpp::Service<lgdxrobot2_agent::srv::AutoTaskAbort>::SharedPtr autoTaskAbortService;

    std::shared_ptr<RobotControllerSignals> robotControllerSignals;
    lgdxrobot2_agent::msg::AutoTask currentTask;
    std::vector<RobotClientsPath> navigationPaths;
    std::size_t navigationProgress = 0;
    std::shared_ptr<RobotStatus> robotStatus;
    RobotClientsRobotCommands currentCommands;
    RobotClientsRobotCriticalStatus criticalStatus;


  public:
    RobotController(rclcpp::Node::SharedPtr node,
      std::shared_ptr<RobotControllerSignals> robotControllerSignalsPtr,
      std::shared_ptr<RobotStatus> robotStatusPtr);
    void OnCloudExchangeDone(const RobotClientsRespond *respond);
    
    void NavigationStart();
    void CloudAutoTaskNext();
    void CloudAutoTaskAbort(RobotClientsAbortReason reason);

    const RobotClientsRobotCriticalStatus GetCriticalStatus();
};

#endif // ROBOT_HPP