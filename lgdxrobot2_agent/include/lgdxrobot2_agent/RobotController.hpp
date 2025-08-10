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

#include "nav_msgs/msg/occupancy_grid.hpp"
#include "rclcpp/rclcpp.hpp"
#include "tf2_ros/buffer.h"
#include "tf2_ros/transform_listener.h"

class RobotController
{
  private:
    // ROS
    rclcpp::Logger logger_;
    rclcpp::TimerBase::SharedPtr autoTaskPublisherTimer;
    rclcpp::TimerBase::SharedPtr cloudExchangeTimer;
    rclcpp::TimerBase::SharedPtr robotDataPublisherTimer;
    rclcpp::Publisher<lgdxrobot2_agent::msg::AutoTask>::SharedPtr autoTaskPublisher;
    rclcpp::Publisher<lgdxrobot2_agent::msg::RobotData>::SharedPtr robotDataPublisher;
    rclcpp::Service<lgdxrobot2_agent::srv::AutoTaskNext>::SharedPtr autoTaskNextService;
    rclcpp::Service<lgdxrobot2_agent::srv::AutoTaskAbort>::SharedPtr autoTaskAbortService;
    rclcpp::Subscription<nav_msgs::msg::OccupancyGrid>::SharedPtr mapSubscription;
    std::shared_ptr<tf2_ros::TransformListener> tfListener{nullptr};
    std::unique_ptr<tf2_ros::Buffer> tfBuffer;
    std::shared_ptr<RobotControllerSignals> robotControllerSignals;

    // Robot Data
    bool isSlam = false;
    RobotStatus robotStatus;
    lgdxrobot2_agent::msg::RobotData robotData;
    std::shared_ptr<RobotClientsAutoTaskNavProgress> navProgress;
    std::vector<double> batteries = {0.0, 0.0};
    RobotClientsRobotCriticalStatus criticalStatus;
    // Robot Data: SLAM
    bool mapHasUpdated = false;
    bool overwriteGoal = false;
    RobotClientsMapData mapData;

    // Exchange
    RobotClientsData exchangeRobotData;
    RobotClientsNextToken exchangeNextToken;
    RobotClientsAbortToken exchangeAbortToken;
    RobotClientsSlamStatus exchangeSlamStatus = RobotClientsSlamStatus::SlamIdle;
    RobotClientsMapData exchangeMapData;

    // AutoTask
    lgdxrobot2_agent::msg::AutoTask currentTask;
    std::vector<RobotClientsPath> navigationPaths;
    std::size_t navigationProgress = 0;

    void UpdateExchange();
    void CloudExchange();

    void SlamExchange();
    void OnSlamMapUpdate(const nav_msgs::msg::OccupancyGrid &msg);

    void TryExitCriticalStatus();

  public:
    RobotController(rclcpp::Node::SharedPtr node,
      std::shared_ptr<RobotControllerSignals> robotControllerSignalsPtr,
      std::shared_ptr<RobotClientsAutoTaskNavProgress> navProgressPtr);

    void OnRobotDataReceived(const RobotData &rd);
    
    void OnConnectedCloud();
    void CloudAutoTaskNext();
    void CloudAutoTaskAbort(RobotClientsAbortReason reason);
    void OnNextCloudChange();
    void OnHandleClouldExchange(const RobotClientsResponse *response);

    void OnNavigationStart();
    void OnNavigationDone();
    void OnNavigationAborted();
    void OnNavigationStuck();
    void OnNavigationCleared();

    void OnNextSlamExchange();
    void OnHandleSlamExchange(const RobotClientsSlamCommands *response);
    
    void Shutdown();
};

#endif // ROBOT_HPP