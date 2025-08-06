#ifndef SLAMCONTROLLER_HPP
#define SLAMCONTROLLER_HPP

#include "lgdxrobot2_agent/msg/robot_data.hpp"
#include "proto/RobotClientsService.grpc.pb.h"
#include "RobotStatus.hpp"
#include "Structs/RobotData.hpp"
#include "Structs/SlamControllerSignals.hpp"

#include "nav_msgs/msg/occupancy_grid.hpp"
#include "rclcpp/rclcpp.hpp"
#include "tf2_ros/buffer.h"
#include "tf2_ros/transform_listener.h"

class SlamController
{
  private:
    rclcpp::Logger logger_;

    rclcpp::TimerBase::SharedPtr robotDataPublisherTimer;
    rclcpp::TimerBase::SharedPtr slamExchangeTimer;

    rclcpp::Publisher<lgdxrobot2_agent::msg::RobotData>::SharedPtr robotDataPublisher;

    rclcpp::Subscription<nav_msgs::msg::OccupancyGrid>::SharedPtr mapSubscription;
    
    std::shared_ptr<tf2_ros::TransformListener> tfListener{nullptr};
    std::unique_ptr<tf2_ros::Buffer> tfBuffer;

    std::shared_ptr<SlamControllerSignals> slamControllerSignals;

    // Robot Data
    bool mapHasUpdated = false;
    std::shared_ptr<RobotStatus> robotStatus;
    std::shared_ptr<RobotClientsAutoTaskNavProgress> navProgress;

    lgdxrobot2_agent::msg::RobotData robotData;
    RobotClientsExchange exchange;
    RobotClientsMapData mapData;
    RobotClientsSlamStatus status = RobotClientsSlamStatus::SlamIdle;
    RobotClientsRobotCriticalStatus criticalStatus;
    std::vector<double> batteries = {0.0, 0.0};

    void MapCallback(const nav_msgs::msg::OccupancyGrid &msg);
    void UpdateExchange();
    void SlamExchange();

  public:
    SlamController(rclcpp::Node::SharedPtr node,
      std::shared_ptr<SlamControllerSignals> slamControllerSignalsPtr,
      std::shared_ptr<RobotStatus> robotStatusPtr,
      std::shared_ptr<RobotClientsAutoTaskNavProgress> navProgressPtr);
    void StatSlamExchange();
    void OnSlamExchangeDone(const RobotClientsSlamCommands *respond);
    void OnNavigationDone();
    void OnNavigationAborted(RobotClientsAbortReason reason);
    void OnRobotDataReceived(const RobotData &rd);

    void Shutdown();
};

#endif // SLAMCONTROLLER_HPP