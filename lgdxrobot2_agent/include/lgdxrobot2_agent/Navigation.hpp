#ifndef NAV_THROUGH_POSES_HPP
#define NAV_THROUGH_POSES_HPP

#include <vector>

#include "RobotStatus.hpp"
#include "Structs/NavigationSignals.hpp"

#include "geometry_msgs/msg/pose_stamped.hpp"
#include "rclcpp_action/rclcpp_action.hpp"
#include "rclcpp/rclcpp.hpp"
#include "nav2_msgs/action/navigate_through_poses.hpp"
#include "proto/RobotClientsService.grpc.pb.h"

class Navigation
{
  private:
    rclcpp::Logger logger_;

    std::shared_ptr<NavigationSignals> navigationSignals;
    RobotClientsAutoTaskNavProgress lastNavProgress;
    std::shared_ptr<RobotClientsAutoTaskNavProgress> navProgress;
    std::shared_ptr<RobotStatus> robotStatus;
    rclcpp_action::Client<nav2_msgs::action::NavigateThroughPoses>::SharedPtr navThroughPosesActionClient;

    void Response(const rclcpp_action::ClientGoalHandle<nav2_msgs::action::NavigateThroughPoses>::SharedPtr &goalHandle);
    void Feedback(rclcpp_action::ClientGoalHandle<nav2_msgs::action::NavigateThroughPoses>::SharedPtr, 
      const std::shared_ptr<const nav2_msgs::action::NavigateThroughPoses::Feedback> feedback);
    void Result(const rclcpp_action::ClientGoalHandle<nav2_msgs::action::NavigateThroughPoses>::WrappedResult &result);

  public:
    Navigation(rclcpp::Node::SharedPtr node,
      std::shared_ptr<NavigationSignals> navigationSignalsPtr,
      std::shared_ptr<RobotStatus> robotStatusPtr,
      std::shared_ptr<RobotClientsAutoTaskNavProgress> navProgressPtr);
    void Start(std::vector<geometry_msgs::msg::PoseStamped> &poses);
    void Abort();
};

#endif // NAV_THROUGH_POSES_HPP