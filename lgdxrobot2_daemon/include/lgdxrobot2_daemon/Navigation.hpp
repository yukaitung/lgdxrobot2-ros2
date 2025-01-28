#ifndef NAV_THROUGH_POSES_HPP
#define NAV_THROUGH_POSES_HPP

#include <vector>

#include "RobotStatus.hpp"

#include "geometry_msgs/msg/pose_stamped.hpp"
#include "rclcpp_action/rclcpp_action.hpp"
#include "rclcpp/rclcpp.hpp"
#include "nav2_msgs/action/navigate_through_poses.hpp"
#include "proto/RobotClientsService.grpc.pb.h"

class Navigation
{
  private:
  RobotClientsAutoTaskNavProgress lastNavProgress;
  RobotClientsAutoTaskNavProgress navProgress;
  std::shared_ptr<RobotStatus> robotStatus;
  rclcpp_action::Client<nav2_msgs::action::NavigateThroughPoses>::SharedPtr navThroughPosesActionClient;

  std::function<void(const char *, int)> log;
  std::function<void(void)> abort;
  std::function<void(void)> next;

  void response(const rclcpp_action::ClientGoalHandle<nav2_msgs::action::NavigateThroughPoses>::SharedPtr &goalHandle);
  void feedback(rclcpp_action::ClientGoalHandle<nav2_msgs::action::NavigateThroughPoses>::SharedPtr, 
    const std::shared_ptr<const nav2_msgs::action::NavigateThroughPoses::Feedback> feedback);
  void result(const rclcpp_action::ClientGoalHandle<nav2_msgs::action::NavigateThroughPoses>::WrappedResult &result);

  public:
    Navigation(rclcpp_action::Client<nav2_msgs::action::NavigateThroughPoses>::SharedPtr actionClientPtr, 
      std::shared_ptr<RobotStatus> robotStatusPtr,
      std::function<void(const char *, int)> logCb,
      std::function<void(void)> abortCb,
      std::function<void(void)> nextCb);
    void navThroughPoses(std::vector<geometry_msgs::msg::PoseStamped> &poses);
    RobotClientsAutoTaskNavProgress getNavProgress();
};

#endif // NAV_THROUGH_POSES_HPP