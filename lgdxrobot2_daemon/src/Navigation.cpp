#include "lgdxrobot2_daemon/Navigation.hpp"

Navigation::Navigation(rclcpp_action::Client<nav2_msgs::action::NavigateThroughPoses>::SharedPtr actionClientPtr, 
  std::shared_ptr<RobotStatus> robotStatusPtr,
  std::function<void(const char *, int)> logCb,
  std::function<void(void)> abortCb,
  std::function<void(void)> nextCb)
{
  navThroughPosesActionClient = actionClientPtr;
  robotStatus = robotStatusPtr;
  log = logCb;
  abort = abortCb;
  next = nextCb;
}

void Navigation::response(const rclcpp_action::ClientGoalHandle<nav2_msgs::action::NavigateThroughPoses>::SharedPtr &goalHandle)
{
  if (!goalHandle)
  {
    log("navThroughPoses goal was rejected by server, the task will be aborted.", 3);
    abort();
  }
}

void Navigation::feedback(rclcpp_action::ClientGoalHandle<nav2_msgs::action::NavigateThroughPoses>::SharedPtr, 
  const std::shared_ptr<const nav2_msgs::action::NavigateThroughPoses::Feedback> feedback)
{
  lastNavProgress = navProgress;
  navProgress.set_eta(rclcpp::Duration(feedback->estimated_time_remaining).seconds());
  navProgress.set_recoveries(feedback->number_of_recoveries);
  navProgress.set_distanceremaining(feedback->distance_remaining);
  navProgress.set_waypointsremaining(feedback->number_of_poses_remaining);
  if (robotStatus->getRobotStatus() == RobotClientsRobotStatus::Running)
  {
    // Determine if the robot is stuck by
    // 1. Recoveries is increasing
    // 2. Eta is 0
    if (navProgress.recoveries() > lastNavProgress.recoveries() && 
        navProgress.eta() == 0)
    {
      robotStatus->navigationStuck();
      log("The robot is stuck.", 1);
    }
  }
  else if (robotStatus->getRobotStatus() == RobotClientsRobotStatus::Stuck)
  {
    // Determine if the robot is cleared by
    // 1. Recoveries is unchanged
    // 2. Eta is not 0 and decreasing
    // 3. distanceRemaining is decreasing
    if (navProgress.recoveries() == lastNavProgress.recoveries() &&
        navProgress.eta() < lastNavProgress.eta() &&
        navProgress.eta() > 0 &&
        navProgress.distanceremaining() < lastNavProgress.distanceremaining())
    {
      robotStatus->navigationCleared();
      log("The robot resumed navigation.", 1);
    }
  }
}

void Navigation::result(const rclcpp_action::ClientGoalHandle<nav2_msgs::action::NavigateThroughPoses>::WrappedResult &result)
{
  switch (result.code)
  {
    case rclcpp_action::ResultCode::SUCCEEDED:
      next();
      break;
    case rclcpp_action::ResultCode::ABORTED:
    case rclcpp_action::ResultCode::CANCELED:
    default:
      abort();
      return;
  }
}

void Navigation::navThroughPoses(std::vector<geometry_msgs::msg::PoseStamped> &poses)
{
  using namespace std::placeholders;

  if (!navThroughPosesActionClient->wait_for_action_server())
  {
    log("NAV2 stack is not ready.", 3);
    return;
  }

  auto goal = nav2_msgs::action::NavigateThroughPoses::Goal();
  goal.poses = poses;

  auto goalOption = rclcpp_action::Client<nav2_msgs::action::NavigateThroughPoses>::SendGoalOptions();
  goalOption.goal_response_callback = std::bind(&Navigation::response, this, _1);
  goalOption.feedback_callback = std::bind(&Navigation::feedback, this, _1, _2);
  goalOption.result_callback = std::bind(&Navigation::result, this, _1);
  navThroughPosesActionClient->async_send_goal(goal, goalOption);
}

RobotClientsAutoTaskNavProgress Navigation::getNavProgress()
{
  return navProgress;
}