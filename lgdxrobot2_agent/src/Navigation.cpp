#include "lgdxrobot2_agent/Navigation.hpp"

Navigation::Navigation(rclcpp::Node::SharedPtr node, 
    std::shared_ptr<NavigationSignals> navigationSignalsPtr,
    std::shared_ptr<RobotStatus> robotStatusPtr,
    std::shared_ptr<RobotClientsAutoTaskNavProgress> navProgressPtr
 ) : logger_(node->get_logger())
{
  navigationSignals = navigationSignalsPtr;
  robotStatus = robotStatusPtr;
  navProgress = navProgressPtr;

  navThroughPosesActionClient = rclcpp_action::create_client<nav2_msgs::action::NavigateThroughPoses>(
    node,
    "navigate_through_poses");
}

void Navigation::Response(const rclcpp_action::ClientGoalHandle<nav2_msgs::action::NavigateThroughPoses>::SharedPtr &goalHandle)
{
  if (!goalHandle)
  {
    RCLCPP_ERROR(logger_, "navThroughPoses goal was rejected by server, the task will be aborted.");
    navigationSignals->Abort(RobotClientsAbortReason::NavStack);
  }
}

void Navigation::Feedback(rclcpp_action::ClientGoalHandle<nav2_msgs::action::NavigateThroughPoses>::SharedPtr, 
  const std::shared_ptr<const nav2_msgs::action::NavigateThroughPoses::Feedback> feedback)
{
  lastNavProgress = *navProgress;
  navProgress->set_eta(rclcpp::Duration(feedback->estimated_time_remaining).seconds());
  navProgress->set_recoveries(feedback->number_of_recoveries);
  navProgress->set_distanceremaining(feedback->distance_remaining);
  navProgress->set_waypointsremaining(feedback->number_of_poses_remaining);
  if (robotStatus->GetStatus() == RobotClientsRobotStatus::Running)
  {
    // Determine if the robot is stuck by
    // 1. Recoveries is increasing
    // 2. Eta is 0
    if (navProgress->recoveries() > lastNavProgress.recoveries() && 
        navProgress->eta() == 0)
    {
      robotStatus->NavigationStuck();
      RCLCPP_INFO(logger_, "The robot is stuck.");
    }
  }
  else if (robotStatus->GetStatus() == RobotClientsRobotStatus::Stuck)
  {
    // Determine if the robot is cleared by
    // 1. Recoveries is unchanged
    // 2. Eta is not 0 and decreasing
    // 3. distanceRemaining is decreasing
    if (navProgress->recoveries() == lastNavProgress.recoveries() &&
        navProgress->eta() < lastNavProgress.eta() &&
        navProgress->eta() > 0 &&
        navProgress->distanceremaining() < lastNavProgress.distanceremaining())
    {
      robotStatus->NavigationCleared();
      RCLCPP_INFO(logger_, "The robot is cleared.");
    }
  }
}

void Navigation::Result(const rclcpp_action::ClientGoalHandle<nav2_msgs::action::NavigateThroughPoses>::WrappedResult &result)
{
  switch (result.code)
  {
    case rclcpp_action::ResultCode::SUCCEEDED:
      navigationSignals->NextNavigation();
      break;
    case rclcpp_action::ResultCode::ABORTED:
    case rclcpp_action::ResultCode::CANCELED:
    default:
      navigationSignals->Abort(RobotClientsAbortReason::NavStack);
      return;
  }
}

void Navigation::Start(std::vector<geometry_msgs::msg::PoseStamped> &poses)
{
  using namespace std::placeholders;

  if (!navThroughPosesActionClient->wait_for_action_server())
  {
    RCLCPP_ERROR(logger_, "navThroughPoses action server is not available.");
    return;
  }

  auto goal = nav2_msgs::action::NavigateThroughPoses::Goal();
  goal.poses = poses;

  auto goalOption = rclcpp_action::Client<nav2_msgs::action::NavigateThroughPoses>::SendGoalOptions();
  goalOption.goal_response_callback = std::bind(&Navigation::Response, this, _1);
  goalOption.feedback_callback = std::bind(&Navigation::Feedback, this, _1, _2);
  goalOption.result_callback = std::bind(&Navigation::Result, this, _1);
  navThroughPosesActionClient->async_send_goal(goal, goalOption);
}

void Navigation::Abort()
{
  if (navThroughPosesActionClient->wait_for_action_server())
  {
    // The NAV2 stack is running, cancel the goal
    auto cancelResult = navThroughPosesActionClient->async_cancel_all_goals(
      [this](auto response)
      {
        if (response)
        {
          RCLCPP_INFO(logger_, "Navigation aborted.");
        }
        else
        {
          RCLCPP_ERROR(logger_, "Navigation abort failed.");
        }
      }
    );
  }
  robotStatus->TaskAborted();
}
