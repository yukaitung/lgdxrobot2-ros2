#include "lgdxrobot2_agent/RobotController.hpp"

#include "nav2_util/geometry_utils.hpp"

RobotController::RobotController(rclcpp::Node::SharedPtr node,
    std::shared_ptr<RobotControllerSignals> robotControllerSignalsPtr,
    std::shared_ptr<RobotStatus> robotStatusPtr
  ) : logger_(node->get_logger())
{
  robotStatus = robotStatusPtr;

  // Topics
  autoTaskPublisher = node->create_publisher<lgdxrobot2_agent::msg::AutoTask>("/agent/auto_task", 
    rclcpp::SensorDataQoS().reliable());
  autoTaskPublisherTimer = node->create_wall_timer(std::chrono::milliseconds(100), 
    [this]()
    {
      autoTaskPublisher->publish(currentTask);
    });

  // Services
  autoTaskNextService = node->create_service<lgdxrobot2_agent::srv::AutoTaskNext>("auto_task_next",
    [this](const std::shared_ptr<lgdxrobot2_agent::srv::AutoTaskNext::Request> request,
      std::shared_ptr<lgdxrobot2_agent::srv::AutoTaskNext::Response> response) 
    {
      if (!currentTask.next_token.empty() && 
          request->task_id == currentTask.task_id &&
          request->next_token == currentTask.next_token)
      {
        CloudAutoTaskNext();
        response->success = true;
      }
      else
      {
        response->success = false;
      }
    });
  autoTaskAbortService = node->create_service<lgdxrobot2_agent::srv::AutoTaskAbort>("auto_task_abort",
    [this](const std::shared_ptr<lgdxrobot2_agent::srv::AutoTaskAbort::Request> request,
      std::shared_ptr<lgdxrobot2_agent::srv::AutoTaskAbort::Response> response)
    {
      if (!currentTask.next_token.empty() && 
          request->task_id == currentTask.task_id &&
          request->next_token == currentTask.next_token)
      {
        CloudAutoTaskAbort(RobotClientsAbortReason::Robot);
        response->success = true;
      }
      else
      {
        response->success = false;
      }
    });
}

void RobotController::OnCloudExchangeDone(const RobotClientsRespond *respond)
{
  // Handle AutoTask
  if (respond->has_task())
  {
    RobotClientsAutoTask task = respond->task();
    currentTask.task_id = task.taskid();
    currentTask.task_name = task.taskname();
    currentTask.task_progress_id = task.taskprogressid();
    currentTask.task_progress_name = task.taskprogressname();
    currentTask.next_token = task.nexttoken();
    if (currentTask.task_progress_id == 3)
    {
      RCLCPP_INFO(logger_, "AutoTask Id: %d completed.", task.taskid());
      robotStatus->TaskCompleted();
    }
    else if (currentTask.task_progress_id == 4)
    {
      RCLCPP_INFO(logger_, "AutoTask Id: %d aborted.", task.taskid());
      robotControllerSignals->NavigationAbort();
    }
    else
    {
      RCLCPP_INFO(logger_, "Received AutoTask Id: %d, Progress: %d", task.taskid(), task.taskprogressid());
      if (task.paths_size())
      {
        RCLCPP_INFO(logger_, "This task has %d waypoint(s).", task.paths_size());
        navigationPaths.clear();
        navigationPaths.assign(task.paths().begin(), task.paths().end());
        navigationProgress = 0;
        NavigationStart();
      }
      robotStatus->TaskAssigned();
    }
  }

  // Handle Robot Command
  if (respond->has_commands())
  {
    auto commands = respond->commands();
    // Abort Task
    if (commands.aborttask() == true && currentCommands.aborttask() == false)
    {
      CloudAutoTaskAbort(RobotClientsAbortReason::UserApi);
    }
    currentCommands.set_aborttask(commands.aborttask());

    // Emergency Stop
    if (commands.softwareemergencystop() == true && currentCommands.softwareemergencystop() == false)
    {
      robotStatus->EnterCritical();
    }
    else if (commands.softwareemergencystop() == false && currentCommands.softwareemergencystop() == true)
    {
      robotStatus->ExitCritical();
    }
    criticalStatus.set_softwareemergencystop(commands.softwareemergencystop());
    currentCommands.set_softwareemergencystop(commands.softwareemergencystop());

    // Pause Task Assigement
    if (commands.pausetaskassigement() == true && currentCommands.pausetaskassigement() == false)
    {
      robotStatus->PauseTaskAssigement();
    }
    else if (commands.pausetaskassigement() == false && currentCommands.pausetaskassigement() == true)
    {
      robotStatus->ResumeTaskAssigement();
    }
    currentCommands.set_pausetaskassigement(commands.pausetaskassigement());
  }
}

void RobotController::NavigationStart()
{
  if (navigationProgress < navigationPaths.size())
  {
    std::vector<geometry_msgs::msg::PoseStamped> poses;
    auto pose = geometry_msgs::msg::PoseStamped();
    pose.header.stamp = rclcpp::Clock().now();
    pose.header.frame_id = "map";
    pose.pose.position.z = 0.0;
    for (int i = 0; i < navigationPaths.at(navigationProgress).waypoints_size(); i++)
    {
      const RobotClientsDof waypoint = navigationPaths.at(navigationProgress).waypoints(i);
      pose.pose.position.x = waypoint.x();
      pose.pose.position.y = waypoint.y();
      pose.pose.orientation = nav2_util::geometry_utils::orientationAroundZAxis(waypoint.rotation());
      poses.push_back(pose);
    }
    robotControllerSignals->NavigationStart(poses);
    navigationProgress++;
  }
  else
  {
    CloudAutoTaskNext();
  }
}

void RobotController::CloudAutoTaskNext()
{
  if (!currentTask.next_token.empty())
  {
    RCLCPP_INFO(logger_, "AutoTask advances to next progress.");
    RobotClientsNextToken token;
    token.set_taskid(currentTask.task_id);
    token.set_nexttoken(currentTask.next_token);
    robotControllerSignals->AutoTaskNext(token);
  }
}

void RobotController::CloudAutoTaskAbort(RobotClientsAbortReason reason)
{
  if (!currentTask.next_token.empty())
  {
    robotStatus->TaskAborting();
    RCLCPP_INFO(logger_, "AutoTask will be aborted.");
    RobotClientsAbortToken token;
    token.set_taskid(currentTask.task_id);
    token.set_nexttoken(currentTask.next_token);
    token.set_abortreason(reason);
    robotControllerSignals->AutoTaskAbort(token);
  }
}

const RobotClientsRobotCriticalStatus RobotController::GetCriticalStatus()
{
  return criticalStatus;
}