#include "lgdxrobot2_agent/RobotController.hpp"

#include "nav2_util/geometry_utils.hpp"

RobotController::RobotController(rclcpp::Node::SharedPtr node,
    std::shared_ptr<RobotControllerSignals> robotControllerSignalsPtr,
    std::shared_ptr<RobotStatus> robotStatusPtr,
    std::shared_ptr<RobotClientsAutoTaskNavProgress> navProgressPtr
  ) : logger_(node->get_logger())
{
  robotControllerSignals = robotControllerSignalsPtr;
  robotStatus = robotStatusPtr;
  navProgress = navProgressPtr;

  cloudExchangeTimer = node->create_wall_timer(std::chrono::milliseconds(500), 
    std::bind(&RobotController::CloudExchange, this));
  cloudExchangeTimer->cancel();
  
  tfBuffer = std::make_unique<tf2_ros::Buffer>(node->get_clock());
  tfListener = std::make_shared<tf2_ros::TransformListener>(*tfBuffer);

  // Topics
  autoTaskPublisher = node->create_publisher<lgdxrobot2_agent::msg::AutoTask>("/agent/auto_task", 
    rclcpp::SensorDataQoS().reliable());
  autoTaskPublisherTimer = node->create_wall_timer(std::chrono::milliseconds(100), 
    [this]()
    {
      autoTaskPublisher->publish(currentTask);
      robotData.robot_status = static_cast<int>(robotStatus->GetStatus());
      robotDataPublisher->publish(robotData);
    });
  robotDataPublisher = node->create_publisher<lgdxrobot2_agent::msg::RobotData>("/agent/robot_data", 
    rclcpp::SensorDataQoS().reliable());

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

void RobotController::CloudExchange()
{
  if (!cloudExchangeTimer->is_canceled())
    cloudExchangeTimer->cancel();

  try
  {
    geometry_msgs::msg::TransformStamped t;
    t = tfBuffer->lookupTransform("base_link", "map", tf2::TimePointZero);

    tf2::Quaternion q(
        t.transform.rotation.x,
        t.transform.rotation.y,
        t.transform.rotation.z,
        t.transform.rotation.w);
    tf2::Matrix3x3 m(q);
    double roll, pitch, yaw;
    m.getRPY(roll, pitch, yaw);

    robotPosition.set_x(-(t.transform.translation.x * cos(yaw) + t.transform.translation.y * sin(yaw)));
    robotPosition.set_y(-(-t.transform.translation.x * sin(yaw) + t.transform.translation.y * cos(yaw)));
    robotPosition.set_rotation(yaw);
  }
  catch (const tf2::TransformException &ex)
  {
  }
  batteries[0] = robotData.battery[0];
  batteries[1] = robotData.battery[1];

  RobotClientsAutoTaskNavProgress np = *navProgress;
  
  robotControllerSignals->CloudExchange(criticalStatus,
    batteries,
    robotPosition,
    np);
  // Don't reset the cloudExchangeTimer here
}

void RobotController::StatCloudExchange()
{
  cloudExchangeTimer->reset();
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

void RobotController::OnRobotDataReceived(const RobotData &rd)
{
  robotData.transform[0] = rd.transform[0];
  robotData.transform[1] = rd.transform[1];
  robotData.transform[2] = rd.transform[2];
  robotData.forward_kinematic[0] = rd.forwardKinematic[0];
  robotData.forward_kinematic[1] = rd.forwardKinematic[1];
  robotData.forward_kinematic[2] = rd.forwardKinematic[2];
  robotData.target_wheel_velocity[0] = rd.targetWheelVelocity[0];
  robotData.target_wheel_velocity[1] = rd.targetWheelVelocity[1];
  robotData.target_wheel_velocity[2] = rd.targetWheelVelocity[2];
  robotData.target_wheel_velocity[3] = rd.targetWheelVelocity[3];
  robotData.measured_wheel_velocity[0] = rd.measuredWheelVelocity[0];
  robotData.measured_wheel_velocity[1] = rd.measuredWheelVelocity[1];
  robotData.measured_wheel_velocity[2] = rd.measuredWheelVelocity[2];
  robotData.measured_wheel_velocity[3] = rd.measuredWheelVelocity[3];
  robotData.p_constant[0] = rd.pConstant[0];
  robotData.p_constant[1] = rd.pConstant[1];
  robotData.p_constant[2] = rd.pConstant[2];
  robotData.p_constant[3] = rd.pConstant[3];
  robotData.i_constant[0] = rd.iConstant[0];
  robotData.i_constant[1] = rd.iConstant[1];
  robotData.i_constant[2] = rd.iConstant[2];
  robotData.i_constant[3] = rd.iConstant[3];
  robotData.d_constant[0] = rd.dConstant[0];
  robotData.d_constant[1] = rd.dConstant[1];
  robotData.d_constant[2] = rd.dConstant[2];
  robotData.d_constant[3] = rd.dConstant[3];
  robotData.battery[0] = rd.battery[0];
  robotData.battery[1] = rd.battery[1];
  robotData.e_stop[0] = rd.eStop[0];
  robotData.e_stop[1] = rd.eStop[1];
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

void RobotController::Shutdown()
{
  if (cloudExchangeTimer != nullptr && !cloudExchangeTimer->is_canceled())
  {
    cloudExchangeTimer->cancel();
  }
}