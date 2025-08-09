#include "lgdxrobot2_agent/RobotController.hpp"

#include "nav2_util/geometry_utils.hpp"

RobotController::RobotController(rclcpp::Node::SharedPtr node,
    std::shared_ptr<RobotControllerSignals> robotControllerSignalsPtr,
    std::shared_ptr<RobotClientsAutoTaskNavProgress> navProgressPtr
  ) : logger_(node->get_logger())
{
  robotControllerSignals = robotControllerSignalsPtr;
  navProgress = navProgressPtr;

  bool cloudEnable = node->get_parameter("cloud_enable").as_bool();
  isSlam = node->get_parameter("cloud_slam_enable").as_bool();
  if (cloudEnable)
  {
    tfBuffer = std::make_unique<tf2_ros::Buffer>(node->get_clock());
    tfListener = std::make_shared<tf2_ros::TransformListener>(*tfBuffer);
  
    if (isSlam)
    {
      cloudExchangeTimer = node->create_wall_timer(std::chrono::milliseconds(200), 
        std::bind(&RobotController::SlamExchange, this));
      cloudExchangeTimer->cancel();

      mapSubscription = node->create_subscription<nav_msgs::msg::OccupancyGrid>("map", 
        rclcpp::SensorDataQoS().reliable(), 
        std::bind(&RobotController::OnSlamMapUpdate, this, std::placeholders::_1));
    }
    else
    {
      cloudExchangeTimer = node->create_wall_timer(std::chrono::milliseconds(500), 
        std::bind(&RobotController::CloudExchange, this));
      cloudExchangeTimer->cancel();

      // Topics
      autoTaskPublisher = node->create_publisher<lgdxrobot2_agent::msg::AutoTask>("agent/auto_task", 
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
  }
  
  // Topics
  robotDataPublisherTimer = node->create_wall_timer(std::chrono::milliseconds(100), 
    [this]()
    {
      robotData.robot_status = static_cast<int>(robotStatus.GetStatus());
      robotDataPublisher->publish(robotData);
    });
  robotDataPublisher = node->create_publisher<lgdxrobot2_agent::msg::RobotData>("agent/robot_data", 
    rclcpp::SensorDataQoS().reliable());
}

void RobotController::UpdateExchange()
{
  if (isSlam)
  {
    exchange.set_robotstatus(robotStatus.GetStatus() == RobotClientsRobotStatus::Critical ? RobotClientsRobotStatus::Critical : RobotClientsRobotStatus::Paused);
  }
  else
  {
    exchange.set_robotstatus(robotStatus.GetStatus());
  }
  exchange.mutable_criticalstatus()->CopyFrom(criticalStatus);
  auto exchangeBatteries = exchange.mutable_batteries();
  exchangeBatteries->Clear();
  exchangeBatteries->Reserve(batteries.size());
  for (size_t i = 0; i < batteries.size(); i++)
  {
    exchangeBatteries->AddAlreadyReserved(batteries[i]);
  }
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

    exchange.mutable_position()->set_x(-(t.transform.translation.x * cos(yaw) + t.transform.translation.y * sin(yaw)));
    exchange.mutable_position()->set_y(-(-t.transform.translation.x * sin(yaw) + t.transform.translation.y * cos(yaw)));
    exchange.mutable_position()->set_rotation(yaw);
  }
  catch (const tf2::TransformException &ex) 
  {
    exchange.mutable_position()->set_x(0.0);
    exchange.mutable_position()->set_y(0.0);
    exchange.mutable_position()->set_rotation(0.0);
  }
  exchange.mutable_navprogress()->CopyFrom(*navProgress);
}

void RobotController::CloudExchange()
{
  if (!cloudExchangeTimer->is_canceled())
    cloudExchangeTimer->cancel();

  UpdateExchange();

  robotControllerSignals->CloudExchange(exchange);
  // Don't reset the cloudExchangeTimer here
}

void RobotController::SlamExchange()
{
  if (!cloudExchangeTimer->is_canceled())
  cloudExchangeTimer->cancel();

  UpdateExchange();

  slamExchange.clear_mapdata();
  slamExchange.set_status(slamStatus);
  slamExchange.mutable_exchange()->CopyFrom(exchange);
  if (mapHasUpdated)
  {
    slamExchange.mutable_mapdata()->CopyFrom(mapData);
    mapHasUpdated = false;
  }
  robotControllerSignals->SlamExchange(slamExchange);
  // Don't reset the slamExchangeTimer here
}

void RobotController::OnSlamMapUpdate(const nav_msgs::msg::OccupancyGrid &msg)
{
  // Compare the map with the current map
  int incomingSize = (int)msg.data.size();
  if (incomingSize == mapData.data_size())
  {
    bool same = true;
    for (size_t i = 0; i < msg.data.size(); i++)
    {
      if ((int32_t)msg.data[i] != mapData.data(i))
      {
        same = false;
        break;
      }
    }
    if (same)
    {
      return;
    }
  }

  // Update the map
  mapData.set_resolution(msg.info.resolution);
  mapData.set_width(msg.info.width);
  mapData.set_height(msg.info.height);
  // Origin
  auto origin = mapData.mutable_origin();
  origin->set_x(msg.info.origin.position.x);
  origin->set_y(msg.info.origin.position.y);
  tf2::Quaternion q(
    msg.info.origin.orientation.x,
    msg.info.origin.orientation.y,
    msg.info.origin.orientation.z,
    msg.info.origin.orientation.w);
  tf2::Matrix3x3 m(q);
  double roll, pitch, yaw;
  m.getRPY(roll, pitch, yaw);
  origin->set_rotation(yaw);
  // Map Data
  auto md = mapData.mutable_data();
  md->Clear();
  md->Reserve(incomingSize);
  for (int i = 0; i < incomingSize; i++)
  {
    md->AddAlreadyReserved(msg.data[i]);
  }
  mapHasUpdated = true;
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

void RobotController::OnConnectedCloud()
{
  robotStatus.ConnnectedCloud();
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
    //robotStatus.TaskAborting();
    robotStatus.TaskCompleted();
    RCLCPP_INFO(logger_, "AutoTask will be aborted.");
    RobotClientsAbortToken token;
    token.set_taskid(currentTask.task_id);
    token.set_nexttoken(currentTask.next_token);
    token.set_abortreason(reason);
    robotControllerSignals->AutoTaskAbort(token);
  }
}

void RobotController::OnNextCloudChange()
{
  cloudExchangeTimer->reset();
}

void RobotController::OnHandleClouldExchange(const RobotClientsRespond *respond)
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
      robotStatus.TaskCompleted();
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
        OnNavigationStart();
      }
      robotStatus.TaskAssigned();
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
      robotStatus.EnterCritical();
    }
    else if (commands.softwareemergencystop() == false && currentCommands.softwareemergencystop() == true)
    {
      robotStatus.ExitCritical();
    }
    criticalStatus.set_softwareemergencystop(commands.softwareemergencystop());
    currentCommands.set_softwareemergencystop(commands.softwareemergencystop());

    // Pause Task Assigement
    if (commands.pausetaskassigement() == true && currentCommands.pausetaskassigement() == false)
    {
      robotStatus.PauseTaskAssigement();
    }
    else if (commands.pausetaskassigement() == false && currentCommands.pausetaskassigement() == true)
    {
      robotStatus.ResumeTaskAssigement();
    }
    currentCommands.set_pausetaskassigement(commands.pausetaskassigement());
  }
}

void RobotController::OnNavigationStart()
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

void RobotController::OnNavigationDone()
{
  if (isSlam)
  {
    slamStatus = RobotClientsSlamStatus::SlamSuccess;
  }
  else
  {
    OnNavigationStart();
  }
}

void RobotController::OnNavigationStuck()
{
  if (!isSlam)
    robotStatus.NavigationStuck();
}

void RobotController::OnNavigationCleared()
{
  if (!isSlam)
    robotStatus.NavigationCleared();
}

void RobotController::OnNavigationAborted()
{
  if (isSlam)
  {
    if (overwriteGoal)
    {
      // Not showing aborted because the goal was overwritten
      overwriteGoal = false;
    }
    else
    {
      slamStatus = RobotClientsSlamStatus::SlamAborted;
    }
  }
  else
  {
    CloudAutoTaskAbort(RobotClientsAbortReason::NavStack);
  }
}

void RobotController::OnNextSlamExchange()
{
  cloudExchangeTimer->reset();
}

void RobotController::OnHandleSlamExchange(const RobotClientsSlamCommands *respond)
{
  if (respond->has_setgoal())
  {
    if (slamStatus == RobotClientsSlamStatus::SlamRunning)
    {
      overwriteGoal = true;
    }
    double x = respond->setgoal().x();
    double y = respond->setgoal().y();
    double rotation = respond->setgoal().rotation();
    RCLCPP_INFO(logger_, "A new goal is set: %fm, %fm, %frad", x, y, rotation);
    std::vector<geometry_msgs::msg::PoseStamped> poses;
    auto pose = geometry_msgs::msg::PoseStamped();
    pose.header.stamp = rclcpp::Clock().now();
    pose.header.frame_id = "map";
    pose.pose.position.z = 0.0;
    pose.pose.position.x = x;
    pose.pose.position.y = y;
    pose.pose.orientation = nav2_util::geometry_utils::orientationAroundZAxis(rotation);
    poses.push_back(pose);
    robotControllerSignals->NavigationStart(poses);
    slamStatus = RobotClientsSlamStatus::SlamRunning;
  }
  if (respond->has_abortgoal())
  {
    RCLCPP_INFO(logger_, "Aborting the current goal");
    robotControllerSignals->NavigationAbort();
  }
  if (respond->has_softwareemergencystopenable() && respond->softwareemergencystopenable() == true)
  {
    RCLCPP_INFO(logger_, "Enabling software emergency stop");
    robotStatus.EnterCritical();
  }
  if (respond->has_softwareemergencystopdisable() && respond->softwareemergencystopdisable() == true)
  {
    RCLCPP_INFO(logger_, "Disabling software emergency stop");
    robotStatus.ExitCritical();
  }
  if (respond->has_savemap() && respond->savemap() == true)
  {
    RCLCPP_INFO(logger_, "Saving the map");
    robotControllerSignals->SaveMap();
  }
  if (respond->has_refreshmap() && respond->refreshmap() == true)
  {
    RCLCPP_INFO(logger_, "Refreshing the map");
    mapHasUpdated = true;
  }
  if (respond->has_abortslam() && respond->abortslam() == true)
  {
    RCLCPP_INFO(logger_, "Aborting the current SLAM");
    robotControllerSignals->NavigationAbort();
    robotControllerSignals->Shutdown();
  }
  if (respond->has_completeslam() && respond->completeslam() == true)
  {
    robotControllerSignals->NavigationAbort();
    RCLCPP_INFO(logger_, "Completing the current SLAM and saving the map with 5 seconds blocking");
    robotControllerSignals->SaveMap();
    std::this_thread::sleep_for(std::chrono::seconds(5));
    robotControllerSignals->Shutdown();
  }
}

void RobotController::Shutdown()
{
  if (cloudExchangeTimer != nullptr && !cloudExchangeTimer->is_canceled())
  {
    cloudExchangeTimer->cancel();
  }
}