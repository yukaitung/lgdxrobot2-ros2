#include "lgdxrobot2_agent/Agent.hpp"

#include "nav2_util/geometry_utils.hpp"

Agent::Agent() : Node("lgdxrobot2_agent_node")
{}

void Agent::Initalise()
{
  auto cloudEnableParam = rcl_interfaces::msg::ParameterDescriptor{};
  cloudEnableParam.description = "Enable LGDXRobot Cloud.";
  this->declare_parameter("cloud_enable", false, cloudEnableParam);
  auto mcuEnableParam = rcl_interfaces::msg::ParameterDescriptor{};
  mcuEnableParam.description = "Enable LGDXRobot2 MCU.";
  this->declare_parameter("mcu_enable", false, mcuEnableParam);

  // Signals
  cloudSignals = std::make_shared<CloudSignals>();
  mcuSignals = std::make_shared<McuSignals>();
  sensorSignals = std::make_shared<SensorSignals>();

  // Cloud
  bool cloudEnable = this->get_parameter("cloud_enable").as_bool();
  if (cloudEnable)
  {
    robotStatus = std::make_shared<RobotStatus>();
    cloud = std::make_unique<Cloud>(shared_from_this(), cloudSignals, robotStatus);
    navigation = std::make_unique<Navigation>(shared_from_this(), robotStatus);

    cloudExchangeTimer = this->create_wall_timer(std::chrono::milliseconds(500), 
      std::bind(&Agent::CloudExchange, this));
    cloudExchangeTimer->cancel();

    autoTaskPublisher = this->create_publisher<lgdxrobot2_agent::msg::AutoTask>("/agent/auto_task", 
      rclcpp::SensorDataQoS().reliable());
    crtitcalStatusPublisher = this->create_publisher<std_msgs::msg::Bool>("/agent/crtitcal_status", 
      rclcpp::SensorDataQoS().reliable());
    autoTaskPublisherTimer = this->create_wall_timer(std::chrono::milliseconds(100), 
      [this]()
      {
        autoTaskPublisher->publish(currentTask);
        /*
        std_msgs::msg::Bool criticalStatus;
        criticalStatus.data = robotStatus->getRobotStatus() == RobotClientsRobotStatus::Critical;
        crtitcalStatusPublisher->publish(criticalStatus);*/
      });

    autoTaskNextService = this->create_service<lgdxrobot2_agent::srv::AutoTaskNext>("auto_task_next",
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
    autoTaskAbortService = this->create_service<lgdxrobot2_agent::srv::AutoTaskAbort>("auto_task_abort",
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

    tfBuffer = std::make_unique<tf2_ros::Buffer>(this->get_clock());
    tfListener = std::make_shared<tf2_ros::TransformListener>(*tfBuffer);
  }

  // MCU
  bool mcuEnable = this->get_parameter("mcu_enable").as_bool();
  if (mcuEnable)
  {
    mcu = std::make_unique<Mcu>(shared_from_this(), mcuSignals);
    sensors = std::make_unique<Sensors>(shared_from_this(), sensorSignals);
  }

  // Signals
  if (cloudEnable)
  {
    cloudSignals->NextExchange.connect([this](){
      // Initalise the timer for sending exchange
      cloudExchangeTimer->reset();
    });
    cloudSignals->HandleExchange.connect(boost::bind(&Agent::CloudExchange, this));
  }
  if (mcuEnable)
  {
    mcuSignals->UpdateRobotData.connect(boost::bind(&Sensors::PublishOdom, sensors.get(), boost::placeholders::_1));
    if (cloudEnable)
    {
      // Start connection if MCU serial number obtained
      mcuSignals->UpdateSerialNumber.connect(boost::bind(&Cloud::Greet, cloud.get(), boost::placeholders::_1));
    }
    sensorSignals->SetEstop.connect(boost::bind(&Mcu::SetEstop, mcu.get(), boost::placeholders::_1));
    sensorSignals->SetInverseKinematics.connect(boost::bind(&Mcu::SetInverseKinematics, mcu.get(), 
      boost::placeholders::_1, boost::placeholders::_2, boost::placeholders::_3));
    sensorSignals->SetExternalImu.connect(boost::bind(&Mcu::SetExternalImu, mcu.get(), 
      boost::placeholders::_1, boost::placeholders::_2, boost::placeholders::_3, boost::placeholders::_4));
  }
}

void Agent::CloudExchange()
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
  std::vector<double> batteries = {12.00, 12.00};

  RobotClientsAutoTaskNavProgress navProgress = navigation->GetNavProgress();
  
  cloud->Exchange(criticalStatus,
    batteries,
    robotPosition,
    navProgress);
  // Don't reset the cloudExchangeTimer here
}

void Agent::OnCloudExchangeDone(const RobotClientsRespond *respond)
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
      RCLCPP_INFO(this->get_logger(), "AutoTask Id: %d completed.", task.taskid());
      robotStatus->TaskCompleted();
    }
    else if (currentTask.task_progress_id == 4)
    {
      RCLCPP_INFO(this->get_logger(), "AutoTask Id: %d aborted.", task.taskid());
      navigation->Abort();
    }
    else
    {
      RCLCPP_INFO(this->get_logger(), "Received AutoTask Id: %d, Progress: %d", task.taskid(), task.taskprogressid());
      if (task.paths_size())
      {
        RCLCPP_INFO(this->get_logger(), "This task has %d waypoint(s).", task.paths_size());
        navigationPaths.clear();
        navigationPaths.assign(task.paths().begin(), task.paths().end());
        navigationProgress = 0;
        HandleNavigation();
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

void Agent::HandleNavigation()
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
    navigation->Start(poses);
    navigationProgress++;
  }
  else
  {
    CloudAutoTaskNext();
  }
}

void Agent::CloudAutoTaskNext()
{
  if (!currentTask.next_token.empty())
  {
    RCLCPP_INFO(this->get_logger(), "AutoTask advances to next progress.");
    RobotClientsNextToken token;
    token.set_taskid(currentTask.task_id);
    token.set_nexttoken(currentTask.next_token);
    cloud->AutoTaskNext(token);
  }
}

void Agent::CloudAutoTaskAbort(RobotClientsAbortReason reason)
{
  if (!currentTask.next_token.empty())
  {
    robotStatus->TaskAborting();
    RCLCPP_INFO(this->get_logger(), "AutoTask will be aborted.");
    RobotClientsAbortToken token;
    token.set_taskid(currentTask.task_id);
    token.set_nexttoken(currentTask.next_token);
    token.set_abortreason(reason);
    cloud->AutoTaskAbort(token);
  }
}

void Agent::Shutdown()
{
  if (cloudExchangeTimer != nullptr && !cloudExchangeTimer->is_canceled())
  {
    cloudExchangeTimer->cancel();
  }
  
  if (cloud != nullptr)
  {
    cloud->Shutdown();
  }
}