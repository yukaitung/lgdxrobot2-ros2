#include <random>
#include <vector>

#include "lgdxrobot2_daemon/DaemonNode.hpp"

#include "geometry_msgs/msg/transform_stamped.hpp"
#include "tf2_geometry_msgs/tf2_geometry_msgs.hpp"
#include "nav2_util/geometry_utils.hpp"

DaemonNode::DaemonNode() : Node("lgdxrobot2_daemon_node")
{
  /*
   * Parameters
   */
  // Cloud
  auto cloudEnableParam = rcl_interfaces::msg::ParameterDescriptor{};
  cloudEnableParam.description = "Enable LGDXRobot2 Cloud.";
  this->declare_parameter("cloud_enable", false, cloudEnableParam);
  auto cloudAddressParam = rcl_interfaces::msg::ParameterDescriptor{};
  cloudAddressParam.description = "Address of LGDXRobot2 Cloud.";
  this->declare_parameter("cloud_address", "", cloudAddressParam);
  auto cloudRootCertParam = rcl_interfaces::msg::ParameterDescriptor{};
  cloudRootCertParam.description = "Path to server root certificate, required in LGDXRobot2 Cloud.";
  this->declare_parameter("cloud_root_cert", "", cloudRootCertParam);
  auto cloudClientKeyParam = rcl_interfaces::msg::ParameterDescriptor{};
  cloudClientKeyParam.description = "Path to client's private key, required in LGDXRobot2 Cloud.";
  this->declare_parameter("cloud_client_key", "", cloudClientKeyParam);
  auto cloudClientCertParam = rcl_interfaces::msg::ParameterDescriptor{};
  cloudClientCertParam.description = "Path to client's certificate chain, required in LGDXRobot2 Cloud.";
  this->declare_parameter("cloud_client_cert", "", cloudClientCertParam);

  // Serial Port
  auto serialPortEnableParam = rcl_interfaces::msg::ParameterDescriptor{};
  serialPortEnableParam.description = "Enable serial port communication for LGDXRobot2 MCU.";
  this->declare_parameter("serial_port_enable", false, serialPortEnableParam);
  auto serialPortNameParam = rcl_interfaces::msg::ParameterDescriptor{};
  serialPortNameParam.description = "Default serial port name or (Linux only) perform automated search if the this is unspecified.";
  this->declare_parameter("serial_port_name", "", serialPortNameParam);
  auto serialPortResetTransformParam = rcl_interfaces::msg::ParameterDescriptor{};
  serialPortResetTransformParam.description = "Reset robot transform on start up.";
  this->declare_parameter("serial_port_reset_transform", false, serialPortResetTransformParam);
  auto serialPortControlModeParam = rcl_interfaces::msg::ParameterDescriptor{};
  serialPortControlModeParam.description = "Robot control mode, using `joy` for joystick or `cmd_vel` for ROS nav stack.";
  this->declare_parameter("serial_port_control_mode", "cmd_vel", serialPortControlModeParam);
  auto serialPortOdomParam = rcl_interfaces::msg::ParameterDescriptor{};
  serialPortOdomParam.description = "Publishing odometry information from the robot.";
  this->declare_parameter("serial_port_publish_odom", false, serialPortOdomParam);
  auto serialPortTfParam = rcl_interfaces::msg::ParameterDescriptor{};
  serialPortTfParam.description = "Publishing tf information from the robot.";
  this->declare_parameter("serial_port_publish_tf", false, serialPortTfParam);
  auto serialPortBaseLinkParam = rcl_interfaces::msg::ParameterDescriptor{};
  serialPortBaseLinkParam.description = "Custom `base_link` name.";
  this->declare_parameter("serial_port_base_link_name", "base_link", serialPortBaseLinkParam);
  auto serialPortExternalImuParam = rcl_interfaces::msg::ParameterDescriptor{};
  serialPortExternalImuParam.description = "Using external IMU for odometry calcuation.";
  this->declare_parameter("serial_port_use_external_imu", false, serialPortExternalImuParam);

  // Simulation
  auto simEnableParam = rcl_interfaces::msg::ParameterDescriptor{};
  simEnableParam.description = "Enable simulation for LGDXRobot2 hardware, serial port must be disable for this feature.";
  this->declare_parameter("sim_enable", false, simEnableParam);

  /*
   * Object Init
   */
  bool cloudEnable = this->get_parameter("cloud_enable").as_bool();
  if (cloudEnable)
  {
    // Cloud
    std::string cloudAddress = this->get_parameter("cloud_address").as_string();
    std::string root = this->get_parameter("cloud_root_cert").as_string();
    std::string key = this->get_parameter("cloud_client_key").as_string();
    std::string cert = this->get_parameter("cloud_client_cert").as_string();
    cloud = std::make_unique<CloudAdapter>(cloudAddress.c_str(), 
      root.c_str(), 
      key.c_str(),
      cert.c_str(), 
      [this]()
      {
        // startNextExchangeCb
        robotStatus.connnectedCloud();
        cloudExchangeTimer->reset();
      },
      [this](const RobotClientsRespond *respond)
      {
        // updateDaemonCb
        cloudUpdate(respond);
      },
      [this](const char *message, int level)
      {
        // logCb
        logCallback(message, level);
      },
      [this](CloudFunctions function)
      {
        // errorCb
        cloudErrorQueue.push(function);
        if (cloudRetryTimer->is_canceled())
          cloudRetryTimer->reset();
      });

    std::random_device rd;
    std::mt19937 gen(rd());
    std::uniform_int_distribution<> dis(1000, 6000);
    int cloudRetryWait = dis(gen);
    RCLCPP_INFO(this->get_logger(), "LGDXRobot2 Cloud is enabled, the break for reconnection to the cloud is %d ms.", cloudRetryWait);
    cloudRetryTimer = this->create_wall_timer(std::chrono::milliseconds(cloudRetryWait), 
      std::bind(&DaemonNode::cloudRetry, this));
    cloudRetryTimer->cancel();
    
    cloudExchangeTimer = this->create_wall_timer(std::chrono::milliseconds(500), 
      std::bind(&DaemonNode::cloudExchange, this));
    cloudExchangeTimer->cancel();
    
    autoTaskPublisher = this->create_publisher<lgdxrobot2_daemon::msg::AutoTask>("/daemon/auto_task", 
      rclcpp::SensorDataQoS().reliable());
    autoTaskPublisherTimer = this->create_wall_timer(std::chrono::milliseconds(100), 
      [this]()
      {
        autoTaskPublisher->publish(currentTask);
      });

    autoTaskNextService = this->create_service<lgdxrobot2_daemon::srv::AutoTaskNext>("auto_task_next",
      [this](const std::shared_ptr<lgdxrobot2_daemon::srv::AutoTaskNext::Request> request,
        std::shared_ptr<lgdxrobot2_daemon::srv::AutoTaskNext::Response> response) 
      {
        if (!currentTask.next_token.empty() && 
            request->task_id == currentTask.task_id &&
            request->next_token == currentTask.next_token)
        {
          cloudAutoTaskNext();
          response->success = true;
        }
        else
        {
          response->success = false;
        }
      });
    autoTaskAbortService = this->create_service<lgdxrobot2_daemon::srv::AutoTaskAbort>("auto_task_abort",
      [this](const std::shared_ptr<lgdxrobot2_daemon::srv::AutoTaskAbort::Request> request,
        std::shared_ptr<lgdxrobot2_daemon::srv::AutoTaskAbort::Response> response)
      {
        if (!currentTask.next_token.empty() && 
            request->task_id == currentTask.task_id &&
            request->next_token == currentTask.next_token)
        {
          cloudAutoTaskAbort();
          response->success = true;
        }
        else
        {
          response->success = false;
        }
      });

    navThroughPosesActionClient = rclcpp_action::create_client<nav2_msgs::action::NavigateThroughPoses>(
        this,
        "navigate_through_poses");

    tfBuffer = std::make_unique<tf2_ros::Buffer>(this->get_clock());
    tfListener = std::make_shared<tf2_ros::TransformListener>(*tfBuffer);
  }

  // Serial Port
  bool serialPortEnable = this->get_parameter("serial_port_enable").as_bool();
  if (serialPortEnable)
  {
    std::string portName = this->get_parameter("serial_port_name").as_string();
    bool resetTransform = this->get_parameter("serial_port_reset_transform").as_bool();
    serialPort = std::make_unique<SerialPort>(portName,
      resetTransform,
      [this](const RobotData &robotData) 
      {
        serialUpdate(robotData);
      },
      [this](const char *message, int level)
      {
        logCallback(message, level);
      }
    );

    std::string controlMode = this->get_parameter("serial_port_control_mode").as_string();
    if (controlMode.empty() || controlMode == "cmd_vel")
    {
      cmdVelSubscription = this->create_subscription<geometry_msgs::msg::Twist>("cmd_vel", 
        rclcpp::SensorDataQoS().reliable(),
        std::bind(&DaemonNode::cmdVelCallback, this, std::placeholders::_1));
    }
    else if(controlMode == "joy")
    {
      joySubscription = this->create_subscription<sensor_msgs::msg::Joy>("joy",
        rclcpp::SensorDataQoS().reliable(),
        std::bind(&DaemonNode::joyCallback, this, std::placeholders::_1));
    }
    else
    {
      RCLCPP_FATAL(this->get_logger(), "Control mode is invalid, the program is terminaling");
      exit(0);
    }
    if (this->get_parameter("serial_port_use_external_imu").as_bool())
    {
      imuSubscription = this->create_subscription<sensor_msgs::msg::Imu>("/daemon/ext_imu",
        rclcpp::SensorDataQoS().reliable(),
        std::bind(&DaemonNode::imuCallback, this, std::placeholders::_1));
    }
    if (this->get_parameter("serial_port_publish_odom").as_bool())
    {
      baseLinkName = this->get_parameter("base_link_frame").as_string();
      odomPublisher = this->create_publisher<nav_msgs::msg::Odometry>("/daemon/odom",
        rclcpp::SensorDataQoS().reliable());
    }
    if (this->get_parameter("serial_port_publish_tf").as_bool())
    {
      tfBroadcaster = std::make_shared<tf2_ros::TransformBroadcaster>(this);
    }
  }

  /*
   * Everything Initialised
   */
  if (cloud != nullptr)
  {
    cloudGreet();
  }
}

void DaemonNode::logCallback(const char *msg, int level)
{
  switch (level)
  {
  case RCLCPP_LOG_MIN_SEVERITY_INFO:
    RCLCPP_INFO(this->get_logger(), "%s", msg);
    break;
  case RCLCPP_LOG_MIN_SEVERITY_ERROR:
    RCLCPP_ERROR(this->get_logger(), "%s", msg);
    break;
  }
}

void DaemonNode::cloudUpdate(const RobotClientsRespond *respond)
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
      robotStatus.taskCompleted();
    }
    else if (currentTask.task_progress_id == 4)
    {
      RCLCPP_INFO(this->get_logger(), "AutoTask Id: %d aborted.", task.taskid());
      if (navThroughPosesGoalHandle)
      {
        auto cancelResult = navThroughPosesActionClient->async_cancel_goal(
          navThroughPosesGoalHandle,
          [this](auto response)
          {
            if (response)
            {
              RCLCPP_INFO(this->get_logger(), "Navigation aborted.");
              navThroughPosesGoalHandle.reset();
            }
            else
            {
              RCLCPP_ERROR(this->get_logger(), "Navigation abort failed.");
            }
          }
        );
      }
      robotStatus.taskAborted();
    }
    else
    {
      RCLCPP_INFO(this->get_logger(), "Received AutoTask Id: %d, Progress: %d", task.taskid(), task.taskprogressid());
      if (task.waypoints_size())
      {
        RCLCPP_INFO(this->get_logger(), "This task has %d waypoint(s).", task.waypoints_size());
        std::vector<geometry_msgs::msg::PoseStamped> poses;
        auto pose = geometry_msgs::msg::PoseStamped();
        pose.header.stamp = rclcpp::Clock().now();
        pose.header.frame_id = "map";
        pose.pose.position.z = 0.0;
        for (int i = 0; i < task.waypoints_size(); i++)
        {
          const RobotClientsDof waypoint = task.waypoints(i);
          pose.pose.position.x = waypoint.x();
          pose.pose.position.y = waypoint.y();
          pose.pose.orientation = nav2_util::geometry_utils::orientationAroundZAxis(waypoint.rotation());
          poses.push_back(pose);
        }
        navThroughPoses(poses);
      }
      robotStatus.taskAssigned();
    }
  }

  // Handle Robot Command
  if (respond->has_commands())
  {
    auto commands = respond->commands();
    // Abort Task
    if (commands.aborttask() == true && currentCommands.aborttask() == false)
    {
      cloudAutoTaskAbort();
    }
    currentCommands.set_aborttask(commands.aborttask());

    // Emergency Stop
    if (commands.softwareemergencystop() == true && currentCommands.softwareemergencystop() == false)
    {
      robotStatus.enterCritical();
      // TODO: stop motor
    }
    else if (commands.softwareemergencystop() == false && currentCommands.softwareemergencystop() == true)
    {
      robotStatus.exitCritical();
    }
    criticalStatus.set_softwareemergencystop(commands.softwareemergencystop());
    currentCommands.set_softwareemergencystop(commands.softwareemergencystop());

    // Pause Task Assigement
    if (commands.pausetaskassigement() == true && currentCommands.pausetaskassigement() == false)
    {
      robotStatus.pauseTaskAssigement();
    }
    else if (commands.pausetaskassigement() == false && currentCommands.pausetaskassigement() == true)
    {
      robotStatus.resumeTaskAssigement();
    }
    currentCommands.set_pausetaskassigement(commands.pausetaskassigement());
  }
}

void DaemonNode::cloudRetry()
{
  cloudRetryTimer->cancel();
  // Avoid inf loop if the queue increases
  for (int i = 0, size = cloudErrorQueue.size(); i < size; i++)
  {
    CloudFunctions function = cloudErrorQueue.front();
    switch (function)
    {
    case CloudFunctions::Greet:
      cloudGreet();
      break;
    case CloudFunctions::Exchange:
      cloudExchange();
      break;
    case CloudFunctions::AutoTaskNext:
      cloudAutoTaskNext();
      break;
    case CloudFunctions::AutoTaskAbort:
      cloudAutoTaskAbort();
      break;
    }
    cloudErrorQueue.pop();
  }
}

void DaemonNode::cloudGreet()
{
  cloud->greet();
}

void DaemonNode::cloudExchange()
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
  std::vector<double> batteries = {11.59, 11.45};

  cloud->exchange(robotStatus.getRobotStatus(),
                  criticalStatus,
                  batteries,
                  robotPosition,
                  navProgress);
  // Don't reset the cloudExchangeTimer here
}

void DaemonNode::cloudAutoTaskNext()
{
  if (!currentTask.next_token.empty())
  {
    RCLCPP_INFO(this->get_logger(), "AutoTask will be done.");
    RobotClientsNextToken token;
    token.set_taskid(currentTask.task_id);
    token.set_nexttoken(currentTask.next_token);
    cloud->autoTaskNext(token);
  }
}

void DaemonNode::cloudAutoTaskAbort()
{
  if (!currentTask.next_token.empty())
  {
    robotStatus.taskAborting();
    RCLCPP_INFO(this->get_logger(), "AutoTask will be aborted.");
    RobotClientsNextToken token;
    token.set_taskid(currentTask.task_id);
    token.set_nexttoken(currentTask.next_token);
    cloud->autoTaskAbort(token);
  }
}

void DaemonNode::navThroughPoses(std::vector<geometry_msgs::msg::PoseStamped> &poses)
{
  using namespace std::placeholders;

  if (!navThroughPosesActionClient->wait_for_action_server())
  {
    RCLCPP_ERROR(this->get_logger(), "NAV2 stack is not ready.");
    return;
  }

  auto goal = nav2_msgs::action::NavigateThroughPoses::Goal();
  goal.poses = poses;

  auto goalOption = rclcpp_action::Client<nav2_msgs::action::NavigateThroughPoses>::SendGoalOptions();
  goalOption.goal_response_callback = std::bind(&DaemonNode::navThroughPosesGoalResponse, this, _1);
  goalOption.feedback_callback = std::bind(&DaemonNode::navThroughPosesFeedback, this, _1, _2);
  goalOption.result_callback = std::bind(&DaemonNode::navThroughPosesResult, this, _1);
  auto futureGoalHandle = navThroughPosesActionClient->async_send_goal(goal, goalOption);
  navThroughPosesGoalHandle = futureGoalHandle.get();
}

void DaemonNode::navThroughPosesGoalResponse(const rclcpp_action::ClientGoalHandle<nav2_msgs::action::NavigateThroughPoses>::SharedPtr &goalHandle)
{
  if (!goalHandle)
  {
    RCLCPP_ERROR(this->get_logger(), "navThroughPoses goal was rejected by server.");
  }
}

void DaemonNode::navThroughPosesFeedback(rclcpp_action::ClientGoalHandle<nav2_msgs::action::NavigateThroughPoses>::SharedPtr, 
  const std::shared_ptr<const nav2_msgs::action::NavigateThroughPoses::Feedback> feedback)
{
  navProgress.set_eta(rclcpp::Duration(feedback->estimated_time_remaining).seconds());
  navProgress.set_recoveries(feedback->number_of_recoveries);
  navProgress.set_distanceremaining(feedback->distance_remaining);
  navProgress.set_waypointsremaining(feedback->number_of_poses_remaining);
}

void DaemonNode::navThroughPosesResult(const rclcpp_action::ClientGoalHandle<nav2_msgs::action::NavigateThroughPoses>::WrappedResult &result)
{
  switch (result.code)
  {
    case rclcpp_action::ResultCode::SUCCEEDED:
      cloudAutoTaskNext();
      break;
    case rclcpp_action::ResultCode::ABORTED:
    case rclcpp_action::ResultCode::CANCELED:
    default:
      cloudAutoTaskAbort();
      return;
  }
}

void DaemonNode::serialUpdate(const RobotData& data)
{
  if (tfBroadcaster != nullptr || odomPublisher != nullptr)
  {
    tf2::Quaternion quaternion;
    quaternion.setRPY(0, 0, data.transform[2]);
    geometry_msgs::msg::Quaternion odomQuaternion = tf2::toMsg(quaternion);
    rclcpp::Time currentTime = this->get_clock()->now();

    if (tfBroadcaster != nullptr)
    {
      geometry_msgs::msg::TransformStamped odomTf;
      odomTf.header.stamp = currentTime;
      odomTf.header.frame_id = "odom";
      odomTf.child_frame_id = baseLinkName;
      odomTf.transform.translation.x = data.transform[0];
      odomTf.transform.translation.y = data.transform[1];
      odomTf.transform.translation.z = 0.0;
      odomTf.transform.rotation = odomQuaternion;
      tfBroadcaster->sendTransform(odomTf);
    }
    if (odomPublisher != nullptr)
    {
      nav_msgs::msg::Odometry odometry;
      odometry.header.stamp = currentTime;
      odometry.header.frame_id = "odom";
      odometry.pose.pose.position.x = data.transform[0];
      odometry.pose.pose.position.y = data.transform[1];
      odometry.pose.pose.position.z = 0.0;
      odometry.pose.pose.orientation = odomQuaternion;
      odometry.child_frame_id = baseLinkName;
      odometry.twist.twist.linear.x = data.forwardKinematic[0];
      odometry.twist.twist.linear.y = data.forwardKinematic[1];
      odometry.twist.twist.angular.z = data.forwardKinematic[2];
      odomPublisher->publish(odometry);
    }
  }
}

void DaemonNode::cmdVelCallback(const geometry_msgs::msg::Twist &msg)
{
  float x = msg.linear.x;
  float y = msg.linear.y;
  float w = msg.angular.z;
  // RCLCPP_INFO(this->get_logger(), "/cmd_vel %f %f %f", x, y, w);
  serialPort->setInverseKinematics(x, y, w);
}

void DaemonNode::joyCallback(const sensor_msgs::msg::Joy &msg)
{
  // E-Stop
  if (lastEstopButton[0] == 0 && msg.buttons[0] == 1)
  {
    // A = disable software E-Stop
    serialPort->setEstop(false);
    RCLCPP_INFO(this->get_logger(), "Software E-Stop Disabled");
  }
  lastEstopButton[0] = msg.buttons[0];
  if (lastEstopButton[1] == 0 && msg.buttons[1] == 1)
  {
    // B = enable software E-Stop
    serialPort->setEstop(true);
    RCLCPP_INFO(this->get_logger(), "Software E-Stop Enabled");
  }
  lastEstopButton[1] = msg.buttons[1];
  // Velocity Change
  if (lastVelocityChangeButton[0] == 0 && msg.buttons[6] == 1)
  {
    // LB = decrease max velocity
    if (maximumVelocity >= 0.2)
    {
      maximumVelocity -= 0.1;
      RCLCPP_INFO(this->get_logger(), "Maximum velocity decreased to %.1f m/s", maximumVelocity);
    }
  }
  lastVelocityChangeButton[0] = msg.buttons[6];
  if (lastVelocityChangeButton[1] == 0 && msg.buttons[7] == 1)
  {
    // RB = increase max velocity
    if (maximumVelocity < 1.0)
    {
      maximumVelocity += 0.1;
      RCLCPP_INFO(this->get_logger(), "Maximum velocity increased to %.1f m/s", maximumVelocity);
    }
  }
  lastVelocityChangeButton[1] = msg.buttons[7];
  // Control IK
  // Left Stick = XY
  float x = msg.axes[1] * maximumVelocity;
  float y = msg.axes[0] * maximumVelocity;
  if (msg.axes[1] == 0 && msg.axes[0] == 0)
  {
    // Use D-pad = XY if Left Stick no input
    x = msg.axes[7] * maximumVelocity;
    y = msg.axes[6] * maximumVelocity;
  }
  // LT = w left, RT = w right
  float w = (((msg.axes[4] - 1) / 2) - ((msg.axes[5] - 1) / 2)) * maximumVelocity;
  serialPort->setInverseKinematics(x, y, w);
}

void DaemonNode::imuCallback(const sensor_msgs::msg::Imu &msg)
{
  float ax = msg.linear_acceleration.x;
  float ay = msg.linear_acceleration.y;
  float az = msg.linear_acceleration.z;
  float gz = msg.angular_velocity.z;
  serialPort->setExternalImu(ax, ay, az, gz);
}
