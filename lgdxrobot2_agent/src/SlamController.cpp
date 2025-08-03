#include "lgdxrobot2_agent/SlamController.hpp"

#include "nav2_util/geometry_utils.hpp"

SlamController::SlamController(rclcpp::Node::SharedPtr node,
    std::shared_ptr<SlamControllerSignals> slamControllerSignalsPtr,
    std::shared_ptr<RobotStatus> robotStatusPtr,
    std::shared_ptr<RobotClientsAutoTaskNavProgress> navProgressPtr
  ) : logger_(node->get_logger())
{
  slamControllerSignals = slamControllerSignalsPtr;
  robotStatus = robotStatusPtr;
  navProgress = navProgressPtr;

  tfBuffer = std::make_unique<tf2_ros::Buffer>(node->get_clock());
  tfListener = std::make_shared<tf2_ros::TransformListener>(*tfBuffer);

  slamExchangeTimer = node->create_wall_timer(std::chrono::milliseconds(500), 
    std::bind(&SlamController::SlamExchange2, this));
  slamExchangeTimer->cancel();

  // Topics
  robotDataPublisherTimer = node->create_wall_timer(std::chrono::milliseconds(100), 
    [this]()
    {
      robotData.robot_status = static_cast<int>(robotStatus->GetStatus());
      robotDataPublisher->publish(robotData);
    });
  robotDataPublisher = node->create_publisher<lgdxrobot2_agent::msg::RobotData>("/agent/robot_data", 
    rclcpp::SensorDataQoS().reliable());

  mapSubscription = node->create_subscription<nav_msgs::msg::OccupancyGrid>("map", 
    rclcpp::SensorDataQoS().reliable(), 
    std::bind(&SlamController::MapCallback, this, std::placeholders::_1));
}

void SlamController::MapCallback(const nav_msgs::msg::OccupancyGrid &msg)
{
  // Compare the map with the current map
  if ((int)msg.data.size() == mapData.data_size())
  {
    for (size_t i = 0; i < msg.data.size(); i++)
    {
      bool same = true;
      if (msg.data[i] != mapData.data(i))
      {
        same = false;
        break;
      }
      if (same)
      {
        return;
      }
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
  md->Reserve(msg.data.size());
  for (size_t i = 0; i < msg.data.size(); i++)
  {
    md->AddAlreadyReserved(msg.data[i]);
  }
  slamControllerSignals->SlamExchange3(status, exchange, mapData);
}

void SlamController::UpdateExchange()
{
  exchange.set_robotstatus(RobotClientsRobotStatus::Paused);
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
  catch (const tf2::TransformException &ex) {}
  exchange.mutable_navprogress()->CopyFrom(*navProgress);
}

void SlamController::SlamExchange2()
{
  if (!slamExchangeTimer->is_canceled())
    slamExchangeTimer->cancel();

  UpdateExchange();
  slamControllerSignals->SlamExchange2(status, exchange);
  // Don't reset the slamExchangeTimer here
}

void SlamController::StatSlamExchange()
{
  slamExchangeTimer->reset();
}

void SlamController::OnSlamExchangeDone(const RobotClientsSlamRespond *respond)
{
  if (respond->has_goal())
  {
    std::vector<geometry_msgs::msg::PoseStamped> poses;
    auto pose = geometry_msgs::msg::PoseStamped();
    pose.header.stamp = rclcpp::Clock().now();
    pose.header.frame_id = "map";
    pose.pose.position.z = 0.0;
    pose.pose.position.x = respond->goal().x();
    pose.pose.position.y = respond->goal().y();
    pose.pose.orientation = nav2_util::geometry_utils::orientationAroundZAxis(respond->goal().rotation());
    poses.push_back(pose);
    slamControllerSignals->NavigationStart(poses);
  }
  if (respond->has_commands())
  {
    if (respond->commands().has_abortgoal() && respond->commands().abortgoal() == true)
    {
      slamControllerSignals->NavigationAbort();
    }
    if (respond->commands().has_softwareemergencystop() && respond->commands().softwareemergencystop() == true)
    {
      robotStatus->EnterCritical();
    }
    else if (respond->commands().has_softwareemergencystop() && respond->commands().softwareemergencystop() == false)
    {
      robotStatus->ExitCritical();
    }
    if (respond->commands().has_getmap() && respond->commands().getmap() == true)
    {
      slamControllerSignals->SlamExchange3(status, exchange, mapData);
    }
  }
}

void SlamController::OnNavigationDone()
{
  status = RobotClientsRealtimeNavResults::SlamSuccess;
}

void SlamController::OnNavigationAborted(RobotClientsAbortReason reason)
{
  status = RobotClientsRealtimeNavResults::SlamAborted;
}

void SlamController::OnRobotDataReceived(const RobotData &rd)
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

void SlamController::Shutdown()
{
  if (slamExchangeTimer != nullptr && !slamExchangeTimer->is_canceled())
  {
    slamExchangeTimer->cancel();
  }
}