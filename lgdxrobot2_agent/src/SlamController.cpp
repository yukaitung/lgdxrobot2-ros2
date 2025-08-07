#include <csignal>
#include <thread>

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
    std::bind(&SlamController::SlamExchange, this));
  slamExchangeTimer->cancel();

  // Topics
  robotDataPublisherTimer = node->create_wall_timer(std::chrono::milliseconds(100), 
    [this]()
    {
      robotData.robot_status = static_cast<int>(robotStatus->GetStatus());
      robotDataPublisher->publish(robotData);
    });
  robotDataPublisher = node->create_publisher<lgdxrobot2_agent::msg::RobotData>("agent/robot_data", 
    rclcpp::SensorDataQoS().reliable());

  mapSubscription = node->create_subscription<nav_msgs::msg::OccupancyGrid>("map", 
    rclcpp::SensorDataQoS().reliable(), 
    std::bind(&SlamController::MapCallback, this, std::placeholders::_1));
}

void SlamController::MapCallback(const nav_msgs::msg::OccupancyGrid &msg)
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

void SlamController::UpdateExchange()
{
  exchange.set_robotstatus(robotStatus->GetStatus() == RobotClientsRobotStatus::Critical ? RobotClientsRobotStatus::Critical : RobotClientsRobotStatus::Paused);
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

void SlamController::SlamExchange()
{
  if (!slamExchangeTimer->is_canceled())
    slamExchangeTimer->cancel();

  UpdateExchange();

  if (mapHasUpdated)
  {
    slamControllerSignals->SlamExchange3(status, exchange, mapData);
    mapHasUpdated = false;
  }
  else
  {
    slamControllerSignals->SlamExchange2(status, exchange);
  }
  // Don't reset the slamExchangeTimer here
}

void SlamController::StatSlamExchange()
{
  slamExchangeTimer->reset();
}

void SlamController::OnSlamExchangeDone(const RobotClientsSlamCommands *respond)
{
  if (respond->has_setgoal())
  {
    if (status == RobotClientsSlamStatus::SlamRunning)
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
    slamControllerSignals->NavigationStart(poses);
    status = RobotClientsSlamStatus::SlamRunning;
  }
  if (respond->has_abortgoal())
  {
    RCLCPP_INFO(logger_, "Aborting the current goal");
    slamControllerSignals->NavigationAbort();
  }
  if (respond->has_softwareemergencystopenable() && respond->softwareemergencystopenable() == true)
  {
    RCLCPP_INFO(logger_, "Enabling software emergency stop");
    robotStatus->EnterCritical();
  }
  if (respond->has_softwareemergencystopdisable() && respond->softwareemergencystopdisable() == true)
  {
    RCLCPP_INFO(logger_, "Disabling software emergency stop");
    robotStatus->ExitCritical();
  }
  if (respond->has_savemap() && respond->savemap() == true)
  {
    RCLCPP_INFO(logger_, "Saving the map");
    slamControllerSignals->SaveMap();
  }
  if (respond->has_refreshmap() && respond->refreshmap() == true)
  {
    RCLCPP_INFO(logger_, "Refreshing the map");
    mapHasUpdated = true;
  }
  if (respond->has_abortslam() && respond->abortslam() == true)
  {
    RCLCPP_INFO(logger_, "Aborting the current SLAM");
    slamControllerSignals->NavigationAbort();
    slamControllerSignals->Shutdown();
  }
  if (respond->has_completeslam() && respond->completeslam() == true)
  {
    slamControllerSignals->NavigationAbort();
    RCLCPP_INFO(logger_, "Completing the current SLAM and saving the map with 5 seconds blocking");
    slamControllerSignals->SaveMap();
    std::this_thread::sleep_for(std::chrono::seconds(5));
    slamControllerSignals->Shutdown();
  }
}

void SlamController::OnNavigationDone()
{
  status = RobotClientsSlamStatus::SlamSuccess;
}

void SlamController::OnNavigationAborted(RobotClientsAbortReason)
{
  if (overwriteGoal)
  {
    // Not showing aborted because the goal was overwritten
    overwriteGoal = false;
  }
  else
  {
    status = RobotClientsSlamStatus::SlamAborted;
  }
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