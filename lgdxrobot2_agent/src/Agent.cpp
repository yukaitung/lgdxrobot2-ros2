#include "lgdxrobot2_agent/Agent.hpp"


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
  auto simEnableParam = rcl_interfaces::msg::ParameterDescriptor{};
  simEnableParam.description = "Enable simulation for LGDXRobot2 hardware, MCU must be disable for this feature.";
  this->declare_parameter("sim_enable", false, simEnableParam);

  // Signals
  cloudSignals = std::make_shared<CloudSignals>();
  mcuSignals = std::make_shared<McuSignals>();
  navigationSignals = std::make_shared<NavigationSignals>();
  robotControllerSignals = std::make_shared<RobotControllerSignals>();
  sensorSignals = std::make_shared<SensorSignals>();

  // Cloud
  bool cloudEnable = this->get_parameter("cloud_enable").as_bool();
  if (cloudEnable)
  {
    robotStatus = std::make_shared<RobotStatus>();
    cloud = std::make_unique<Cloud>(shared_from_this(), cloudSignals, robotStatus);
    navigation = std::make_unique<Navigation>(shared_from_this(), navigationSignals, robotStatus);
    robotController = std::make_unique<RobotController>(shared_from_this(), robotControllerSignals, robotStatus);

    robotDataPublisher = this->create_publisher<lgdxrobot2_agent::msg::RobotData>("/agent/robot_data", 
      rclcpp::SensorDataQoS().reliable());
    robotDataPublisherTimer = this->create_wall_timer(std::chrono::milliseconds(100), 
      [this]()
      {
        robotData.robot_status = static_cast<int>(robotStatus->GetStatus());
        robotDataPublisher->publish(robotData);
      });

    cloudExchangeTimer = this->create_wall_timer(std::chrono::milliseconds(500), 
      std::bind(&Agent::CloudExchange, this));
    cloudExchangeTimer->cancel();
    
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

  bool simEnable = this->get_parameter("sim_enable").as_bool();
  if (mcuEnable && simEnable)
  {
    RCLCPP_FATAL(this->get_logger(), "Simulation and MCU cannot be enabled at the same time.");
    exit(0);
  }

  // Signals
  if (cloudEnable)
  {
    cloudSignals->NextExchange.connect([this](){
      // Initalise the timer for sending exchange
      cloudExchangeTimer->reset();
    });
    cloudSignals->HandleExchange.connect(boost::bind(&RobotController::OnCloudExchangeDone, robotController.get(), boost::placeholders::_1));
    robotControllerSignals->NavigationStart.connect(boost::bind(&Navigation::Start, navigation.get(), boost::placeholders::_1));
    robotControllerSignals->NavigationAbort.connect(boost::bind(&Navigation::Abort, navigation.get()));
    robotControllerSignals->AutoTaskNext.connect(boost::bind(&Cloud::AutoTaskNext, cloud.get(), boost::placeholders::_1));
    robotControllerSignals->AutoTaskAbort.connect(boost::bind(&Cloud::AutoTaskAbort, cloud.get(), boost::placeholders::_1));
    navigationSignals->NextNavigation.connect(boost::bind(&RobotController::NavigationStart, robotController.get()));
    navigationSignals->Abort.connect(boost::bind(&RobotController::CloudAutoTaskAbort, robotController.get(), boost::placeholders::_1));
  }
  if (mcuEnable)
  {
    mcuSignals->UpdateRobotData.connect(boost::bind(&Agent::OnRobotDataReceived, this, boost::placeholders::_1));
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

  if (simEnable && cloudEnable)
  {
    robotData.battery[0] = 12.1;
    robotData.battery[1] = 12.2;
    // Start the connection now for simulation
    cloud->Greet("");
  }
}

void Agent::OnRobotDataReceived(const RobotData &rd)
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
  batteries[0] = robotData.battery[0];
  batteries[1] = robotData.battery[1];

  RobotClientsAutoTaskNavProgress navProgress = navigation->GetNavProgress();
  
  RobotClientsRobotCriticalStatus criticalStatus = robotController->GetCriticalStatus();
  cloud->Exchange(criticalStatus,
    batteries,
    robotPosition,
    navProgress);
  // Don't reset the cloudExchangeTimer here
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