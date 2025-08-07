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
  auto cloudSlamEnableParam = rcl_interfaces::msg::ParameterDescriptor{};
  cloudSlamEnableParam.description = "Enable LGDXRobot Cloud SLAM Mode.";
  this->declare_parameter("cloud_slam_enable", false, cloudSlamEnableParam);

  // Signals
  cloudSignals = std::make_shared<CloudSignals>();
  mcuSignals = std::make_shared<McuSignals>();
  navigationSignals = std::make_shared<NavigationSignals>();
  robotControllerSignals = std::make_shared<RobotControllerSignals>();
  sensorSignals = std::make_shared<SensorSignals>();

  // Main Controller
  navProgress = std::make_shared<RobotClientsAutoTaskNavProgress>();
  robotController = std::make_unique<RobotController>(shared_from_this(), robotControllerSignals, navProgress);

  // Cloud
  bool cloudEnable = this->get_parameter("cloud_enable").as_bool();
  if (cloudEnable)
  {
    cloud = std::make_unique<Cloud>(shared_from_this(), cloudSignals);
    navigation = std::make_unique<Navigation>(shared_from_this(), navigationSignals, navProgress);
  }
  bool cloudSlamEnable = this->get_parameter("cloud_slam_enable").as_bool();
  if (cloudSlamEnable)
  {
    map = std::make_unique<Map>(shared_from_this());
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
    Shutdown();
  }

  // Signals
  if (cloudEnable)
  {
    robotControllerSignals->NavigationStart.connect(boost::bind(&Navigation::Start, navigation.get(), boost::placeholders::_1));
    robotControllerSignals->NavigationAbort.connect(boost::bind(&Navigation::Abort, navigation.get()));
    robotControllerSignals->AutoTaskNext.connect(boost::bind(&Cloud::AutoTaskNext, cloud.get(), boost::placeholders::_1));
    robotControllerSignals->AutoTaskAbort.connect(boost::bind(&Cloud::AutoTaskAbort, cloud.get(), boost::placeholders::_1));

    navigationSignals->Done.connect(boost::bind(&RobotController::OnNavigationDone, robotController.get()));
    navigationSignals->Stuck.connect(boost::bind(&RobotController::OnNavigationStuck, robotController.get()));
    navigationSignals->Cleared.connect(boost::bind(&RobotController::OnNavigationCleared, robotController.get()));
    navigationSignals->Abort.connect(boost::bind(&RobotController::OnNavigationAborted, robotController.get()));

    cloudSignals->StreamError.connect(boost::bind(&Cloud::Error, cloud.get(), boost::placeholders::_1));
    if (cloudSlamEnable)
    {
      robotControllerSignals->SlamExchange.connect(boost::bind(&Cloud::SlamExchange, cloud.get(), boost::placeholders::_1));
      robotControllerSignals->SaveMap.connect(boost::bind(&Map::Save, map.get()));
      robotControllerSignals->Shutdown.connect(boost::bind(&Agent::Shutdown, this));

      cloudSignals->NextExchange.connect(boost::bind(&RobotController::OnNextSlamExchange, robotController.get()));
      cloudSignals->HandleSlamExchange.connect(boost::bind(&RobotController::OnHandleSlamExchange, robotController.get(), boost::placeholders::_1));
    }
    else
    {
      robotControllerSignals->CloudExchange.connect(boost::bind(&Cloud::Exchange, cloud.get(), boost::placeholders::_1));

      cloudSignals->NextExchange.connect(boost::bind(&RobotController::OnNextCloudChange, robotController.get()));
      cloudSignals->HandleExchange.connect(boost::bind(&RobotController::OnHandleClouldExchange, robotController.get(), boost::placeholders::_1));
    }
  }
  if (mcuEnable)
  {
    mcuSignals->UpdateRobotData.connect(boost::bind(&Sensors::PublishOdom, sensors.get(), boost::placeholders::_1));
    mcuSignals->UpdateRobotData.connect(boost::bind(&RobotController::OnRobotDataReceived, robotController.get(), boost::placeholders::_1));
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
    // Start the connection now for simulation
    cloud->Greet("");
  }
}

void Agent::Shutdown()
{
  RCLCPP_INFO(this->get_logger(), "Shutting down agent");
  if (cloud != nullptr)
  {
    cloud->Shutdown();
  }
  if (robotController != nullptr)
  {
    robotController->Shutdown();
  }
  rclcpp::shutdown();
  exit(0);
}