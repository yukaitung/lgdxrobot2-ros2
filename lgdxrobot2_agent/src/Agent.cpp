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

  // Signals
  mcuSignals = std::make_shared<McuSignals>();
  sensorSignals = std::make_shared<SensorSignals>();

  // Cloud
  bool cloudEnable = this->get_parameter("cloud_enable").as_bool();
  if (cloudEnable)
  {
    robotStatus = std::make_shared<RobotStatus>();
    cloud = std::make_unique<Cloud>(shared_from_this(), robotStatus);
    navigation = std::make_unique<Navigation>(shared_from_this(), robotStatus);

    tfBuffer = std::make_unique<tf2_ros::Buffer>(this->get_clock());
    tfListener = std::make_shared<tf2_ros::TransformListener>(*tfBuffer);

    cloudExchangeTimer = this->create_wall_timer(std::chrono::milliseconds(500), 
      std::bind(&Agent::CloudExchangeReady, this));
    cloudExchangeTimer->cancel();
  }

  // MCU
  bool mcuEnable = this->get_parameter("mcu_enable").as_bool();
  if (1)
  {
    mcu = std::make_unique<Mcu>(shared_from_this(), mcuSignals);
    sensors = std::make_unique<Sensors>(shared_from_this(), sensorSignals);
  }
}

void Agent::CloudExchangeReady()
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