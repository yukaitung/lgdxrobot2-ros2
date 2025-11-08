#include "lgdxrobot2_agent/Sensors.hpp"

#include "tf2_geometry_msgs/tf2_geometry_msgs.hpp"

Sensors::Sensors(rclcpp::Node::SharedPtr node, std::shared_ptr<SensorSignals> sensorSignalsPtr) :
  clock_(node->get_clock()),
  logger_(node->get_logger())
{
  sensorSignals = sensorSignalsPtr;

  // Parameters
  auto mcuControlModeParam = rcl_interfaces::msg::ParameterDescriptor{};
  mcuControlModeParam.description = "Robot control mode, using `joy` for joystick or `cmd_vel` for ROS nav stack.";
  node->declare_parameter("mcu_control_mode", "cmd_vel", mcuControlModeParam);
  auto mcuOdomParam = rcl_interfaces::msg::ParameterDescriptor{};
  mcuOdomParam.description = "Publishing odometry information from the robot.";
  node->declare_parameter("mcu_publish_odom", false, mcuOdomParam);
  auto mcuTfParam = rcl_interfaces::msg::ParameterDescriptor{};
  mcuTfParam.description = "Publishing tf information from the robot.";
  node->declare_parameter("mcu_publish_tf", false, mcuTfParam);
  auto mcuJointStateParam = rcl_interfaces::msg::ParameterDescriptor{};
  mcuJointStateParam.description = "Publishing joint state information from the robot.";
  node->declare_parameter("mcu_publish_joint_state", false, mcuJointStateParam);
  auto mcuBaseLinkParam = rcl_interfaces::msg::ParameterDescriptor{};
  mcuBaseLinkParam.description = "Custom `base_link` name.";
  node->declare_parameter("mcu_base_link_name", "base_link", mcuBaseLinkParam);
  auto mcuExternalImuParam = rcl_interfaces::msg::ParameterDescriptor{};
  mcuExternalImuParam.description = "Using external IMU for odometry calcuation.";

  // Topics
  std::string controlMode = node->get_parameter("mcu_control_mode").as_string();
  if (controlMode.empty() || controlMode == "cmd_vel")
  {
    cmdVelSubscription = node->create_subscription<geometry_msgs::msg::Twist>("cmd_vel", 
      rclcpp::SensorDataQoS().reliable(),
      std::bind(&Sensors::CmdVelCallback, this, std::placeholders::_1));
  }
  else if(controlMode == "joy")
  {
    joySubscription = node->create_subscription<sensor_msgs::msg::Joy>("joy",
      rclcpp::SensorDataQoS().reliable(),
      std::bind(&Sensors::JoyCallback, this, std::placeholders::_1));
  }
  else
  {
    RCLCPP_FATAL(logger_, "Control mode is invalid, the program is terminaling");
    exit(0);
  }
  if (node->get_parameter("mcu_publish_odom").as_bool())
  {
    baseLinkName = node->get_parameter("mcu_base_link_name").as_string();
    odomPublisher = node->create_publisher<nav_msgs::msg::Odometry>("/agent/odom",
      rclcpp::SensorDataQoS().reliable());
  }
  if (node->get_parameter("mcu_publish_tf").as_bool())
  {
    tfBroadcaster = std::make_shared<tf2_ros::TransformBroadcaster>(node);
  }
  if (node->get_parameter("mcu_publish_joint_state").as_bool())
  {
    jointStatePublisher = node->create_publisher<sensor_msgs::msg::JointState>("/joint_states", 
      rclcpp::SensorDataQoS().reliable());
  }
  needPublishOdom = tfBroadcaster != nullptr || odomPublisher != nullptr;
}

void Sensors::CmdVelCallback(const geometry_msgs::msg::Twist &msg)
{
  float x = msg.linear.x;
  float y = msg.linear.y;
  float w = msg.angular.z;
  // RCLCPP_INFO(node->get_logger(), "/cmd_vel %f %f %f", x, y, w);
  sensorSignals->SetInverseKinematics(x, y, w);
}

void Sensors::JoyCallback(const sensor_msgs::msg::Joy &msg)
{
  // E-Stop
  if (lastEstopButton[0] == 0 && msg.buttons[0] == 1)
  {
    // A = disable software E-Stop
    sensorSignals->SetEstop(false);
    RCLCPP_INFO(logger_, "Software E-Stop Disabled");
  }
  lastEstopButton[0] = msg.buttons[0];
  if (lastEstopButton[1] == 0 && msg.buttons[1] == 1)
  {
    // B = enable software E-Stop
    sensorSignals->SetEstop(true);
    RCLCPP_INFO(logger_, "Software E-Stop Enabled");
  }
  lastEstopButton[1] = msg.buttons[1];
  // Velocity Change
  if (lastVelocityChangeButton[0] == 0 && msg.buttons[6] == 1)
  {
    // LB = decrease max velocity
    if (maximumVelocity >= 0.2)
    {
      maximumVelocity -= 0.1;
      RCLCPP_INFO(logger_, "Maximum velocity decreased to %.1f m/s", maximumVelocity);
    }
  }
  lastVelocityChangeButton[0] = msg.buttons[6];
  if (lastVelocityChangeButton[1] == 0 && msg.buttons[7] == 1)
  {
    // RB = increase max velocity
    if (maximumVelocity < 1.0)
    {
      maximumVelocity += 0.1;
      RCLCPP_INFO(logger_, "Maximum velocity increased to %.1f m/s", maximumVelocity);
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
  float w = ((msg.axes[4] - 1) - (msg.axes[5] - 1)) * maximumVelocity;
  sensorSignals->SetInverseKinematics(x, y, w);
}

void Sensors::PublishOdom(const RobotData& data)
{
  if (needPublishOdom)
  {
    tf2::Quaternion quaternion;
    quaternion.setRPY(0, 0, data.transform[2]);
    geometry_msgs::msg::Quaternion odomQuaternion = tf2::toMsg(quaternion);
    rclcpp::Time currentTime = clock_->now();

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
  if (jointStatePublisher != nullptr)
  {
    for (int i = 0; i < 4; i++)
    {
      motorsPosition[i] += data.motorsActualVelocity[i] * (data.responseTime / 1000.0);
    }
    rclcpp::Time currentTime = clock_->now();
    sensor_msgs::msg::JointState jointState;
    jointState.header.stamp = currentTime;
    jointState.name = {"wheel1_link_joint", "wheel2_link_joint", "wheel3_link_joint", "wheel4_link_joint"};
    jointState.position = {motorsPosition[0], motorsPosition[1], motorsPosition[2], motorsPosition[3]};
    jointState.velocity = {data.motorsActualVelocity[0], data.motorsActualVelocity[1], data.motorsActualVelocity[2], data.motorsActualVelocity[3]};
    if(jointStatePublisher)
      jointStatePublisher->publish(jointState);
  }
}