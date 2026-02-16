#include "lgdxrobot2_agent/Sensors.hpp"

#include "tf2_geometry_msgs/tf2_geometry_msgs.hpp"

Sensors::Sensors(rclcpp::Node::SharedPtr node, std::shared_ptr<SensorSignals> sensorSignalsPtr) :
  _clock(node->get_clock()),
  _logger(node->get_logger())
{
  sensorSignals = sensorSignalsPtr;

  // Parameters
  auto mcuTfParam = rcl_interfaces::msg::ParameterDescriptor{};
  mcuTfParam.description = "Publishing tf information from the robot.";
  node->declare_parameter("publish_tf", false, mcuTfParam);
  auto mcuBaseLinkParam = rcl_interfaces::msg::ParameterDescriptor{};
  mcuBaseLinkParam.description = "Custom `base_link` name.";
  node->declare_parameter("base_link_name", "base_link", mcuBaseLinkParam);
  auto mcuUseJoyParam = rcl_interfaces::msg::ParameterDescriptor{};
  mcuUseJoyParam.description = "Control robot using `joy` node.";
  node->declare_parameter("use_joy", false, mcuUseJoyParam);

  // Subscriber
  cmdVelSubscription = node->create_subscription<geometry_msgs::msg::Twist>("cmd_vel", 
    rclcpp::SensorDataQoS().reliable(),
    std::bind(&Sensors::CmdVelCallback, this, std::placeholders::_1));
  if (node->get_parameter("use_joy").as_bool())
  {
    joySubscription = node->create_subscription<sensor_msgs::msg::Joy>("joy",
      rclcpp::SensorDataQoS().reliable(),
      std::bind(&Sensors::JoyCallback, this, std::placeholders::_1));
  }

  // Publisher
  imuPublisher = node->create_publisher<sensor_msgs::msg::Imu>("agent/imu", 
    rclcpp::SensorDataQoS().reliable());
  magneticFieldPublisher = node->create_publisher<sensor_msgs::msg::MagneticField>("agent/mag", 
    rclcpp::SensorDataQoS().reliable());
  systemPublisher = node->create_publisher<lgdxrobot2_msgs::msg::System>("agent/system", 
    rclcpp::SensorDataQoS().reliable());
  odomPublisher = node->create_publisher<nav_msgs::msg::Odometry>("agent/odom",
    rclcpp::SensorDataQoS().reliable());
  jointStatePublisher = node->create_publisher<sensor_msgs::msg::JointState>("joint_states", 
    rclcpp::SensorDataQoS().reliable());
  baseLinkName = node->get_parameter("base_link_name").as_string();
  if (node->get_parameter("publish_tf").as_bool())
  {
    tfBroadcaster = std::make_shared<tf2_ros::TransformBroadcaster>(node);
  }
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
    RCLCPP_INFO(_logger, "Software E-Stop Disabled");
  }
  lastEstopButton[0] = msg.buttons[0];
  if (lastEstopButton[1] == 0 && msg.buttons[1] == 1)
  {
    // B = enable software E-Stop
    sensorSignals->SetEstop(true);
    RCLCPP_INFO(_logger, "Software E-Stop Enabled");
  }
  lastEstopButton[1] = msg.buttons[1];
  // Velocity Change
  if (lastVelocityChangeButton[0] == 0 && msg.buttons[6] == 1)
  {
    // LB = decrease max velocity
    if (maximumVelocity >= 0.2)
    {
      maximumVelocity -= 0.1;
      RCLCPP_INFO(_logger, "Maximum velocity decreased to %.1f m/s", maximumVelocity);
    }
  }
  lastVelocityChangeButton[0] = msg.buttons[6];
  if (lastVelocityChangeButton[1] == 0 && msg.buttons[7] == 1)
  {
    // RB = increase max velocity
    if (maximumVelocity < 1.0)
    {
      maximumVelocity += 0.1;
      RCLCPP_INFO(_logger, "Maximum velocity increased to %.1f m/s", maximumVelocity);
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
  float w = ((msg.axes[4] - 1) - (msg.axes[5] - 1)) * maximumVelocity * 2;
  sensorSignals->SetInverseKinematics(x, y, w);
}

void Sensors::Publish(const McuData& mcuData)
{
  if (imuPublisher != nullptr)
  {
    sensor_msgs::msg::Imu imu;
    imu.header.stamp = _clock->now();
    imu.orientation_covariance[0] = -1;
    imu.angular_velocity.x = mcuData.imu.gyroscope.x;
    imu.angular_velocity.y = mcuData.imu.gyroscope.y;
    imu.angular_velocity.z = mcuData.imu.gyroscope.z;
    imu.angular_velocity_covariance[0] = mcuData.imu.accelerometer_covariance.x;
    imu.angular_velocity_covariance[4] = mcuData.imu.accelerometer_covariance.y;
    imu.angular_velocity_covariance[8] = mcuData.imu.accelerometer_covariance.z;
    imu.linear_acceleration.x = mcuData.imu.accelerometer.x;
    imu.linear_acceleration.y = mcuData.imu.accelerometer.y;
    imu.linear_acceleration.z = mcuData.imu.accelerometer.z;
    imu.linear_acceleration_covariance[0] = mcuData.imu.accelerometer_covariance.x;
    imu.linear_acceleration_covariance[4] = mcuData.imu.accelerometer_covariance.y;
    imu.linear_acceleration_covariance[8] = mcuData.imu.accelerometer_covariance.z;
    imuPublisher->publish(imu);
  }

  if (magneticFieldPublisher != nullptr)
  {
    sensor_msgs::msg::MagneticField magneticField;
    magneticField.header.stamp = _clock->now();
    magneticField.magnetic_field.x = mcuData.imu.magnetometer.x / 1000000;
    magneticField.magnetic_field.y = mcuData.imu.magnetometer.y / 1000000;
    magneticField.magnetic_field.z = mcuData.imu.magnetometer.z / 1000000;
    magneticField.magnetic_field_covariance[0] = mcuData.imu.magnetometer_covariance.x;
    magneticField.magnetic_field_covariance[4] = mcuData.imu.magnetometer_covariance.y;
    magneticField.magnetic_field_covariance[8] = mcuData.imu.magnetometer_covariance.z;
    magneticFieldPublisher->publish(magneticField);
  }

  if (systemPublisher != nullptr)
  {
    lgdxrobot2_msgs::msg::System system;
    system.battery1.voltage = mcuData.battery1.voltage;
    system.battery1.current = mcuData.battery1.current;
    system.battery2.voltage = mcuData.battery2.voltage;
    system.battery2.current = mcuData.battery2.current;
    system.software_emergency_stop_enabled = mcuData.software_emergency_stop_enabled;
    system.hardware_emergency_stop_enabled = mcuData.hardware_emergency_stop_enabled;
    system.bettery_low_emergency_stop_enabled = mcuData.bettery_low_emergency_stop_enabled;
    systemPublisher->publish(system);
  }
  
  tf2::Quaternion quaternion;
  quaternion.setRPY(0, 0, mcuData.transform.rotation);
  geometry_msgs::msg::Quaternion odomQuaternion = tf2::toMsg(quaternion);
  if (tfBroadcaster != nullptr)
  {
    geometry_msgs::msg::TransformStamped odomTf;
    odomTf.header.stamp = _clock->now();
    odomTf.header.frame_id = "odom";
    odomTf.child_frame_id = baseLinkName;
    odomTf.transform.translation.x = mcuData.transform.x;
    odomTf.transform.translation.y = mcuData.transform.y;
    odomTf.transform.translation.z = 0.0;
    odomTf.transform.rotation = odomQuaternion;
    tfBroadcaster->sendTransform(odomTf);
  }
  if (odomPublisher != nullptr)
  {
    nav_msgs::msg::Odometry odometry;
    odometry.header.stamp = _clock->now();
    odometry.header.frame_id = "odom";
    odometry.pose.pose.position.x = mcuData.transform.x;
    odometry.pose.pose.position.y = mcuData.transform.y;
    odometry.pose.pose.position.z = 0.0;
    odometry.pose.pose.orientation = odomQuaternion;
    odometry.child_frame_id = baseLinkName;
    odometry.twist.twist.linear.x = mcuData.forward_kinematic.x;
    odometry.twist.twist.linear.y = mcuData.forward_kinematic.y;
    odometry.twist.twist.angular.z = mcuData.forward_kinematic.rotation;
    odomPublisher->publish(odometry);
  }

  if (jointStatePublisher != nullptr)
  {
    for (int i = 0; i < 4; i++)
    {
      motorsPosition[i] += mcuData.motors_actual_velocity[i] * (mcuData.response_time / 1000.0);
    }
    sensor_msgs::msg::JointState jointState;
    jointState.header.stamp = _clock->now();
    jointState.name = {"wheel1_link_joint", "wheel2_link_joint", "wheel3_link_joint", "wheel4_link_joint"};
    jointState.position = {motorsPosition[0], motorsPosition[1], motorsPosition[2], motorsPosition[3]};
    jointState.velocity = {mcuData.motors_actual_velocity[0], mcuData.motors_actual_velocity[1], mcuData.motors_actual_velocity[2], mcuData.motors_actual_velocity[3]};
    jointStatePublisher->publish(jointState);
  }
}
