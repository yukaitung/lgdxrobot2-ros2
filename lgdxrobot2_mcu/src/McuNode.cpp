#include "McuNode.hpp"

// Odom
#include "geometry_msgs/msg/transform_stamped.hpp"
#include "tf2_geometry_msgs/tf2_geometry_msgs.hpp"

void McuNode::serialDebugCallback(const std::string &msg, int level)
{
  switch(level)
  {
    case RCLCPP_LOG_MIN_SEVERITY_INFO:
      RCLCPP_INFO(this->get_logger(), "%s", msg.c_str());
      break;
    case RCLCPP_LOG_MIN_SEVERITY_ERROR:
      RCLCPP_ERROR(this->get_logger(), "%s", msg.c_str());
      break;
  }
}

void McuNode::serialReadCallback(const McuData& data)
{
  if(publishTf || publishOdom)
  {
    tf2::Quaternion quaternion;
    quaternion.setRPY(0, 0, data.transform[2]);
    geometry_msgs::msg::Quaternion odomQuaternion = tf2::toMsg(quaternion);
    rclcpp::Time currentTime = this->get_clock()->now();

    if(publishTf)
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
    
    if(publishOdom)
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

void McuNode::cmdVelCallback(const geometry_msgs::msg::Twist &msg)
{
  float x = msg.linear.x;
  float y = msg.linear.y;
  float w = msg.angular.z;
  //RCLCPP_INFO(this->get_logger(), "/cmd_vel %f %f %f", x, y, w);
  serial.setInverseKinematics(x, y, w);
}

void McuNode::joyCallback(const sensor_msgs::msg::Joy &msg)
{
  // E-Stop
  if(lastEstopButton[0] == 0 && msg.buttons[0] == 1)
  {
    // A = disable software E-Stop
    serial.setEstop(false);
    RCLCPP_INFO(this->get_logger(), "Software E-Stop Disabled");
  }
  lastEstopButton[0] = msg.buttons[0];
  if(lastEstopButton[1] == 0 && msg.buttons[1] == 1)
  {
    // B = enable software E-Stop
    serial.setEstop(true);
    RCLCPP_INFO(this->get_logger(), "Software E-Stop Enabled");
  }
  lastEstopButton[1] = msg.buttons[1];
  // Velocity Change
  if(lastVelocityChangeButton[0] == 0 && msg.buttons[6] == 1)
  {
    // LB = decrease max velocity
    if(maximumVelocity >= 0.2) 
    {
      maximumVelocity -= 0.1;
      RCLCPP_INFO(this->get_logger(), "Maximum velocity decreased to %.1f m/s", maximumVelocity);
    }
  }
  lastVelocityChangeButton[0] = msg.buttons[6];
  if(lastVelocityChangeButton[1] == 0 && msg.buttons[7] == 1)
  {
    // RB = increase max velocity
    if(maximumVelocity < 1.0)
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
  if(msg.axes[1] == 0 && msg.axes[0] == 0)
  {
    // Use D-pad = XY if Left Stick no input
    x = msg.axes[7] * maximumVelocity;
    y = msg.axes[6] * maximumVelocity;
  }
  // LT = w left, RT = w right
  float w = (((msg.axes[4] - 1) / 2) - ((msg.axes[5] - 1) / 2)) * maximumVelocity;
  serial.setInverseKinematics(x, y, w);
}

void McuNode::imuCallback(const sensor_msgs::msg::Imu &msg)
{
  float ax = msg.linear_acceleration.x;
  float ay = msg.linear_acceleration.y;
  float az = msg.linear_acceleration.z;
  float gz = msg.angular_velocity.z;
  serial.setExternalImu(ax, ay, az, gz);
}

McuNode::McuNode() : Node("lgdxrobot2_mcu"), serial(std::bind(&McuNode::serialReadCallback, this, std::placeholders::_1), std::bind(&McuNode::serialDebugCallback, this, std::placeholders::_1, std::placeholders::_2))
{
  auto serial_param_desc = rcl_interfaces::msg::ParameterDescriptor{};
  serial_param_desc.description = "Default serial port name or (Linux only) perform automated search if the port name is unspecified.";
  this->declare_parameter("serial_port", "", serial_param_desc);
  auto control_param_desc = rcl_interfaces::msg::ParameterDescriptor{};
  control_param_desc.description = "Robot control mode, using `joy` for joystick or `cmd_vel` for ROS nav stack.";
  this->declare_parameter("control_mode", "joy", control_param_desc);
  auto odom_param_desc = rcl_interfaces::msg::ParameterDescriptor{};
  odom_param_desc.description = "Publishing odometry information from the chassis.";
  this->declare_parameter("publish_odom", false, odom_param_desc);
  auto tf_param_desc = rcl_interfaces::msg::ParameterDescriptor{};
  tf_param_desc.description = "Publishing tf information from the chassis.";
  this->declare_parameter("publish_tf", false, tf_param_desc);
  auto base_link_frame_param_desc = rcl_interfaces::msg::ParameterDescriptor{};
  base_link_frame_param_desc.description = "Custom name for base_link frame.";
  this->declare_parameter("base_link_frame", "base_link", base_link_frame_param_desc);
  auto use_external_imu_param_desc = rcl_interfaces::msg::ParameterDescriptor{};
  use_external_imu_param_desc.description = "Using external IMU for odometry calcuation.";
  this->declare_parameter("use_external_imu", false, use_external_imu_param_desc);
  auto reset_transform_param_desc = rcl_interfaces::msg::ParameterDescriptor{};
  reset_transform_param_desc.description = "Reset robot transform on start up.";
  this->declare_parameter("reset_transform", false, reset_transform_param_desc);


  serial.start(this->get_parameter("serial_port").as_string());
  std::string controlMode = this->get_parameter("control_mode").as_string();
  if(controlMode.empty() || controlMode.compare(std::string("joy")) == 0)
  {
    RCLCPP_INFO(this->get_logger(), "MCU Node is running in joy control mode");
    joySubscription = this->create_subscription<sensor_msgs::msg::Joy>("joy", rclcpp::QoS(rclcpp::QoS(10)), std::bind(&McuNode::joyCallback, this, std::placeholders::_1));
  }
  else if(controlMode.compare(std::string("cmd_vel")) == 0)
  {
    RCLCPP_INFO(this->get_logger(), "MCU Node is running in cmd_vel mode");
    cmdVelSubscription = this->create_subscription<geometry_msgs::msg::Twist>("cmd_vel", rclcpp::QoS(rclcpp::QoS(10)), std::bind(&McuNode::cmdVelCallback, this, std::placeholders::_1));
  }
  else
  {
    RCLCPP_FATAL(this->get_logger(), "Control mode is invalid, the program is terminaling");
    exit(0);
  }
  if(this->get_parameter("publish_odom").as_bool())
  {
    RCLCPP_INFO(this->get_logger(), "MCU Node will publish odom");
    publishOdom = true;
    baseLinkName = this->get_parameter("base_link_frame").as_string();
    odomPublisher = this->create_publisher<nav_msgs::msg::Odometry>("/lgdxrobot2/odom", rclcpp::QoS(10));
  }
  if(this->get_parameter("publish_tf").as_bool())
  {
    RCLCPP_INFO(this->get_logger(), "MCU Node will publish tf");
    publishTf = true;
    tfBroadcaster = std::make_shared<tf2_ros::TransformBroadcaster>(this);
  }
  if(this->get_parameter("use_external_imu").as_bool())
  {
    RCLCPP_INFO(this->get_logger(), "MCU Node will use external IMU");
    imuSubscription = this->create_subscription<sensor_msgs::msg::Imu>("/lgdxrobot2/ext_imu", rclcpp::QoS(rclcpp::QoS(10)), std::bind(&McuNode::imuCallback, this, std::placeholders::_1));
  }
  if(this->get_parameter("reset_transform").as_bool())
  {
    RCLCPP_INFO(this->get_logger(), "MCU Node will reset transform");
    serial.resetTransform();
  }
}