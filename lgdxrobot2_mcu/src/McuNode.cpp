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
  if(publishOdom)
  {
    // Calculate the forward kinematic (putting into mcu later...)
    double xDelta = (data.measuredWheelVelocity[0] + data.measuredWheelVelocity[1] + data.measuredWheelVelocity[2] + data.measuredWheelVelocity[3]) * 0.009375 * 0.02; // r / 4 // 20ms
    double yDelta = (-data.measuredWheelVelocity[0] + data.measuredWheelVelocity[1] + data.measuredWheelVelocity[2] - data.measuredWheelVelocity[3]) * 0.009375 * 0.02; // r / 4
    double wDelta = (-data.measuredWheelVelocity[0] + data.measuredWheelVelocity[1] - data.measuredWheelVelocity[2] + data.measuredWheelVelocity[3]) * 0.009375 * 0.02; // r / 4

    xOdom += xDelta;
    yOdom += yDelta;
    wOdom += wDelta;

    tf2::Quaternion quaternion;
    quaternion.setRPY(0, 0, wOdom);
    geometry_msgs::msg::Quaternion odomQuaternion = tf2::toMsg(quaternion);
    rclcpp::Time currentTime = this->get_clock()->now();

    // Publist tf
    geometry_msgs::msg::TransformStamped odomTf;
    odomTf.header.stamp = currentTime;
    odomTf.header.frame_id = "odom";
    odomTf.child_frame_id = "base_link";
    odomTf.transform.translation.x = xOdom;
    odomTf.transform.translation.y = yOdom;
    odomTf.transform.translation.z = 0.0;
    odomTf.transform.rotation = odomQuaternion;
    tfBroadcaster->sendTransform(odomTf);

    // Publish odom
    nav_msgs::msg::Odometry odometry;
    odometry.header.stamp = currentTime;
    odometry.header.frame_id = "odom";
    odometry.pose.pose.position.x = xOdom;
    odometry.pose.pose.position.y = yOdom;
    odometry.pose.pose.position.z = 0.0;
    odometry.pose.pose.orientation = odomQuaternion;
    odometry.child_frame_id = "base_link";
    odometry.twist.twist.linear.x = xDelta;
    odometry.twist.twist.linear.y = yDelta;
    odometry.twist.twist.angular.z = wDelta;
    odomPublisher->publish(odometry);
  }
  
}

void McuNode::cmdVelCallback(const geometry_msgs::msg::Twist &msg)
{
  float x = msg.linear.x;
  float y = msg.linear.y;
  float w = msg.angular.z;
  RCLCPP_DEBUG(this->get_logger(), "Received IK (%f, %f, %f)", x, y, w);
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

McuNode::McuNode() : Node("mcu_node"), serial(std::bind(&McuNode::serialReadCallback, this, std::placeholders::_1), std::bind(&McuNode::serialDebugCallback, this, std::placeholders::_1, std::placeholders::_2))
{
  auto serial_param_desc = rcl_interfaces::msg::ParameterDescriptor{};
  serial_param_desc.description = "Default serial port name or automated search if unspecified.";
  this->declare_parameter("serial_port", "", serial_param_desc);
  auto control_param_desc = rcl_interfaces::msg::ParameterDescriptor{};
  control_param_desc.description = "Robot control mode, using `joy` / unspecified for joystick or `cmd_vel` for ROS nav stack.";
  this->declare_parameter("control_mode", "joy", control_param_desc);
  auto odom_param_desc = rcl_interfaces::msg::ParameterDescriptor{};
  odom_param_desc.description = "Publishing odometry information and tf using the forward kinematic from the chassis.";
  this->declare_parameter("publish_odom", true, control_param_desc);
  auto base_link_frame_param_desc = rcl_interfaces::msg::ParameterDescriptor{};
  base_link_frame_param_desc.description = "Custom name of base link frame.";
  this->declare_parameter("base_link_frame", "base_link", control_param_desc);


  serial.start(this->get_parameter("serial_port").as_string());
  std::string controlMode = this->get_parameter("control_mode").as_string();
  if(controlMode.empty() || controlMode.compare(std::string("joy")) == 0)
  {
    RCLCPP_INFO(this->get_logger(), "MCU Node is running in joy control mode");
    joySubscription = this->create_subscription<sensor_msgs::msg::Joy>("joy", 10, std::bind(&McuNode::joyCallback, this, std::placeholders::_1));
  }
  else if(controlMode.compare(std::string("cmd_vel")) == 0)
  {
    RCLCPP_INFO(this->get_logger(), "MCU Node is running in cmd_vel mode");
    cmdVelSubscription = this->create_subscription<geometry_msgs::msg::Twist>("cmd_vel", 10, std::bind(&McuNode::cmdVelCallback, this, std::placeholders::_1));
  }
  else
  {
    RCLCPP_FATAL(this->get_logger(), "Control mode is invalid, the program isterminaling");
    exit(0);
  }
  if(this->get_parameter("publish_odom").as_bool())
  {
    publishOdom = true;
    baseLinkName = this->get_parameter("base_link_frame").as_string();
    tfBroadcaster = std::make_shared<tf2_ros::TransformBroadcaster>(this);
    odomPublisher = this->create_publisher<nav_msgs::msg::Odometry>("odom", 50);
  }
}