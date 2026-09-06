#include "lgdxrobot2_agent/sensors.hpp"

#include "tf2_geometry_msgs/tf2_geometry_msgs.hpp"

Sensors::Sensors(rclcpp::Node::SharedPtr node, std::shared_ptr<SensorSignals> sensor_signals_ptr) :
  clock_(node->get_clock()),
  logger_(node->get_logger())
{
  sensor_signals_ = sensor_signals_ptr;

  // Subscriber
  nav2_subscription_ = node->create_subscription<geometry_msgs::msg::TwistStamped>("cmd_vel", 
      rclcpp::SensorDataQoS().reliable(),
      std::bind(&Sensors::Nav2Callback, this, std::placeholders::_1));
  if (node->get_parameter("use_keyboard").as_bool())
  {
    keyboard_subscription_ = node->create_subscription<geometry_msgs::msg::Twist>("keyboard", 
      rclcpp::SensorDataQoS().reliable(),
      std::bind(&Sensors::KeyboardCallback, this, std::placeholders::_1));
  }
  if (node->get_parameter("use_joy").as_bool())
  {
    joy_subscription = node->create_subscription<sensor_msgs::msg::Joy>("joy",
      rclcpp::SensorDataQoS().reliable(),
      std::bind(&Sensors::JoyCallback, this, std::placeholders::_1));
  }
  software_emergency_stopSubscription_ = node->create_subscription<std_msgs::msg::Bool>("agent/software_emergency_stop", 
    rclcpp::SensorDataQoS().reliable(),
    [this](const std_msgs::msg::Bool::SharedPtr msg) {
      sensor_signals_->set_estop(msg->data);
    });

  // Publisher
  imu_publisher_ = node->create_publisher<sensor_msgs::msg::Imu>("agent/imu", 
    rclcpp::SensorDataQoS().reliable());
  magnetic_field_publisher_ = node->create_publisher<sensor_msgs::msg::MagneticField>("agent/mag", 
    rclcpp::SensorDataQoS().reliable());
  system_publisher_ = node->create_publisher<lgdxrobot2_msgs::msg::System>("agent/system", 
    rclcpp::SensorDataQoS().reliable());
  odom_publisher_ = node->create_publisher<nav_msgs::msg::Odometry>("agent/odom",
    rclcpp::SensorDataQoS().reliable());
  joint_state_publisher_ = node->create_publisher<sensor_msgs::msg::JointState>("joint_states", 
    rclcpp::SensorDataQoS().reliable());
  base_link_name_ = node->get_parameter("base_link_name").as_string();
  if (node->get_parameter("publish_tf").as_bool())
  {
    tf_broadcaster_ = std::make_shared<tf2_ros::TransformBroadcaster>(node);
  }
}

float Sensors::GetBatteryPercentage(float voltage)
{
  if (voltage < kBatteryLowVoltageThreshold) 
  {
    return 0.0;
  }
  if (voltage > kBatteryHighVoltageThreshold) 
  {
    return 100.0;
  }
  return (voltage - kBatteryLowVoltageThreshold) / (kBatteryHighVoltageThreshold - kBatteryLowVoltageThreshold);
}

void Sensors::KeyboardCallback(const geometry_msgs::msg::Twist &msg)
{
  float x = msg.linear.x;
  float y = msg.linear.y;
  float w = msg.angular.z;
  //RCLCPP_INFO(logger_, "/keyboard %f %f %f", x, y, w);
  sensor_signals_->set_inverse_kinematics(x, y, w);
}

void Sensors::Nav2Callback(const geometry_msgs::msg::TwistStamped &msg)
{
  float x = msg.twist.linear.x;
  float y = msg.twist.linear.y;
  float w = msg.twist.angular.z;
  //RCLCPP_INFO(logger_, "/cmd_vel %f %f %f", x, y, w);
  sensor_signals_->set_inverse_kinematics(x, y, w);
}

void Sensors::JoyCallback(const sensor_msgs::msg::Joy &msg)
{
  // E-Stop
  if (last_estop_button_[0] == 0 && msg.buttons[0] == 1)
  {
    // A = disable software E-Stop
    sensor_signals_->set_estop(false);
    RCLCPP_INFO(logger_, "Software E-Stop Disabled");
  }
  last_estop_button_[0] = msg.buttons[0];
  if (last_estop_button_[1] == 0 && msg.buttons[1] == 1)
  {
    // B = enable software E-Stop
    sensor_signals_->set_estop(true);
    RCLCPP_INFO(logger_, "Software E-Stop Enabled");
  }
  last_estop_button_[1] = msg.buttons[1];
  // Velocity Change
  if (last_velocity_change_button_[0] == 0 && msg.buttons[6] == 1)
  {
    // LB = decrease max velocity
    if (maximum_velocity_ >= 0.2)
    {
      maximum_velocity_ -= 0.1;
      RCLCPP_INFO(logger_, "Maximum velocity decreased to %.1f m/s", maximum_velocity_);
    }
  }
  last_velocity_change_button_[0] = msg.buttons[6];
  if (last_velocity_change_button_[1] == 0 && msg.buttons[7] == 1)
  {
    // RB = increase max velocity
    if (maximum_velocity_ < 1.0)
    {
      maximum_velocity_ += 0.1;
      RCLCPP_INFO(logger_, "Maximum velocity increased to %.1f m/s", maximum_velocity_);
    }
  }
  last_velocity_change_button_[1] = msg.buttons[7];
  // Control IK
  // Left Stick = XY
  float x = msg.axes[1] * maximum_velocity_;
  float y = msg.axes[0] * maximum_velocity_;
  if (msg.axes[1] == 0 && msg.axes[0] == 0)
  {
    // Use D-pad = XY if Left Stick no input
    x = msg.axes[7] * maximum_velocity_;
    y = msg.axes[6] * maximum_velocity_;
  }
  // LT = w left, RT = w right
  float w = ((msg.axes[4] - 1) - (msg.axes[5] - 1)) * maximum_velocity_ * 2;
  sensor_signals_->set_inverse_kinematics(x, y, w);
}

void Sensors::Publish(const McuData& mcu_data)
{
  if (imu_publisher_ != nullptr)
  {
    sensor_msgs::msg::Imu imu;
    imu.header.stamp = clock_->now();
    imu.orientation_covariance[0] = -1;
    imu.angular_velocity.x = mcu_data.imu.gyroscope.x;
    imu.angular_velocity.y = mcu_data.imu.gyroscope.y;
    imu.angular_velocity.z = mcu_data.imu.gyroscope.z;
    imu.angular_velocity_covariance[0] = mcu_data.imu.accelerometer_covariance.x;
    imu.angular_velocity_covariance[4] = mcu_data.imu.accelerometer_covariance.y;
    imu.angular_velocity_covariance[8] = mcu_data.imu.accelerometer_covariance.z;
    imu.linear_acceleration.x = mcu_data.imu.accelerometer.x;
    imu.linear_acceleration.y = mcu_data.imu.accelerometer.y;
    imu.linear_acceleration.z = mcu_data.imu.accelerometer.z;
    imu.linear_acceleration_covariance[0] = mcu_data.imu.accelerometer_covariance.x;
    imu.linear_acceleration_covariance[4] = mcu_data.imu.accelerometer_covariance.y;
    imu.linear_acceleration_covariance[8] = mcu_data.imu.accelerometer_covariance.z;
    imu_publisher_->publish(imu);
  }

  if (magnetic_field_publisher_ != nullptr)
  {
    sensor_msgs::msg::MagneticField magnetic_field;
    magnetic_field.header.stamp = clock_->now();
    magnetic_field.magnetic_field.x = mcu_data.imu.magnetometer.x / 1000000;
    magnetic_field.magnetic_field.y = mcu_data.imu.magnetometer.y / 1000000;
    magnetic_field.magnetic_field.z = mcu_data.imu.magnetometer.z / 1000000;
    magnetic_field.magnetic_field_covariance[0] = mcu_data.imu.magnetometer_covariance.x;
    magnetic_field.magnetic_field_covariance[4] = mcu_data.imu.magnetometer_covariance.y;
    magnetic_field.magnetic_field_covariance[8] = mcu_data.imu.magnetometer_covariance.z;
    magnetic_field_publisher_->publish(magnetic_field);
  }

  if (system_publisher_ != nullptr)
  {
    lgdxrobot2_msgs::msg::System system;
    system.battery1.voltage = mcu_data.battery1.voltage;
    system.battery1.current = mcu_data.battery1.current;
    system.battery1.percentage = GetBatteryPercentage(mcu_data.battery1.voltage);
    system.battery2.voltage = mcu_data.battery2.voltage;
    system.battery2.current = mcu_data.battery2.current;
    system.battery2.percentage = GetBatteryPercentage(mcu_data.battery2.voltage);
    system.software_emergency_stop_enabled = mcu_data.software_emergency_stop_enabled;
    system.hardware_emergency_stop_enabled = mcu_data.hardware_emergency_stop_enabled;
    system.bettery_low_emergency_stop_enabled = mcu_data.bettery_low_emergency_stop_enabled;
    system_publisher_->publish(system);
  }
  
  tf2::Quaternion quaternion;
  quaternion.setRPY(0, 0, mcu_data.transform.rotation);
  geometry_msgs::msg::Quaternion odom_quaternion = tf2::toMsg(quaternion);
  if (tf_broadcaster_ != nullptr)
  {
    geometry_msgs::msg::TransformStamped odom_tf;
    odom_tf.header.stamp = clock_->now();
    odom_tf.header.frame_id = "odom";
    odom_tf.child_frame_id = base_link_name_;
    odom_tf.transform.translation.x = mcu_data.transform.x;
    odom_tf.transform.translation.y = mcu_data.transform.y;
    odom_tf.transform.translation.z = 0.0;
    odom_tf.transform.rotation = odom_quaternion;
    tf_broadcaster_->sendTransform(odom_tf);
  }
  if (odom_publisher_ != nullptr)
  {
    nav_msgs::msg::Odometry odometry;
    odometry.header.stamp = clock_->now();
    odometry.header.frame_id = "odom";
    odometry.pose.pose.position.x = mcu_data.transform.x;
    odometry.pose.pose.position.y = mcu_data.transform.y;
    odometry.pose.pose.position.z = 0.0;
    odometry.pose.pose.orientation = odom_quaternion;
    odometry.child_frame_id = base_link_name_;
    odometry.twist.twist.linear.x = mcu_data.forward_kinematic.x;
    odometry.twist.twist.linear.y = mcu_data.forward_kinematic.y;
    odometry.twist.twist.angular.z = mcu_data.forward_kinematic.rotation;
    odom_publisher_->publish(odometry);
  }

  if (joint_state_publisher_ != nullptr)
  {
    for (int i = 0; i < 4; i++)
    {
      motors_position_[i] += mcu_data.motors_actual_velocity[i] * (mcu_data.response_time / 1000.0);
    }
    sensor_msgs::msg::JointState jointState;
    jointState.header.stamp = clock_->now();
    jointState.name = {"wheel1_link_joint", "wheel2_link_joint", "wheel3_link_joint", "wheel4_link_joint"};
    jointState.position = {motors_position_[0], motors_position_[1], motors_position_[2], motors_position_[3]};
    jointState.velocity = {mcu_data.motors_actual_velocity[0], mcu_data.motors_actual_velocity[1], mcu_data.motors_actual_velocity[2], mcu_data.motors_actual_velocity[3]};
    joint_state_publisher_->publish(jointState);
  }
}
