#ifndef SENSORS_HPP
#define SENSORS_HPP

#include "lgdxrobot2.h"
#include "sensor_signals.hpp"

#include "geometry_msgs/msg/twist.hpp"
#include "lgdxrobot2_msgs/msg/system.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/imu.hpp"
#include "sensor_msgs/msg/joint_state.hpp"
#include "sensor_msgs/msg/joy.hpp"
#include "sensor_msgs/msg/magnetic_field.hpp"
#include "std_msgs/msg/bool.hpp"
#include "tf2_ros/transform_broadcaster.hpp"

class Sensors
{
  public:
    Sensors(rclcpp::Node::SharedPtr node, std::shared_ptr<SensorSignals> sensor_signals_ptr);
    void Publish(const McuData &mcu_data);

  private:
    rclcpp::Clock::SharedPtr clock_;
    rclcpp::Logger logger_;

    rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr keyboard_subscription_;
    rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr nav2_subscription_;
    rclcpp::Subscription<sensor_msgs::msg::Joy>::SharedPtr joy_subscription;
    rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr software_emergency_stopSubscription_;

    std::string base_link_name_;
    rclcpp::Publisher<sensor_msgs::msg::Imu>::SharedPtr imu_publisher_;
    rclcpp::Publisher<sensor_msgs::msg::MagneticField>::SharedPtr magnetic_field_publisher_;
    rclcpp::Publisher<lgdxrobot2_msgs::msg::System>::SharedPtr system_publisher_;
    std::shared_ptr<tf2_ros::TransformBroadcaster> tf_broadcaster_;
    rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr odom_publisher_;
    rclcpp::Publisher<sensor_msgs::msg::JointState>::SharedPtr joint_state_publisher_;

    // Batteries
    float kBatteryLowVoltageThreshold = 12.0;
    float kBatteryHighVoltageThreshold = 16.8;

    // Joy
    float maximum_velocity_ = 0.1; // m/s
    int last_velocity_change_button_[2] = {0}; // 0: LB, 1: RB
    int last_estop_button_[2] = {0}; // 0: A, 1: B
    // JointState
    double motors_position_[4] = {0};
    std::shared_ptr<SensorSignals> sensor_signals_;

    float GetBatteryPercentage(float voltage);
    void KeyboardCallback(const geometry_msgs::msg::Twist &msg);
    void Nav2Callback(const geometry_msgs::msg::Twist &msg);
    void JoyCallback(const sensor_msgs::msg::Joy &msg);
};

#endif // SENSORS_HPP