#ifndef SENSORS_HPP
#define SENSORS_HPP

#include "lgdxrobot2.h"
#include "SensorSignals.hpp"

#include "geometry_msgs/msg/twist.hpp"
#include "lgdxrobot2_msgs/msg/system.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/imu.hpp"
#include "sensor_msgs/msg/joint_state.hpp"
#include "sensor_msgs/msg/joy.hpp"
#include "sensor_msgs/msg/magnetic_field.hpp"
#include "tf2_ros/transform_broadcaster.h"

class Sensors
{
  private:
    rclcpp::Clock::SharedPtr _clock;
    rclcpp::Logger _logger;

    rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr cmdVelSubscription;
    rclcpp::Subscription<sensor_msgs::msg::Joy>::SharedPtr joySubscription;

    std::string baseLinkName;
    rclcpp::Publisher<sensor_msgs::msg::Imu>::SharedPtr imuPublisher;
    rclcpp::Publisher<sensor_msgs::msg::MagneticField>::SharedPtr magneticFieldPublisher;
    rclcpp::Publisher<lgdxrobot2_msgs::msg::System>::SharedPtr systemPublisher;
    std::shared_ptr<tf2_ros::TransformBroadcaster> tfBroadcaster;
    rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr odomPublisher;
    rclcpp::Publisher<sensor_msgs::msg::JointState>::SharedPtr jointStatePublisher;

    // Joy
    float maximumVelocity = 0.1; // m/s
    int lastVelocityChangeButton[2] = {0}; // 0: LB, 1: RB
    int lastEstopButton[2] = {0}; // 0: A, 1: B
    // JointState
    double motorsPosition[4] = {0};
    std::shared_ptr<SensorSignals> sensorSignals;

    void CmdVelCallback(const geometry_msgs::msg::Twist &msg);
    void JoyCallback(const sensor_msgs::msg::Joy &msg);

  public:
    Sensors(rclcpp::Node::SharedPtr node, std::shared_ptr<SensorSignals> sensorSignalsPtr);
    void Publish(const McuData &mcuData);
};

#endif // SENSORS_HPP