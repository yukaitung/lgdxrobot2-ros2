#ifndef SENSORS_HPP
#define SENSORS_HPP

#include "Structs/RobotData.hpp"

#include "geometry_msgs/msg/twist.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/imu.hpp"
#include "sensor_msgs/msg/joy.hpp"
#include "tf2_ros/transform_broadcaster.h"

class Sensors
{
  private:
    rclcpp::Clock::SharedPtr clock_;
    rclcpp::Logger logger_;

    rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr cmdVelSubscription;
    rclcpp::Subscription<sensor_msgs::msg::Joy>::SharedPtr joySubscription;
    rclcpp::Subscription<sensor_msgs::msg::Imu>::SharedPtr imuSubscription;

    // Odom
    std::string baseLinkName;
    rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr odomPublisher;
    std::shared_ptr<tf2_ros::TransformBroadcaster> tfBroadcaster;

    // Joy
    float maximumVelocity = 0.1; // m/s
    int lastVelocityChangeButton[2] = {0}; // 0: LB, 1: RB
    int lastEstopButton[2] = {0}; // 0: A, 1: B

    bool needPublishOdom = false;

    void CmdVelCallback(const geometry_msgs::msg::Twist &msg);
    void JoyCallback(const sensor_msgs::msg::Joy &msg);
    void ImuCallback(const sensor_msgs::msg::Imu &msg);

  public:
    Sensors(rclcpp::Node::SharedPtr node);
    void PublishOdom(const RobotData &robotData);
};

#endif // SENSORS_HPP