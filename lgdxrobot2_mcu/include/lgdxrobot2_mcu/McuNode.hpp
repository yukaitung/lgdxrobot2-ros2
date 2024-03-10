#ifndef MCUNODE_HPP
#define MCUNODE_HPP

#include "rclcpp/rclcpp.hpp"
#include "geometry_msgs/msg/twist.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include "sensor_msgs/msg/joy.hpp"
#include "sensor_msgs/msg/imu.hpp"

// Odom
#include "tf2_ros/transform_broadcaster.h"

#include "SerialPort.hpp"

class McuNode : public rclcpp::Node
{
  private:
    // Object
    SerialPort serial;
    rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr cmdVelSubscription;
    rclcpp::Subscription<sensor_msgs::msg::Joy>::SharedPtr joySubscription;
    rclcpp::Subscription<sensor_msgs::msg::Imu>::SharedPtr imuSubscription;

    // Odom
    rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr odomPublisher;
    std::shared_ptr<tf2_ros::TransformBroadcaster> tfBroadcaster;
    std::string baseLinkName;

    // Joy
    float maximumVelocity = 0.1; // m/s
    int lastVelocityChangeButton[2] = {0}; // 0: LB, 1: RB
    int lastEstopButton[2] = {0}; // 0: A, 1: B

    // Odom
    bool publishOdom = false;
    bool publishTf = false;

    void serialDebugCallback(const std::string &msg, int level);
    void serialReadCallback(const McuData &data);
    void cmdVelCallback(const geometry_msgs::msg::Twist &msg);
    void joyCallback(const sensor_msgs::msg::Joy &msg);
    void imuCallback(const sensor_msgs::msg::Imu &msg);
    
  public:
    McuNode();
};

#endif // MCUNODE_HPP
