#ifndef MCUNODE_HPP
#define MCUNODE_HPP

#include "rclcpp/rclcpp.hpp"
#include "geometry_msgs/msg/twist.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include "sensor_msgs/msg/joy.hpp"

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
    double xOdom = 0;
    double yOdom = 0;
    double wOdom = 0;

    void serialDebugCallback(const std::string &msg, int level);
    void serialReadCallback(const McuData &data);
    void cmdVelCallback(const geometry_msgs::msg::Twist &msg);
    void joyCallback(const sensor_msgs::msg::Joy &msg);
    
  public:
    McuNode();
};

#endif // MCUNODE_HPP
