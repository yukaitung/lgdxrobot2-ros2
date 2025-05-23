#ifndef DAEMON_NODE_HPP
#define DAEMON_NODE_HPP

#include <queue>

#include "CloudAdapter.hpp"
#include "SerialPort.hpp"
#include "RobotStatus.hpp"
#include "Navigation.hpp"

#include "geometry_msgs/msg/pose_stamped.hpp"
#include "geometry_msgs/msg/transform_stamped.hpp"
#include "geometry_msgs/msg/twist.hpp"
#include "lgdxrobot2_daemon/msg/auto_task.hpp"
#include "lgdxrobot2_daemon/srv/auto_task_abort.hpp"
#include "lgdxrobot2_daemon/srv/auto_task_next.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include "nav2_msgs/action/navigate_through_poses.hpp"
#include "rclcpp_action/rclcpp_action.hpp"
#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/imu.hpp"
#include "sensor_msgs/msg/joy.hpp"
#include "std_msgs/msg/bool.hpp"
#include "tf2_ros/buffer.h"
#include "tf2_ros/transform_broadcaster.h"
#include "tf2_ros/transform_listener.h"

class DaemonNode : public rclcpp::Node
{
  private:
    // Cloud
    std::unique_ptr<CloudAdapter> cloud;
    std::queue<CloudFunctions> cloudErrorQueue;
    rclcpp::TimerBase::SharedPtr cloudRetryTimer;
    rclcpp::TimerBase::SharedPtr cloudExchangeTimer;
    rclcpp::TimerBase::SharedPtr autoTaskPublisherTimer;
    rclcpp::Publisher<std_msgs::msg::Bool>::SharedPtr crtitcalStatusPublisher;
    rclcpp::Publisher<lgdxrobot2_daemon::msg::AutoTask>::SharedPtr autoTaskPublisher;
    rclcpp::Service<lgdxrobot2_daemon::srv::AutoTaskNext>::SharedPtr autoTaskNextService;
    rclcpp::Service<lgdxrobot2_daemon::srv::AutoTaskAbort>::SharedPtr autoTaskAbortService;
    rclcpp_action::Client<nav2_msgs::action::NavigateThroughPoses>::SharedPtr navThroughPosesActionClient;
    std::shared_ptr<tf2_ros::TransformListener> tfListener{nullptr};
    std::unique_ptr<tf2_ros::Buffer> tfBuffer;
    RobotClientsDof robotPosition;
    std::shared_ptr<Navigation> navigation;
    bool IsRealtimeExchange = false;

    // Cloud Robot Status
    std::shared_ptr<RobotStatus> robotStatus;
    lgdxrobot2_daemon::msg::AutoTask currentTask;
    RobotClientsRobotCriticalStatus criticalStatus;
    RobotClientsRobotCommands currentCommands;
    RobotClientsAbortReason lastAbortReason;

    // Serial Port
    std::unique_ptr<SerialPort> serialPort;
    rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr cmdVelSubscription;
    rclcpp::Subscription<sensor_msgs::msg::Joy>::SharedPtr joySubscription;
    rclcpp::Subscription<sensor_msgs::msg::Imu>::SharedPtr imuSubscription;
    std::string lastMcuSerialNumber;

    // Odom
    std::string baseLinkName;
    rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr odomPublisher;
    std::shared_ptr<tf2_ros::TransformBroadcaster> tfBroadcaster;

    // Joy
    float maximumVelocity = 0.1; // m/s
    int lastVelocityChangeButton[2] = {0}; // 0: LB, 1: RB
    int lastEstopButton[2] = {0}; // 0: A, 1: B

    void serialUpdate(const RobotData &data);
    void cmdVelCallback(const geometry_msgs::msg::Twist &msg);
    void joyCallback(const sensor_msgs::msg::Joy &msg);
    void imuCallback(const sensor_msgs::msg::Imu &msg);
    void logCallback(const char *msg, int level);

    void cloudUpdate(const RobotClientsRespond *respond);
    void cloudRetry();
    void cloudGreet(std::string mcuSerialNumber);
    void cloudExchange();
    void cloudAutoTaskNext();
    void cloudAutoTaskAbort(RobotClientsAbortReason reason);

  public:
    DaemonNode();
    void shutdown();
};

#endif // DAEMON_NODE_HPP