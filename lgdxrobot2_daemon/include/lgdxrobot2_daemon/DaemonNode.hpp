#ifndef DAEMON_NODE_HPP
#define DAEMON_NODE_HPP

#include <queue>

#include "CloudAdapter.hpp"
#include "SerialPort.hpp"

#include "geometry_msgs/msg/twist.hpp"
#include "geometry_msgs/msg/pose_stamped.hpp"
#include "geometry_msgs/msg/transform_stamped.hpp"
#include "nav2_msgs/action/navigate_through_poses.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include "rclcpp/rclcpp.hpp"
#include "rclcpp_action/rclcpp_action.hpp"
#include "sensor_msgs/msg/imu.hpp"
#include "sensor_msgs/msg/joy.hpp"
#include "tf2_ros/transform_broadcaster.h"
#include "tf2_ros/transform_listener.h"
#include "tf2_ros/buffer.h"

#include "lgdxrobot2_daemon/msg/auto_task.hpp"
#include "lgdxrobot2_daemon/srv/auto_task_next.hpp"
#include "lgdxrobot2_daemon/srv/auto_task_abort.hpp"

class DaemonNode : public rclcpp::Node
{
  private:
    // Cloud
    bool robotIdle = true;
    bool robotStopped = false;
    lgdxrobot2_daemon::msg::AutoTask currentTask;
    std::unique_ptr<CloudAdapter> cloud;
    std::queue<CloudFunctions> cloudErrorQueue;
    rclcpp::TimerBase::SharedPtr cloudRetryTimer;
    rclcpp::TimerBase::SharedPtr cloudExchangeTimer;
    rclcpp::TimerBase::SharedPtr autoTaskPublisherTimer;
    rclcpp::Publisher<lgdxrobot2_daemon::msg::AutoTask>::SharedPtr autoTaskPublisher;
    rclcpp::Service<lgdxrobot2_daemon::srv::AutoTaskNext>::SharedPtr autoTaskNextService;
    rclcpp::Service<lgdxrobot2_daemon::srv::AutoTaskAbort>::SharedPtr autoTaskAbortService;
    rclcpp_action::Client<nav2_msgs::action::NavigateThroughPoses>::SharedPtr navThroughPosesActionClient;
    std::shared_ptr<tf2_ros::TransformListener> tfListener{nullptr};
    std::unique_ptr<tf2_ros::Buffer> tfBuffer;
    RpcRobotDof robotPosition;
    RpcAutoTaskNavProgress navProgress;

    // Serial Port
    std::unique_ptr<SerialPort> serialPort;
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

    void serialUpdate(const RobotData &data);
    void cmdVelCallback(const geometry_msgs::msg::Twist &msg);
    void joyCallback(const sensor_msgs::msg::Joy &msg);
    void imuCallback(const sensor_msgs::msg::Imu &msg);
    void logCallback(const char *msg, int level);

    void navThroughPoses(std::vector<geometry_msgs::msg::PoseStamped> &poses);
    void navThroughPosesGoalResponse(const rclcpp_action::ClientGoalHandle<nav2_msgs::action::NavigateThroughPoses>::SharedPtr &goalHandle);
    void navThroughPosesFeedback(rclcpp_action::ClientGoalHandle<nav2_msgs::action::NavigateThroughPoses>::SharedPtr, 
      const std::shared_ptr<const nav2_msgs::action::NavigateThroughPoses::Feedback> feedback);
    void navThroughPosesResult(const rclcpp_action::ClientGoalHandle<nav2_msgs::action::NavigateThroughPoses>::WrappedResult &result);

    void cloudUpdate(const RpcRespond *respond);
    void cloudRetry();
    void cloudGreet();
    void cloudExchange();
    void cloudAutoTaskNext();
    void cloudAutoTaskAbort();
    
  public:
    DaemonNode();
};

#endif // DAEMON_NODE_HPP