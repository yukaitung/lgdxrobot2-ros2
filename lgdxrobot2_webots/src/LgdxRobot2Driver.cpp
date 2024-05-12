#include "LgdxRobot2Driver.hpp"

#include <webots/robot.h>
#include <webots/motor.h>
#include <webots/position_sensor.h>

// Odom
#include "geometry_msgs/msg/transform_stamped.hpp"
#include "tf2_geometry_msgs/tf2_geometry_msgs.hpp"

namespace LgdxRobot2 {
void LgdxRobot2Driver::cmdVelCallback(const geometry_msgs::msg::Twist &msg) 
{
  double x = msg.linear.x;
  double y = msg.linear.y;
  double w = msg.angular.z;
  wheelsVelocity[0] = (1 / WHEEL_RADIUS) * (x - y - (CHASSIS_LX + CHASSIS_LY) * w);
  wheelsVelocity[1] = (1 / WHEEL_RADIUS) * (x + y + (CHASSIS_LX + CHASSIS_LY) * w);
  wheelsVelocity[2] = (1 / WHEEL_RADIUS) * (x + y - (CHASSIS_LX + CHASSIS_LY) * w);
  wheelsVelocity[3] = (1 / WHEEL_RADIUS) * (x - y + (CHASSIS_LX + CHASSIS_LY) * w);
}

void LgdxRobot2Driver::init(webots_ros2_driver::WebotsNode *node, std::unordered_map<std::string, std::string> &parameters) 
{
  rosNode = node;

  char wheelsNames[4][7] = {"wheel1", "wheel2", "wheel3", "wheel4"};
  for (int i = 0; i < 4; i++) 
  {
    wheels[i] = wb_robot_get_device(wheelsNames[i]);
    wb_motor_set_position(wheels[i], INFINITY);
    wb_motor_set_velocity(wheels[i], 0.0);
  }

  char positionSensorsName[4][9] = {"encoder1", "encoder2", "encoder3", "encoder4"};
  for (int i = 0; i < 4; i++) 
  {
    positionSensors[i] = wb_robot_get_device(positionSensorsName[i]);
    wb_position_sensor_enable(positionSensors[i], wb_robot_get_basic_time_step());
  }

  cmdVelSubscription = node->create_subscription<geometry_msgs::msg::Twist>(
    "cmd_vel", 
    rclcpp::SensorDataQoS().reliable(),
    std::bind(&LgdxRobot2Driver::cmdVelCallback, this, std::placeholders::_1)
  );
  odomPublisher = node->create_publisher<nav_msgs::msg::Odometry>("/odom", rclcpp::SensorDataQoS().reliable());
  tfBroadcaster = std::make_shared<tf2_ros::TransformBroadcaster>(node);
}

void LgdxRobot2Driver::step() 
{
  for (int i = 0; i < 4; i++) 
  {
    wb_motor_set_velocity(wheels[i], wheelsVelocity[i]);
  }
  
  double currentSimTime = wb_robot_get_time();
  double timeElapsed = currentSimTime - lastSimTime;
  double motorPosition[4] = {0};
  double motorPositionChange[4] = {0};
  double motorForwardKinematic[3] = {0};
  
  for (int i = 0; i < 4; i++) 
  {
    motorPosition[i] = wb_position_sensor_get_value(positionSensors[i]);
    motorPositionChange[i] = motorPosition[i] - motorLastPosition[i];
  }
  motorForwardKinematic[0] = ((motorPositionChange[0] + motorPositionChange[1] + motorPositionChange[2] + motorPositionChange[3]) * (WHEEL_RADIUS / 4)) / timeElapsed;
  motorForwardKinematic[1] = ((-motorPositionChange[0] + motorPositionChange[1] + motorPositionChange[2] - motorPositionChange[3]) * (WHEEL_RADIUS / 4)) / timeElapsed; // Variation in formula
  motorForwardKinematic[2] = ((-motorPositionChange[0] + motorPositionChange[1] - motorPositionChange[2] + motorPositionChange[3]) * ((WHEEL_RADIUS) * 2 / (M_PI * (CHASSIS_LX + CHASSIS_LY)))) / timeElapsed;
  robotTransform[0] += (motorForwardKinematic[0] * cos(robotTransform[2]) - motorForwardKinematic[1] * sin(robotTransform[2])) * timeElapsed;
	robotTransform[1] += (motorForwardKinematic[0] * sin(robotTransform[2]) + motorForwardKinematic[1] * cos(robotTransform[2])) * timeElapsed;
  robotTransform[2] += motorForwardKinematic[2] * timeElapsed;

  tf2::Quaternion quaternion;
  quaternion.setRPY(0, 0, robotTransform[2]);
  geometry_msgs::msg::Quaternion odomQuaternion = tf2::toMsg(quaternion);
  rclcpp::Time currentTime = rosNode->get_clock()->now();
  /*
  geometry_msgs::msg::TransformStamped odomTf;
  odomTf.header.stamp = currentTime;
  odomTf.header.frame_id = "odom";
  odomTf.child_frame_id = "base_link";
  odomTf.transform.translation.x = robotTransform[0];
  odomTf.transform.translation.y = robotTransform[1];
  odomTf.transform.translation.z = 0.0;
  odomTf.transform.rotation = odomQuaternion;
  if(tfBroadcaster)
    tfBroadcaster->sendTransform(odomTf);
  */
  nav_msgs::msg::Odometry odometry;
  odometry.header.stamp = currentTime;
  odometry.header.frame_id = "odom";
  odometry.pose.pose.position.x = robotTransform[0];
  odometry.pose.pose.position.y = robotTransform[1];
  odometry.pose.pose.position.z = 0.0;
  odometry.pose.pose.orientation = odomQuaternion;
  odometry.child_frame_id = "base_link";
  odometry.twist.twist.linear.x = motorForwardKinematic[0];
  odometry.twist.twist.linear.y = motorForwardKinematic[1];
  odometry.twist.twist.angular.z = motorForwardKinematic[2];
  if(odomPublisher)
    odomPublisher->publish(odometry);

  for (int i = 0; i < 4; i++) 
  {
    motorLastPosition[i] = motorPosition[i];
  }
  lastSimTime = currentSimTime;
}
}

#include "pluginlib/class_list_macros.hpp"
PLUGINLIB_EXPORT_CLASS(LgdxRobot2::LgdxRobot2Driver, webots_ros2_driver::PluginInterface)