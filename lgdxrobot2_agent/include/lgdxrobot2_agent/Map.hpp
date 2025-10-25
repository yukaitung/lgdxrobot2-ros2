#ifndef MAP_HPP
#define MAP_HPP

#include "nav2_msgs/srv/save_map.hpp"
#include "rclcpp/rclcpp.hpp"

class Map
{
  private:
    rclcpp::Logger logger_;
    rclcpp::TimerBase::SharedPtr saveMapTimer;
    rclcpp::Client<nav2_msgs::srv::SaveMap>::SharedPtr saveMapClient;

    bool running = false;
    bool kWaitSecond = 5;
  
    void OnResult(rclcpp::Client<nav2_msgs::srv::SaveMap>::SharedFuture future);
    void OnTimeout();

  public:
    Map(rclcpp::Node::SharedPtr node);
    void Save();
};

#endif // MAP_HPP