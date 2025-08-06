#include "lgdxrobot2_agent/Map.hpp"

#include <ctime>

Map::Map(rclcpp::Node::SharedPtr node) : logger_(node->get_logger())
{
  saveMapTimer = node->create_wall_timer(std::chrono::seconds(kWaitSecond), std::bind(&Map::OnTimeout, this));
  saveMapTimer->cancel();
  saveMapClient = node->create_client<nav2_msgs::srv::SaveMap>("map_saver/save_map");
}

void Map::OnResult(rclcpp::Client<nav2_msgs::srv::SaveMap>::SharedFuture future)
{
  auto status = future.get();
  if (status->result == true) 
  {
    RCLCPP_INFO(logger_, "The map is saved.");
  } 
  else 
  {
    RCLCPP_ERROR(logger_, "The map is not saved.");
  }
  running = false;
}

void Map::OnTimeout()
{
  saveMapTimer->cancel();
  RCLCPP_ERROR(logger_, "The map is not saved.");
  running = false;
}

void Map::Save()
{
  if (running)
  {
    return;
  }

  auto request = std::make_shared<nav2_msgs::srv::SaveMap::Request>();
  request->map_topic = "map";
  request->map_url = std::to_string(std::time(nullptr));
  request->image_format = "pgm";
  request->map_mode = "trinary";
  request->free_thresh = 0.25;
  request->occupied_thresh = 0.65;
  saveMapClient->async_send_request(request, std::bind(&Map::OnResult, this, std::placeholders::_1));
  saveMapTimer->reset();
  running = true;
}