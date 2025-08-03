#ifndef SLAM_CONTROLLER_SIGNALS_HPP
#define SLAM_CONTROLLER_SIGNALS_HPP

#include <boost/signals2/signal.hpp>

#include "proto/RobotClientsService.grpc.pb.h"

#include "geometry_msgs/msg/pose_stamped.hpp"

struct SlamControllerSignals
{
  boost::signals2::signal<void(RobotClientsRealtimeNavResults,
    RobotClientsExchange &)> SlamExchange2;
  boost::signals2::signal<void(RobotClientsRealtimeNavResults,
    RobotClientsExchange &, 
    RobotClientsMapData &)> SlamExchange3;
  boost::signals2::signal<void(std::vector<geometry_msgs::msg::PoseStamped> &)> NavigationStart;
  boost::signals2::signal<void()> NavigationAbort;
};

#endif // SLAM_CONTROLLER_SIGNALS_HPP