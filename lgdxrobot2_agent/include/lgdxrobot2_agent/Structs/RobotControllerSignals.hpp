#ifndef ROBOT_CONTROLLER_SIGNALS_HPP
#define ROBOT_CONTROLLER_SIGNALS_HPP

#include <boost/signals2/signal.hpp>

#include "proto/RobotClientsService.grpc.pb.h"

#include "geometry_msgs/msg/pose_stamped.hpp"

struct RobotControllerSignals
{
  boost::signals2::signal<void(RobotClientsExchange &)> CloudExchange;
  boost::signals2::signal<void(std::vector<geometry_msgs::msg::PoseStamped> &)> NavigationStart;
  boost::signals2::signal<void()> NavigationAbort;
  boost::signals2::signal<void(RobotClientsNextToken &)> AutoTaskNext;
  boost::signals2::signal<void(RobotClientsAbortToken &)> AutoTaskAbort;

  boost::signals2::signal<void(RobotClientsSlamStatus, RobotClientsExchange &)> SlamExchange2;
  boost::signals2::signal<void(RobotClientsSlamStatus, RobotClientsExchange &, RobotClientsMapData &)> SlamExchange3;
  boost::signals2::signal<void()> SaveMap;
  boost::signals2::signal<void()> Shutdown;
};

#endif // ROBOT_CONTROLLER_SIGNALS_HPP