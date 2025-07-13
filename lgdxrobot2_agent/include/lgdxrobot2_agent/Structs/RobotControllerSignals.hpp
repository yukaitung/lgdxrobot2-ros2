#ifndef ROBOT_CONTROLLER_SIGNALS_HPP
#define ROBOT_CONTROLLER_SIGNALS_HPP

#include <boost/signals2/signal.hpp>

#include "proto/RobotClientsService.grpc.pb.h"

#include "geometry_msgs/msg/pose_stamped.hpp"

struct RobotControllerSignals
{
  boost::signals2::signal<void(RobotClientsRobotCriticalStatus &,
    std::vector<double> &,
    RobotClientsDof &,
    RobotClientsAutoTaskNavProgress &)> CloudExchange;
  boost::signals2::signal<void(std::vector<geometry_msgs::msg::PoseStamped> &)> NavigationStart;
  boost::signals2::signal<void()> NavigationAbort;
  boost::signals2::signal<void(RobotClientsNextToken &)> AutoTaskNext;
  boost::signals2::signal<void(RobotClientsAbortToken &)> AutoTaskAbort;
};

#endif // ROBOT_CONTROLLER_SIGNALS_HPP