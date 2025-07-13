#ifndef CLOUDSIGNALS_HPP
#define CLOUDSIGNALS_HPP

#include <boost/signals2/signal.hpp>
#include "proto/RobotClientsService.grpc.pb.h"

struct CloudSignals
{
  boost::signals2::signal<void()> NextExchange;
  boost::signals2::signal<void(const RobotClientsRespond *respond)> HandleExchange;
  boost::signals2::signal<void()> Error;
};

#endif // CLOUDSIGNALS_HPP