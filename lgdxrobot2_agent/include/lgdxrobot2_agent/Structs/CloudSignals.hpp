#ifndef CLOUDSIGNALS_HPP
#define CLOUDSIGNALS_HPP

#include <boost/signals2/signal.hpp>
#include "proto/RobotClientsService.grpc.pb.h"

enum class CloudFunctions
{
  Greet = 0,
  Exchange,
  AutoTaskNext,
  AutoTaskAbort,
  SlamExchange,
};

struct CloudSignals
{
  boost::signals2::signal<void()> NextExchange;
  boost::signals2::signal<void(const RobotClientsRespond *respond)> HandleExchange;
  boost::signals2::signal<void(const RobotClientsSlamRespond *respond)> HandleSlamExchange;
  boost::signals2::signal<void(CloudFunctions)> StreamError;
};

#endif // CLOUDSIGNALS_HPP