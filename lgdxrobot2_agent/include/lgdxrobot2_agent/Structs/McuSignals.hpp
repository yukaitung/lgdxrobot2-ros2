#ifndef MCUSIGNALS_HPP
#define MCUSIGNALS_HPP

#include <boost/signals2/signal.hpp>

#include "RobotData.hpp"

struct McuSignals
{
  boost::signals2::signal<void(const RobotData &)> UpdateRobotData;
  boost::signals2::signal<void(const char *)> UpdateSerialNumber;
};

#endif // MCUSIGNALS_HPP