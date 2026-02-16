#ifndef SENSORSIGNALS_HPP
#define SENSORSIGNALS_HPP

#include <boost/signals2/signal.hpp>

struct SensorSignals
{
  boost::signals2::signal<void(bool)> SetEstop;
  boost::signals2::signal<void(float, float, float)> SetInverseKinematics;
};

#endif // SENSORSIGNALS_HPP