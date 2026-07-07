#ifndef SENSOR_SIGNALS_HPP
#define SENSOR_SIGNALS_HPP

#include <boost/signals2/signal.hpp>

struct SensorSignals
{
  boost::signals2::signal<void(bool)> set_estop;
  boost::signals2::signal<void(float, float, float)> set_inverse_kinematics;
};

#endif // SENSOR_SIGNALS_HPP