#ifndef MCU_SIGNALS_HPP
#define MCU_SIGNALS_HPP

#include <boost/signals2/signal.hpp>

#include "lgdxrobot2.h"

struct McuSignals
{
  boost::signals2::signal<void(const McuData &)> update_mcu_data;
};

#endif // MCU_SIGNALS_HPP