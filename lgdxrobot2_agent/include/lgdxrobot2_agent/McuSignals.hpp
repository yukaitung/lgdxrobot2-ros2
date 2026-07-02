#ifndef MCUSIGNALS_HPP
#define MCUSIGNALS_HPP

#include <boost/signals2/signal.hpp>

struct McuSignals
{
  boost::signals2::signal<void(const McuData &)> UpdateMcuData;
};

#endif // MCUSIGNALS_HPP