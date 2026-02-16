#ifndef MCUSIGNALS_HPP
#define MCUSIGNALS_HPP

#include <boost/signals2/signal.hpp>

struct McuSignals
{
  boost::signals2::signal<void(const McuData &)> UpdateMcuData;
  boost::signals2::signal<void(std::string)> UpdateSerialNumber;
};

#endif // MCUSIGNALS_HPP