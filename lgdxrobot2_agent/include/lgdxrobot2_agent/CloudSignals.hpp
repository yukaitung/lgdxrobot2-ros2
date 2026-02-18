#ifndef CLOUDSIGNALS_HPP
#define CLOUDSIGNALS_HPP

#include <boost/signals2/signal.hpp>

struct CloudSignals
{
  boost::signals2::signal<void(bool)> SetEstop;
};

#endif // CLOUDSIGNALS_HPP