#ifndef NAVIGATION_SIGNALS_HPP
#define NAVIGATION_SIGNALS_HPP

#include <boost/signals2/signal.hpp>

struct NavigationSignals
{
  boost::signals2::signal<void()> Done;
  boost::signals2::signal<void()> Stuck;
  boost::signals2::signal<void()> Cleared;
  boost::signals2::signal<void()> Abort;
};

#endif // NAVIGATION_SIGNALS_HPP