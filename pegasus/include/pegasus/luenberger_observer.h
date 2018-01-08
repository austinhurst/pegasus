#ifndef LUENBERGER_OBSERVER_H
#define LUENBERGER_OBSERVER_H

#include <pegasus/estimator_base.h>

namespace pegasus
{
class LuenbergerObserver : public Estimator
{
public:
  LuenbergerObserver();

private:
  virtual void estimate(const ros::TimerEvent& event);

};// end class LuenbergerObserver
} // end namespace pegasus

#endif // LUENBERGER_OBSERVER_H
