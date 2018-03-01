#include <pegasus/luenberger_observer.h>

namespace pegasus
{
LuenbergerObserver::LuenbergerObserver()
{
  // SETUP THE ESTIMATOR HERE

}
void LuenbergerObserver::estimate(const ros::TimerEvent& event)
{
  ros::Time new_time = ros::Time::now();
  // IMPLEMENT THE ESTIMATOR HERE:

  state_hat_publisher_.publish(state_hat_.msg());
  last_time_ = new_time;
}
} // end namespace pegasus
