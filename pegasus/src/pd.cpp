#include "pegasus/pd.h"

namespace pegasus
{
PD::PD()
{
  // SETUP THE CONTROLLER HERE

}
void PD::control(const ros::TimerEvent& event)
{
  ros::Time new_time = ros::Time::now();
  mapControlChannels();
  // IMPLEMENT THE CONTROLLER HERE:
  // last_time_
  // state_.
  // motors_->
  // aileron_stick_, elevator_stick_, thrust_stick_, rudder_stick_

  publishMotorCommand();
  last_time_ = new_time;
}
} // end namespace pegasus
