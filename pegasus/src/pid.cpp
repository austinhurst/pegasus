#include "pegasus/pid.h"

namespace pegasus
{
PID::PID()
{
  // SETUP THE CONTROLLER HERE

}
void PID::control(const ros::TimerEvent& event)
{
  ros::Time new_time = ros::Time::now();
  mapControlChannels();
  // IMPLEMENT THE CONTROLLER HERE:
  // last_time_
  // state_.
  // motors_->


  if (flight_mode_ == ANGLE_MODE)
  {

  }
  else if (flight_mode_ == RATES_MODE)
  {

  }
  else if (flight_mode_ == VELOC_MODE)
  {

  }
  else if (flight_mode_ == AUTO__MODE)
  {

  }

  publishMotorCommand();
  last_time_ = new_time;
}
} // end namespace pegasus