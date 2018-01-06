#include "pegasus/pd.h"

namespace pegasus
{
PD::PD()
{
  // SETUP THE CONTROLLER HERE

}
void PD::control(const ros::TimerEvent& event)
{
  // IMPLEMENT THE CONTROLLER HERE:
  // last_time_
  // state_.
  // motors_->
  // rx_ch?_


  publishMotorCommand();
}
} // end namespace pegasus
