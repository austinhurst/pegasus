#ifndef DRYDEN_H
#define DRYDEN_H

#include <math.h>
#include <pegasus_sim/wind_base.h>
#include <pegasus_sim/Wind.h>


namespace pegasus_sim
{
class Dryden : public WindModel
{
public:
  Dryden();
  ~Dryden();
private:
  //********************* NODE HANDLES *********************//
  ros::NodeHandle nh_;         // public node handle for publishing, subscribing

  //************** SUBSCRIBERS AND PUBLISHERS **************//

  //******************** CLASS VARIABLES *******************//

  //***************** CALLBACKS AND TIMERS *****************//
  virtual void sendWind(const ros::TimerEvent& event);

  //********************** FUNCTIONS ***********************//

};// end class Dryden
} // end namespace pegasus_sim

#endif // DRYDEN_H
