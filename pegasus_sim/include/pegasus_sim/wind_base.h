#ifndef WIND_BASE_H
#define WIND_BASE_H

#include <ros/ros.h>
#include <pegasus/motor_struct.h>
#include <pegasus/state_struct.h>
#include <pegasus_sim/Wind.h>

namespace pegasus_sim
{
class WindModel
{
public:
  WindModel();
  ~WindModel();

private:
  //********************* NODE HANDLES *********************//
  ros::NodeHandle nh_;         // public node handle for publishing, subscribing

  //************** SUBSCRIBERS AND PUBLISHERS **************//
protected:
  ros::Publisher wind_publisher_;

  //******************** CLASS VARIABLES *******************//
  float w_ns_;
  float w_es_;
  float w_ds_;
  float w_xg_;
  float w_yg_;
  float w_zg_;

  //***************** CALLBACKS AND TIMERS *****************//
  virtual void sendWind(const ros::TimerEvent& event);
  ros::Timer send_wind_timer_;
  //********************** FUNCTIONS ***********************//

};// end class WindModel
} // end namespace pegasus_sim

#endif // WIND_BASE_H
