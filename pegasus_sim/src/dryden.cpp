#include <pegasus_sim/dryden.h>

namespace pegasus_sim
{
Dryden::Dryden()
{
  //************** SUBSCRIBERS AND PUBLISHERS **************//

  //******************** CLASS VARIABLES *******************//

}
Dryden::~Dryden()
{

}
void Dryden::sendWind(const ros::TimerEvent& event)
{
  // TODO
  w_xg_ = 0.0f;
  w_yg_ = 0.0f;
  w_zg_ = 0.0f;

  Wind wind_msg;
  wind_msg.w_ns = w_ns_;
  wind_msg.w_es = w_es_;
  wind_msg.w_ds = w_ds_;
  wind_msg.w_xg = w_xg_;
  wind_msg.w_yg = w_yg_;
  wind_msg.w_zg = w_zg_;
  wind_publisher_.publish(wind_msg);
}
} // end namespace pegasus_sim
