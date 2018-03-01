#include <pegasus_sim/wind_base.h>

#include <pegasus_sim/dryden.h>

namespace pegasus_sim
{
WindModel::WindModel() :
  nh_(ros::NodeHandle())
{
  //********************** PARAMETERS **********************//
  float wind_rate;
  if (!(ros::param::get("sim/wind/wind_rate",wind_rate)))
    ROS_WARN("No param named 'wind_rate'");
  if (!(ros::param::get("sim/wind/w_ns",w_ns_)))
    ROS_WARN("No param named 'w_ns'");
  if (!(ros::param::get("sim/wind/w_es",w_es_)))
    ROS_WARN("No param named 'w_es'");
  if (!(ros::param::get("sim/wind/w_ds",w_ds_)))
    ROS_WARN("No param named 'w_ds'");

  //************** SUBSCRIBERS AND PUBLISHERS **************//
  wind_publisher_ = nh_.advertise<pegasus_sim::Wind>("wind",1);

  //******************** CLASS VARIABLES *******************//

  //***************** CALLBACKS AND TIMERS *****************//
  send_wind_timer_ = nh_.createTimer(ros::Duration(1.0/wind_rate), &WindModel::sendWind, this);

  //********************** FUNCTIONS ***********************//

}
WindModel::~WindModel()
{

}
void WindModel::sendWind(const ros::TimerEvent& event)
{
  ROS_ERROR("CHILD CLASS FUNCTION 'sendWind' WAS NOT CALLED.");
}
} // end namespace pegasus_sim

//********************************************************//
//************************ MAIN **************************//
//********************************************************//
int main(int argc, char** argv)
{
  ros::init(argc, argv, "wind");
  ros::NodeHandle nh("pegasus_sim");

  pegasus_sim::WindModel *wind_obj;
  std::string wind_type;
  if (!(ros::param::get("sim/wind/wind_type",wind_type)))
    ROS_WARN("No param named 'wind_type'");
  if (wind_type == "dryden")
    wind_obj = new pegasus_sim::Dryden;
  else
    ROS_ERROR("NO WIND MODEL INITIALIZED");

  ros::spin();

  delete wind_obj;
  return 0;
} // end main
