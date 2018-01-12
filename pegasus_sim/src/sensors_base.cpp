#include "pegasus_sim/sensors_base.h"

#include "pegasus_sim/sensor_models.h"

namespace pegasus_sim
{
SimSensors::SimSensors() :
  nh_(ros::NodeHandle())
{
  //********************** PARAMETERS **********************//
  // Simulation Parameters
  bool use_sonar, use_gps, use_imu, use_barometer;
  float sonar_rate, gps_rate, imu_rate, barometer_rate;
  if (!(ros::param::get("/pegasus/vehicle_description/sensors/use_sonar",use_sonar)))
    ROS_WARN("No param named 'use_sonar'");
  else
  {
    if (!(ros::param::get("/pegasus/vehicle_description/sensors/sonar/sonar_rate",sonar_rate)))
      ROS_WARN("No param named 'sonar_rate'");
  }
  if (!(ros::param::get("/pegasus/vehicle_description/sensors/use_gps",use_gps)))
    ROS_WARN("No param named 'use_gps'");
  else
  {
    if (!(ros::param::get("/pegasus/vehicle_description/sensors/gps/gps_rate",gps_rate)))
      ROS_WARN("No param named 'gps_rate'");
  }
  if (!(ros::param::get("/pegasus/vehicle_description/sensors/use_imu",use_imu)))
    ROS_WARN("No param named 'use_imu'");
  else
  {
    if (!(ros::param::get("/pegasus/vehicle_description/sensors/imu/imu_rate",imu_rate)))
      ROS_WARN("No param named 'imu_rate'");
  }
  if (!(ros::param::get("/pegasus/vehicle_description/sensors/use_barometer",use_barometer)))
    ROS_WARN("No param named 'use_barometer'");
  else
  {
    if (!(ros::param::get("/pegasus/vehicle_description/sensors/barometer/barometer_rate",barometer_rate)))
      ROS_WARN("No param named 'barometer_rate'");
  }

  //************** SUBSCRIBERS AND PUBLISHERS **************//
  truth_subscriber_ = nh_.subscribe("truth",1,&SimSensors::truthCallback, this);
  if (use_sonar)
    sonar_publisher_ = nh_.advertise<pegasus::Sonar>("/pegasus/sonar",1);
  if (use_gps)
    gps_publisher_ = nh_.advertise<pegasus::GPS>("/pegasus/gps",1);
  if (use_barometer)
    barometer_publisher_ = nh_.advertise<pegasus::Barometer>("/pegasus/barometer",1);
  if (use_imu)
    imu_publisher_ = nh_.advertise<sensor_msgs::Imu>("/pegasus/imu",1);

  //******************** CLASS VARIABLES *******************//
  srand(time(0));

  //***************** CALLBACKS AND TIMERS *****************//
  if (use_sonar)
    sonar_timer_ = nh_.createTimer(ros::Duration(1.0/sonar_rate), &SimSensors::sendSonar, this);
  if (use_gps)
    gps_timer_ = nh_.createTimer(ros::Duration(1.0/gps_rate), &SimSensors::sendGPS, this);
  if (use_barometer)
    barometer_timer_ = nh_.createTimer(ros::Duration(1.0/barometer_rate), &SimSensors::sendBarometer, this);
  if (use_imu)
    imu_timer_ = nh_.createTimer(ros::Duration(1.0/imu_rate), &SimSensors::sendIMU, this);
  //********************** FUNCTIONS ***********************//

}
SimSensors::~SimSensors()
{

}
void SimSensors::sendSonar(const ros::TimerEvent& event)
{
  ROS_ERROR("CHILD CLASS FUNCTION 'sendSonar' WAS NOT CALLED.");
}
void SimSensors::sendIMU(const ros::TimerEvent& event)
{
  ROS_ERROR("CHILD CLASS FUNCTION 'sendIMU' WAS NOT CALLED.");
}
void SimSensors::sendGPS(const ros::TimerEvent& event)
{
  ROS_ERROR("CHILD CLASS FUNCTION 'sendGPS' WAS NOT CALLED.");
}
void SimSensors::sendBarometer(const ros::TimerEvent& event)
{
  ROS_ERROR("CHILD CLASS FUNCTION 'sendBarometer' WAS NOT CALLED.");
}
void SimSensors::truthCallback(const pegasus::VehicleState &msg)
{
  truth_.msg2struct(msg);
}
} // end namespace pegasus_sim

//********************************************************//
//************************ MAIN **************************//
//********************************************************//
int main(int argc, char** argv)
{
  ros::init(argc, argv, "sensor_simulation");
  ros::NodeHandle nh("pegasus_sim");

  pegasus_sim::SensorModels sensors_obj;
  ros::spin();
  return 0;
} // end main
