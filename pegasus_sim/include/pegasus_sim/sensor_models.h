#ifndef SENSOR_MODELS_H
#define SENSOR_MODELS_H

#include <pegasus_sim/sensors_base.h>

namespace pegasus_sim
{
class SensorModels : public SimSensors
{
public:
  SensorModels();

private:
  virtual void sendSonar(const ros::TimerEvent& event);
  virtual void sendIMU(const ros::TimerEvent& event);
  virtual void sendGPS(const ros::TimerEvent& event);
  virtual void sendBarometer(const ros::TimerEvent& event);

  // GPS Variables
  double nu_n_;
  double nu_e_;
  double nu_h_;
  float  sigma_gps_n_;
  float  sigma_gps_e_;
  float  sigma_gps_h_;
  float  sigma_gps_V_;
  float  K_GPS_;


};// end class SensorModels
} // end namespace pegasus_sim

#endif // SENSOR_MODELS_H
