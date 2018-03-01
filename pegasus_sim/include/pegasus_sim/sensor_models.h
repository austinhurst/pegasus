#ifndef SENSOR_MODELS_H
#define SENSOR_MODELS_H

#include <pegasus_sim/sensors_base.h>
#include <pegasus_sim/forces_and_moments.h>

namespace pegasus_sim
{
class SensorModels : public SimSensors
{
public:
  SensorModels(ForcesAndMoments* f_and_m_ptr);
  ~SensorModels();

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

  // Accelerometer Variables
  float mass_;
  float g_;
  float sigma_accel_x_;
  float sigma_accel_y_;
  float sigma_accel_z_;

  // Gyro Variables
  float sigma_gyro_x_;
  float sigma_gyro_y_;
  float sigma_gyro_z_;

  // Barometer Variables
  float sigma_baro_;
  float bias_baro_;
  float rho_g_;

};// end class SensorModels
} // end namespace pegasus_sim

#endif // SENSOR_MODELS_H
