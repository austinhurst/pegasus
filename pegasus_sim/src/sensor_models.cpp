#include "pegasus_sim/sensor_models.h"

namespace pegasus_sim
{
SensorModels::SensorModels()
{

}
void SensorModels::sendSonar(const ros::TimerEvent& event)
{
  // Implement Sonar Model Here
  sonar_msg_.distance = 0.0;

  sonar_publisher_.publish(sonar_msg_);
}
void SensorModels::sendIMU(const ros::TimerEvent& event)
{
  // Implement IMU Model Here
  geometry_msgs::Quaternion q;
  q.x = 0.0;
  q.y = 0.0;
  q.z = 0.0;
  q.w = 0.0;
  imu_msg_.orientation = q;
  imu_msg_.angular_velocity.x = 0.0;
  imu_msg_.angular_velocity.y = 0.0;
  imu_msg_.angular_velocity.z = 0.0;
  imu_msg_.linear_acceleration.x = 0.0;
  imu_msg_.linear_acceleration.y = 0.0;
  imu_msg_.linear_acceleration.z = 0.0;
  for (int i = 0; i < 9; i++)
  {
    imu_msg_.orientation_covariance[i] = 0.0;
    imu_msg_.angular_velocity_covariance[i] = 0.0;
    imu_msg_.linear_acceleration_covariance[i] = 0.0;
  }

}
void SensorModels::sendGPS(const ros::TimerEvent& event)
{
  // Implement GPS Model Here
  double lat_N, lon_E, h_M;
  double N = (double) truth_.pn;
  double E = (double) truth_.pe;
  double D = (double) truth_.pd;

  gps_converter_.ned2gps(N, E, D, lat_N, lon_E, h_M);

  gps_msg_.fix           = true;
  gps_msg_.NumSat        = 1;
  gps_msg_.latitude      = lat_N;
  gps_msg_.longitude     = lon_E;
  gps_msg_.altitude      = h_M;
  gps_msg_.speed         = 0.0;
  gps_msg_.ground_course = 0.0;
  gps_msg_.covariance    = 0.0;

  gps_publisher_.publish(gps_msg_);
}
void SensorModels::sendBarometer(const ros::TimerEvent& event)
{
  barometer_msg_.pressure = 0.0;

  barometer_publisher_.publish(barometer_msg_);
}
} // end namespace pegasus_sim
