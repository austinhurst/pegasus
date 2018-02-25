#include "pegasus_sim/sensor_models.h"

namespace pegasus_sim
{
SensorModels::SensorModels()
{
  // GPS Parameters
  float gps_rate, k_GPS;
  if (!(ros::param::get("/pegasus/vehicle_description/sensors/gps/gps_rate",gps_rate)))
    ROS_WARN("No param named 'gps_rate'");
  if (!(ros::param::get("/pegasus/vehicle_description/sensors/gps/k_GPS",k_GPS)))
    ROS_WARN("No param named 'k_GPS'");
  if (!(ros::param::get("/pegasus/vehicle_description/sensors/gps/sigma_gps_n",sigma_gps_n_)))
    ROS_WARN("No param named 'sigma_gps_n'");
  if (!(ros::param::get("/pegasus/vehicle_description/sensors/gps/sigma_gps_e",sigma_gps_e_)))
    ROS_WARN("No param named 'sigma_gps_e'");
  if (!(ros::param::get("/pegasus/vehicle_description/sensors/gps/sigma_gps_h",sigma_gps_h_)))
    ROS_WARN("No param named 'sigma_gps_h'");
  if (!(ros::param::get("/pegasus/vehicle_description/sensors/gps/sigma_gps_V",sigma_gps_V_)))
    ROS_WARN("No param named 'sigma_gps_V'");
  K_GPS_ = exp(-k_GPS/gps_rate);
  nu_n_ = 0.0f;
  nu_e_ = 0.0f;
  nu_h_ = 0.0f;
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

  float c_theta = cos(truth_.theta);
  float s_theta = sin(truth_.theta);
  float c_psi   = cos(truth_.psi);
  float s_psi   = sin(truth_.psi);
  float c_phi   = cos(truth_.phi);
  float s_phi   = sin(truth_.phi);

  double R[2][3];
  R[0][0]   = c_theta*c_psi;
  R[0][1]   = s_phi*s_theta*c_psi - c_phi*s_psi;
  R[0][2]   = c_phi*s_theta*c_psi + s_phi*s_psi;
  R[1][0]   = c_theta*s_psi;
  R[1][1]   = s_phi*s_theta*s_psi;
  R[1][2]   = c_phi*s_theta*s_psi - s_phi*c_psi;

  double Vn = R[0][0]*truth_.u + R[0][1]*truth_.v + R[0][2]*truth_.w;
  double Ve = R[1][0]*truth_.u + R[1][1]*truth_.v + R[1][2]*truth_.w;

  // Position
  nu_n_     = K_GPS_*nu_n_ + norm_rnd(0.0,sigma_gps_n_);
  nu_e_     = K_GPS_*nu_e_ + norm_rnd(0.0,sigma_gps_e_);
  nu_h_     = K_GPS_*nu_h_ + norm_rnd(0.0,sigma_gps_h_);

  double N  = (double)  truth_.pn + nu_n_;
  double E  = (double)  truth_.pe + nu_e_;
  double H  = (double) -truth_.pd + nu_h_;

  // Ground Velocity
  float Vg  = sqrtf(Vn*Vn + Ve*Ve) + norm_rnd(0.0f, sigma_gps_V_);

  // Ground Course
  float sigma_gps_chi = sigma_gps_V_/Vg;
  float chi = atan2f(Ve, Vn) + norm_rnd(0.0f,sigma_gps_chi);

  gps_converter_.ned2gps(N, E, -H, lat_N, lon_E, h_M);

  gps_msg_.fix           = true;
  gps_msg_.NumSat        = 1;
  gps_msg_.latitude      = lat_N;
  gps_msg_.longitude     = lon_E;
  gps_msg_.altitude      = h_M;
  gps_msg_.speed         = Vg;
  gps_msg_.ground_course = chi;
  gps_msg_.covariance    = 0.0;

  gps_publisher_.publish(gps_msg_);
}
void SensorModels::sendBarometer(const ros::TimerEvent& event)
{
  barometer_msg_.pressure = 0.0;

  barometer_publisher_.publish(barometer_msg_);
}
} // end namespace pegasus_sim
