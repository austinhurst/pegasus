#include <pegasus_sim/sensor_models.h>

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
  nu_n_ = 0.0f; // TODO set to right value
  nu_e_ = 0.0f;
  nu_h_ = 0.0f;

  // Accelerometer Parameters
  if (!(ros::param::get("/pegasus/vehicle_description/mass",mass_)))
    ROS_WARN("No param named 'mass");
  if (!(ros::param::get("/pegasus/vehicle_description/g",g_)))
    ROS_WARN("No param named 'g");
  if (!(ros::param::get("/pegasus/vehicle_description/sensors/imu/sigma_accel_x",sigma_accel_x_)))
    ROS_WARN("No param named 'sigma_accel_x'");
  if (!(ros::param::get("/pegasus/vehicle_description/sensors/imu/sigma_accel_y",sigma_accel_y_)))
    ROS_WARN("No param named 'sigma_accel_y'");
  if (!(ros::param::get("/pegasus/vehicle_description/sensors/imu/sigma_accel_z",sigma_accel_z_)))
    ROS_WARN("No param named 'sigma_accel_z'");
  sigma_accel_x_ *= g_;
  sigma_accel_y_ *= g_;
  sigma_accel_z_ *= g_;

  // Gyro Parameters
  if (!(ros::param::get("/pegasus/vehicle_description/sensors/imu/sigma_accel_x",sigma_gyro_x_)))
    ROS_WARN("No param named 'sigma_gyro_x'");
  if (!(ros::param::get("/pegasus/vehicle_description/sensors/imu/sigma_accel_y",sigma_gyro_y_)))
    ROS_WARN("No param named 'sigma_gyro_y'");
  if (!(ros::param::get("/pegasus/vehicle_description/sensors/imu/sigma_accel_z",sigma_gyro_z_)))
    ROS_WARN("No param named 'sigma_gyro_z'");
  sigma_gyro_x_ *= M_PI/180.f;
  sigma_gyro_y_ *= M_PI/180.f;
  sigma_gyro_z_ *= M_PI/180.f;

  // Barometer Parameters
  float rho;
  if (!(ros::param::get("/pegasus/vehicle_description/rho",rho)))
    ROS_WARN("No param named 'rho");
  if (!(ros::param::get("/pegasus/vehicle_description/sensors/barometer/sigma_baro",sigma_baro_)))
    ROS_WARN("No param named 'sigma_baro'");
  if (!(ros::param::get("/pegasus/vehicle_description/sensors/barometer/bias_baro",bias_baro_)))
    ROS_WARN("No param named 'bias_baro'");
  rho_g_ = rho*g_;

}
SensorModels::~SensorModels()
{

}
void SensorModels::sendSonar(const ros::TimerEvent& event)
{
  // Implement Sonar Model Here
  // TODO
  sonar_msg_.distance = 0.0;

  sonar_publisher_.publish(sonar_msg_);
}
void SensorModels::sendIMU(const ros::TimerEvent& event)
{
  float fx, fy, fz, l, m, n;
  f_and_m_obj_->getForcesAndMoments(truth_, fx, fy, fz, l, m, n);

  // Gyro
  imu_msg_.angular_velocity.x    = truth_.p + norm_rnd(0.0f, sigma_gyro_x_);
  imu_msg_.angular_velocity.y    = truth_.q + norm_rnd(0.0f, sigma_gyro_y_);
  imu_msg_.angular_velocity.z    = truth_.r + norm_rnd(0.0f, sigma_gyro_z_);

  // Accelerometer
  imu_msg_.linear_acceleration.x = fx/mass_ + g_*sinf(truth_.theta)                  + norm_rnd(0.0f, sigma_accel_x_);
  imu_msg_.linear_acceleration.y = fy/mass_ - g_*cosf(truth_.theta)*sinf(truth_.phi) + norm_rnd(0.0f, sigma_accel_y_);
  imu_msg_.linear_acceleration.z = fz/mass_ - g_*cosf(truth_.theta)*cosf(truth_.phi) + norm_rnd(0.0f, sigma_accel_z_);

  imu_publisher_.publish(imu_msg_);

}
void SensorModels::sendGPS(const ros::TimerEvent& event)
{
  // Implement GPS Model Here
  float lat_N, lon_E, h_M;

  float c_theta = cosf(truth_.theta);
  float s_theta = sinf(truth_.theta);
  float c_psi   = cosf(truth_.psi);
  float s_psi   = sinf(truth_.psi);
  float c_phi   = cosf(truth_.phi);
  float s_phi   = sinf(truth_.phi);

  float R[2][3];
  R[0][0]   = c_theta*c_psi;
  R[0][1]   = s_phi*s_theta*c_psi - c_phi*s_psi;
  R[0][2]   = c_phi*s_theta*c_psi + s_phi*s_psi;
  R[1][0]   = c_theta*s_psi;
  R[1][1]   = s_phi*s_theta*s_psi;
  R[1][2]   = c_phi*s_theta*s_psi - s_phi*c_psi;

  float Vn  = R[0][0]*truth_.u + R[0][1]*truth_.v + R[0][2]*truth_.w;
  float Ve  = R[1][0]*truth_.u + R[1][1]*truth_.v + R[1][2]*truth_.w;

  // Position
  nu_n_     = K_GPS_*nu_n_ + norm_rnd(0.0,sigma_gps_n_);
  nu_e_     = K_GPS_*nu_e_ + norm_rnd(0.0,sigma_gps_e_);
  nu_h_     = K_GPS_*nu_h_ + norm_rnd(0.0,sigma_gps_h_);

  float N   =  truth_.pn + nu_n_;
  float E   =  truth_.pe + nu_e_;
  float H   = -truth_.pd + nu_h_;

  // Ground Velocity
  float Vg  = sqrtf(Vn*Vn + Ve*Ve) + norm_rnd(0.0f, sigma_gps_V_);

  // Ground Course
  float sigma_gps_chi = sigma_gps_V_/Vg;
  float chi = atan2f(Ve, Vn) + norm_rnd(0.0f,sigma_gps_chi);

  gps_converter_.ned2gps(N, E, -H, lat_N, lon_E, h_M);

  gps_msg_.fix           = true;
  gps_msg_.NumSat        = 12;
  gps_msg_.latitude      = lat_N;
  gps_msg_.longitude     = lon_E;
  gps_msg_.altitude      = h_M;
  gps_msg_.speed         = Vg;
  gps_msg_.ground_course = chi;
  gps_msg_.covariance    = 0.0; // TODO

  gps_publisher_.publish(gps_msg_);
}
void SensorModels::sendBarometer(const ros::TimerEvent& event)
{
  // TODO Put altitude in this?
  barometer_msg_.pressure = rho_g_*(-truth_.pd) + bias_baro_ + norm_rnd(0.0f, sigma_baro_);
  barometer_publisher_.publish(barometer_msg_);
}
} // end namespace pegasus_sim
