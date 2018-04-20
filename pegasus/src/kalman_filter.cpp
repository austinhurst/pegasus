#include <pegasus/kalman_filter.h>

namespace pegasus
{
KalmanFilter::KalmanFilter()
{
  // SETUP THE ESTIMATOR HERE
  N_ = 10;
  for (int i = 0; i < NUM_STATES; i++)
    xhat_[i] = 0.0f;

  float p_cutoff_freq, q_cutoff_freq, r_cutoff_freq, baro_cutoff_freq;
  if (!(ros::param::get("est/kalman/p_cutoff_freq",p_cutoff_freq)))
    ROS_WARN("No param named 'p_cutoff_freq'");
  if (!(ros::param::get("est/kalman/q_cutoff_freq",q_cutoff_freq)))
    ROS_WARN("No param named 'q_cutoff_freq'");
  if (!(ros::param::get("est/kalman/r_cutoff_freq",r_cutoff_freq)))
    ROS_WARN("No param named 'r_cutoff_freq'");
  if (!(ros::param::get("est/kalman/baro_cutoff_freq",baro_cutoff_freq)))
    ROS_WARN("No param named 'baro_cutoff_freq'");

  gyro_p_.cutoff_frequency_    = p_cutoff_freq;
  gyro_q_.cutoff_frequency_    = q_cutoff_freq;
  gyro_r_.cutoff_frequency_    = r_cutoff_freq;
  barometer_.cutoff_frequency_ = baro_cutoff_freq;
  gyro_last_time_              = ros::Time::now();
  barometer_last_time_         = ros::Time::now();
}
void KalmanFilter::predict(const ros::TimerEvent& event)
{

  // needs to estimate at least:
  // pn, pe, pd
  // phi, theta, psi
  // p, q, r
  // either Vg and chi
  //     or u, v, and w
  //     or u1 and v1

  // n and e for future navigation

  ros::Time new_time = ros::Time::now();
  ros::Duration time_step = new_time - last_time_;
  float ts = time_step.toSec();
  // Attitude Estimator - PHI AND THETA
  // Prediction Step

  float phi, theta, psi, u1, v1;
  float s_phi, c_phi, t_theta, sec_theta, s_theta, c_theta;
  float p = state_hat_.p;
  float q = state_hat_.q;
  float r = state_hat_.r;
  getF();
  float az = fz_p_/mass_;
  for (int i = 0; i < N_; i++)
  {
    phi   = xhat_[0];
    theta = xhat_[1];
    psi   = xhat_[2];
    u1    = xhat_[3];
    v1    = xhat_[4];
    c_phi     = cosf(phi);
    s_phi     = sinf(phi);
    t_theta   = tanf(theta);
    sec_theta = 1.0f/cos(theta);
    s_theta   = sinf(theta);


    f_[0] = p +   s_phi*t_theta*q +   c_phi*t_theta*r;
    f_[1] =               c_phi*q -           s_phi*r;
    f_[2] =     s_phi*sec_theta*q + c_phi*sec_theta*r;
    f_[3] = c_phi*s_theta*az;
    f_[4] = -s_phi*az;

    // xhat_ = xhat_ + ts/N*f;
    for (int i = 0; i < NUM_STATES; i++)
      xhat_[i] += ts/N_*f_[i];

    phi   = xhat_[0];
    theta = xhat_[1];
    psi   = xhat_[2];
    u1    = xhat_[3];
    v1    = xhat_[4];
    c_phi     = cosf(phi);
    s_phi     = sinf(phi);
    t_theta   = tanf(theta);
    sec_theta = 1.0f/cos(theta);
    s_theta   = sinf(theta);
    c_theta   = cosf(theta);
    // A_ = df/dx
    A_[0][0] = q*c_phi*t_theta - r*s_phi*t_theta;
    A_[0][1] = (q*s_phi + r*c_phi)/(c_theta*c_theta);
    A_[0][2] = 0.0f;
    A_[0][3] = 0.0f;
    A_[0][4] = 0.0f;
    A_[1][0] = -q*s_phi - r*c_phi;
    A_[1][1] = 0.0f;
    A_[1][2] = 0.0f;
    A_[1][3] = 0.0f;
    A_[1][4] = 0.0f;
    A_[2][0] = (q*c_phi - r*s_phi)/c_theta;
    A_[2][1] = -(q*s_phi + r*c_phi)*t_theta/c_theta;
    A_[2][2] = 0.0f;
    A_[2][3] = 0.0f;
    A_[2][4] = 0.0f;
    A_[3][0] = -s_phi*s_theta*az;
    A_[3][1] = c_phi*c_theta*az;
    A_[3][2] = 0.0f;
    A_[3][3] = 0.0f;
    A_[3][4] = 0.0f;
    A_[4][0] = -c_phi*az;
    A_[4][1] = 0.0;
    A_[4][2] = 0.0f;
    A_[4][3] = 0.0f;
    A_[4][4] = 0.0f;

    // P = P + (ts/N)*(A*P + P*A.' + Q);
    for (int i = 0; i < NUM_STATES; i++)
    {
      for (int j = 0; j < NUM_STATES; j++)
      {
        float AP  = 0.0f;
        float PAt = 0.0f;
        for (int k = 0; k < NUM_STATES; k++)
          AP  += A_[i][k]*P_[k][j];
        for (int k = 0; k < NUM_STATES; k++)
          PAt += P_[i][k]*A_[j][k];
        P_[i][j] += (ts/N_)*(AP + PAt + Q_[i][j]);
      }
    }
    // wrap any angles
  }

  // Pull in xhat variables into state_hat_
  state_hat_.phi   = xhat_[0];
  state_hat_.theta = xhat_[1];
  state_hat_.psi   = xhat_[2];
  state_hat_.u1    = xhat_[3];
  state_hat_.v1    = xhat_[4];
  state_hat_publisher_.publish(state_hat_.msg());
  last_time_ = new_time;
}
void KalmanFilter::correctBarometer()
{
  // TODO put altitude in this?
  ros::Time new_time      = ros::Time::now();
  ros::Duration time_step = new_time - barometer_last_time_;
  float ts                = time_step.toSec();
  float pressure          = barometer_.filter(barometer_pressure_,ts);
  barometer_last_time_    = new_time;
  float h                 = pressure/(rho_*g_);

  // phi       = xhat_[0];
  // theta     = xhat_[1];
  // C_    = [0, q*Va*cos(theta) + g_*cos(theta)]; // dh/dx but for x
  // K_    = P_*C_.'*inv(R_ + C_*P_*C_.');
  // P_    = (eye(2) - K_*C_)*P_;
  // h_    = q*Va*sin(theta) + g_*sin(theta);
  // xhat_ = xhat_ + K_*(Sax - h_);

  // Pull in xhat variables into state_hat_
  state_hat_publisher_.publish(state_hat_.msg());
  last_time_ = new_time;
}
void KalmanFilter::correctGPS()
{
  // Measurements
  // gps_N_, gps_E_, gps_D_, gps_speed_, gps_ground_course_;
  // phi       = xhat_[0];
  // theta     = xhat_[1];
  // C_    = [0, q*Va*cos(theta) + g_*cos(theta)]; // dh/dx but for x
  // K_    = P_*C_.'*inv(R_ + C_*P_*C_.');
  // P_    = (eye(2) - K_*C_)*P_;
  // h_    = q*Va*sin(theta) + g_*sin(theta);
  // xhat_ = xhat_ + K_*(Sax - h_);

  // Pull in xhat variables into state_hat_
  state_hat_publisher_.publish(state_hat_.msg());
}
void KalmanFilter::correctIMU()
{

  // imu_angular_velocity_[0], imu_linear_acceleration_[0]
  ros::Time new_time      = ros::Time::now();
  ros::Duration time_step = new_time - gyro_last_time_;
  float ts                = time_step.toSec();
  float p                 = gyro_p_.filter(imu_angular_velocity_[0],ts);
  float q                 = gyro_q_.filter(imu_angular_velocity_[1],ts);
  float r                 = gyro_r_.filter(imu_angular_velocity_[2],ts);
  gyro_last_time_         = new_time;

  float phi, theta, psi, u1, v1;
  float s_phi, c_phi, t_theta, sec_theta, s_theta, c_theta;
  phi   = xhat_[0];
  theta = xhat_[1];
  psi   = xhat_[2];
  u1    = xhat_[3];
  v1    = xhat_[4];
  c_phi     = cosf(phi);
  s_phi     = sinf(phi);
  t_theta   = tanf(theta);
  sec_theta = 1.0f/cos(theta);
  s_theta   = sinf(theta);
  c_theta   = cosf(theta);

  getF();
  float hx = fx_p_/mass_ + g_*s_theta;
  float hy = fy_p_/mass_ - g_*c_theta*s_phi;
  float hz = fz_p_/mass_ - g_*c_theta*c_phi;
  float x_correction = imu_linear_acceleration_[0] - hx;
  float y_correction = imu_linear_acceleration_[1] - hy;
  float z_correction = imu_linear_acceleration_[2] - hz;

  // C_    = [0, q*Va*cos(theta) + g_*cos(theta)]; // dh/dx but for x
  // K_    = P_*C_.'*inv(R_ + C_*P_*C_.');
  // P_    = (eye(2) - K_*C_)*P_;

  // xhat_ = xhat_ + K_*(Sensor - h_);
  for (int i = 0; i < NUM_STATES; i++)
    xhat_[i] += K_[i]*x_correction;
  // C_    = [0, q*Va*cos(theta) + g_*cos(theta)]; // dh/dx but for y
  // K_    = P_*C_.'*inv(R_ + C_*P_*C_.');
  // P_    = (eye(2) - K_*C_)*P_;

  // xhat_ = xhat_ + K_*(Sensor - h_);
  for (int i = 0; i < NUM_STATES; i++)
    xhat_[i] += K_[i]*y_correction;

  // C_    = [0, q*Va*cos(theta) + g_*cos(theta)]; // dh/dx but for z
  // K_    = P_*C_.'*inv(R_ + C_*P_*C_.');
  // P_    = (eye(2) - K_*C_)*P_;

  // xhat_ = xhat_ + K_*(Sensor - h_);
  for (int i = 0; i < NUM_STATES; i++)
    xhat_[i] += K_[i]*z_correction;


  // Pull in xhat variables into state_hat_
  state_hat_.p     = p;
  state_hat_.q     = q;
  state_hat_.r     = r;
  state_hat_.phi   = xhat_[0];
  state_hat_.theta = xhat_[1];
  state_hat_.psi   = xhat_[2];
  state_hat_.u1    = xhat_[3];
  state_hat_.v1    = xhat_[4];
  state_hat_publisher_.publish(state_hat_.msg());
  last_time_ = new_time;
}
} // end namespace pegasus
