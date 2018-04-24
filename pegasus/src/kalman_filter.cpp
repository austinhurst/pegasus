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

  float process_noises[NUM_STATES];
  if (!(ros::param::get("est/kalman/Q1",process_noises[0])))
    ROS_WARN("No param named 'Q1'");
  if (!(ros::param::get("est/kalman/Q2",process_noises[1])))
    ROS_WARN("No param named 'Q2'");
  if (!(ros::param::get("est/kalman/Q3",process_noises[2])))
    ROS_WARN("No param named 'Q3'");
  if (!(ros::param::get("est/kalman/Q4",process_noises[3])))
    ROS_WARN("No param named 'Q4'");
  if (!(ros::param::get("est/kalman/Q5",process_noises[4])))
    ROS_WARN("No param named 'Q5'");

  for (int i = 0; i < NUM_STATES; i++)
  {
    for (int j = 0; j < NUM_STATES; j++)
    {
      P_[i][j] = 0.0f;
      Q_[i][j] = 0.0f;
      if (i == j)
      {
        P_[i][j] = 1.0f;
        Q_[i][j] = process_noises[i];
      }
    }
  }
  if (!(ros::param::get("est/kalman/RVg",RVg_)))
    ROS_WARN("No param named 'RVg'");
  if (!(ros::param::get("est/kalman/Rchi",Rchi_)))
    ROS_WARN("No param named 'Rchi'");
  if (!(ros::param::get("est/kalman/Racc",Racc_)))
    ROS_WARN("No param named 'Racc'");
}
void KalmanFilter::predict(const ros::TimerEvent& event)
{
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
    f_[3] = c_phi*s_theta*az;  // TODO this model assumes the only force is by propulsion and in the (-)Z direction
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
    A_[0][0] = c_phi*t_theta*q - s_phi*t_theta*r;
    A_[0][1] = (q*s_phi + r*c_phi)/(c_theta*c_theta);
    A_[0][2] = 0.0f;
    A_[0][3] = 0.0f;
    A_[0][4] = 0.0f;
    A_[1][0] = -q*s_phi - r*c_phi;
    A_[1][1] = 0.0f;
    A_[1][2] = 0.0f;
    A_[1][3] = 0.0f;
    A_[1][4] = 0.0f;
    A_[2][0] = c_phi*sec_theta*q - s_phi*sec_theta*r;
    A_[2][1] = (q*s_phi + r*c_phi)*t_theta*sec_theta;
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
    float P1[NUM_STATES][NUM_STATES];
    for (int i = 0; i < NUM_STATES; i++)
      for (int j = 0; j < NUM_STATES; j++)
        P1[i][j] = P_[i][j];
    for (int i = 0; i < NUM_STATES; i++)
    {
      for (int j = 0; j < NUM_STATES; j++)
      {
        float AP  = 0.0f;
        float PAt = 0.0f;
        for (int k = 0; k < NUM_STATES; k++)
        {
          AP  += A_[i][k]*P1[k][j];
          PAt += P1[i][k]*A_[j][k];
        }
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
  // TODO put altitude in this model inversion?
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
}
void KalmanFilter::correctGPS()
{
  // // Measurements
  // // gps_N_, gps_E_, gps_D_, gps_speed_, gps_ground_course_;
  // float psi, u1, v1;
  // float s_psi, c_psi;
  // psi        = xhat_[2];
  // u1         = xhat_[3];
  // v1         = xhat_[4];
  // c_psi      = cosf(psi);
  // s_psi      = sinf(psi);
  // float hVg  = sqrtf(u1*u1 + v1*v1);
  // float hchi = atan2f(u1*s_psi + v1*c_psi, u1*c_psi - v1*s_psi);
  // float Vg_correction  = gps_speed_ - hVg;
  // float chi_correction = gps_ground_course_ - hchi;
  // // can't allow chi_correction to become huge, fmod?
  //
  // float C[NUM_STATES]; // dh/dx for Vg
  // C[0] = 0.0f;
  // C[1] = 0.0f;
  // C[2] = 0.0f;
  // C[3] = u1/sqrtf(u1*u1 + v1*v1);
  // C[4] = v1/sqrtf(u1*u1 + v1*v1);
  // // K_    = P_*C_.'*inv(R_ + C_*P_*C_.');
  // float PCt[NUM_STATES];
  // for (int i = 0; i < NUM_STATES; i++)
  // {
  //   float pc = 0.0f;
  //   for (int j = 0; j < NUM_STATES; j++)
  //     pc += P_[i][j]*C[j];
  //   PCt[i] = pc;
  // }
  // float R_p_CPCt = RVg_; // *****************
  // for (int i = 0; i < NUM_STATES; i++)
  //   R_p_CPCt += C[i]*PCt[i];
  //
  // for (int i = 0; i < NUM_STATES; i++)
  // {
  //   float ki = 0.0f;
  //   for (int j = 0; j < NUM_STATES; j++)
  //     ki += P_[i][j]*C[j];
  //   K_[i] = ki/R_p_CPCt;
  // }
  //
  // // P_    = (eye(5) - K_*C_)*P_;
  // float ImKC[NUM_STATES][NUM_STATES];
  // for (int i = 0; i < NUM_STATES; i++)
  // {
  //   for (int j = 0; j < NUM_STATES; j++)
  //   {
  //     if (i == j)
  //     {
  //       ImKC[i][j] = 1.0f - K_[i]*C[j];
  //     }
  //     else
  //     {
  //       ImKC[i][j] = 0.0f - K_[i]*C[j];
  //     }
  //   }
  // }
  // float P2[NUM_STATES][NUM_STATES];
  // for (int i = 0; i < NUM_STATES; i++)
  // {
  //   for (int j = 0; j < NUM_STATES; j++)
  //   {
  //     float ImKCP  = 0.0f;
  //     for (int k = 0; k < NUM_STATES; k++)
  //     {
  //       ImKCP  += ImKC[i][k]*P_[k][j];
  //     }
  //     P2[i][j] = ImKCP;
  //   }
  // }
  // P_ = P2;
  // for (int i = 0; i < NUM_STATES; i++)
  //   xhat_[i] += K_[i]*Vg_correction;
  //
  // C[0] = 0.0f; // dh/dx for chi
  // C[1] = 0.0f;
  // C[2] = 1.0f/(1.0f + powf(((u1*s_psi + v1*c_psi)/(u1*c_psi - v1*s_psi)),2.0f))*((u1*c_psi - v1*s_psi)*(u1*c_psi - \
  //        v1*s_psi) - (-u1*s_psi - v1*c_psi)*(u1*s_psi + v1*c_psi))/powf(u1*c_psi - v1*s_psi, 2.0f);
  // C[3] = 1.0f/(1.0f + powf((u1*s_psi + v1*c_psi)/(u1*c_psi - v1*s_psi), 2.0f))*(s_psi*(u1*c_psi - v1*s_psi)\
  //        - c_psi*(u1*s_psi + v1*c_psi))/powf(u1*c_psi - v1*s_psi, 2.0f);
  // C[4] = 1.0f/(1.0f + powf((u1*s_psi + v1*c_psi)/(u1*c_psi - v1*s_psi), 2.0f))*(c_psi*(u1*c_psi - v1*s_psi)\
  //        - (-s_psi)*(u1*s_psi + v1*c_psi))/powf(u1*c_psi - v1*s_psi, 2.0f);
  //
  // // K_    = P_*C_.'*inv(R_ + C_*P_*C_.');
  // for (int i = 0; i < NUM_STATES; i++)
  // {
  //   float pc = 0.0f;
  //   for (int j = 0; j < NUM_STATES; j++)
  //     pc += P_[i][j]*C[j];
  //   PCt[i] = pc;
  // }
  // R_p_CPCt = Rchi_; // *****************
  // for (int i = 0; i < NUM_STATES; i++)
  //   R_p_CPCt += C[i]*PCt[i];
  // for (int i = 0; i < NUM_STATES; i++)
  // {
  //   float ki = 0.0f;
  //   for (int j = 0; j < NUM_STATES; j++)
  //     ki += P_[i][j]*C[j];
  //   K_[i] = ki/R_p_CPCt;
  // }
  // // P_    = (eye(5) - K_*C_)*P_;
  // for (int i = 0; i < NUM_STATES; i++)
  // {
  //   for (int j = 0; j < NUM_STATES; j++)
  //   {
  //     if (i == j)
  //     {
  //       ImKC[i][j] = 1.0f - K_[i]*C[j];
  //     }
  //     else
  //     {
  //       ImKC[i][j] = 0.0f - K_[i]*C[j];
  //     }
  //   }
  // }
  // for (int i = 0; i < NUM_STATES; i++)
  // {
  //   for (int j = 0; j < NUM_STATES; j++)
  //   {
  //     float ImKCP  = 0.0f;
  //     for (int k = 0; k < NUM_STATES; k++)
  //     {
  //       ImKCP  += ImKC[i][k]*P_[k][j];
  //     }
  //     P2[i][j] = ImKCP;
  //   }
  // }
  // P_ = P2;
  // // for (int i = 0; i < NUM_STATES; i++)
  // //   xhat_[i] += K_[i]*chi_correction;
  //
  //
  // // Pull in xhat variables into state_hat_
  // state_hat_.phi   = xhat_[0];
  // state_hat_.theta = xhat_[1];
  // state_hat_.psi   = xhat_[2];
  // state_hat_.u1    = xhat_[3];
  // state_hat_.v1    = xhat_[4];
  // state_hat_publisher_.publish(state_hat_.msg());
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

  float C[NUM_STATES]; // dh/dx
  C[0] = 0.0f;
  C[1] = g_*c_theta;
  C[2] = 0.0f;
  C[3] = 0.0f;
  C[4] = 0.0f;

  // K_    = P_*C_.'*inv(R_ + C_*P_*C_.');
  float PCt[NUM_STATES];
  for (int i = 0; i < NUM_STATES; i++)
  {
    float pc = 0.0f;
    for (int j = 0; j < NUM_STATES; j++)
      pc += P_[i][j]*C[j];
    PCt[i] = pc;
  }
  float R_p_CPCt = Racc_; // *****************
  for (int i = 0; i < NUM_STATES; i++)
    R_p_CPCt += C[i]*PCt[i];

  for (int i = 0; i < NUM_STATES; i++)
  {
    float ki = 0.0f;
    for (int j = 0; j < NUM_STATES; j++)
      ki += P_[i][j]*C[j];
    K_[i] = ki/R_p_CPCt;
  }

  // P_    = (eye(5) - K_*C_)*P_;
  float ImKC[NUM_STATES][NUM_STATES];
  for (int i = 0; i < NUM_STATES; i++)
  {
    for (int j = 0; j < NUM_STATES; j++)
    {
      if (i == j)
      {
        ImKC[i][j] = 1.0f - K_[i]*C[j];
      }
      else
      {
        ImKC[i][j] = 0.0f - K_[i]*C[j];
      }
    }
  }
  float P2[NUM_STATES][NUM_STATES];
  for (int i = 0; i < NUM_STATES; i++)
  {
    for (int j = 0; j < NUM_STATES; j++)
    {
      float ImKCP  = 0.0f;
      for (int k = 0; k < NUM_STATES; k++)
      {
        ImKCP  += ImKC[i][k]*P_[k][j];
      }
      P2[i][j] = ImKCP;
    }
  }
  for (int i = 0; i < NUM_STATES; i++)
    for (int j = 0; j < NUM_STATES; j++)
      P_[i][j] = P2[i][j];

  // xhat_ = xhat_ + K_*(Sensor - h_);
  for (int i = 0; i < NUM_STATES; i++)
    xhat_[i] += K_[i]*x_correction;

  C[0] = -g_*c_theta*c_phi;
  C[1] =  g_*s_theta*s_phi;
  C[2] = 0.0f;
  C[3] = 0.0f;
  C[4] = 0.0f;

  // K_    = P_*C_.'*inv(R_ + C_*P_*C_.');
  for (int i = 0; i < NUM_STATES; i++)
  {
    float pc = 0.0f;
    for (int j = 0; j < NUM_STATES; j++)
      pc += P_[i][j]*C[j];
    PCt[i] = pc;
  }
  R_p_CPCt = Racc_; // *****************
  for (int i = 0; i < NUM_STATES; i++)
    R_p_CPCt += C[i]*PCt[i];

  for (int i = 0; i < NUM_STATES; i++)
  {
    float ki = 0.0f;
    for (int j = 0; j < NUM_STATES; j++)
      ki += P_[i][j]*C[j];
    K_[i] = ki/R_p_CPCt;
  }

  // P_    = (eye(5) - K_*C_)*P_;
  for (int i = 0; i < NUM_STATES; i++)
  {
    for (int j = 0; j < NUM_STATES; j++)
    {
      if (i == j)
      {
        ImKC[i][j] = 1.0f - K_[i]*C[j];
      }
      else
      {
        ImKC[i][j] = 0.0f - K_[i]*C[j];
      }
    }
  }
  for (int i = 0; i < NUM_STATES; i++)
  {
    for (int j = 0; j < NUM_STATES; j++)
    {
      float ImKCP  = 0.0f;
      for (int k = 0; k < NUM_STATES; k++)
      {
        ImKCP  += ImKC[i][k]*P_[k][j];
      }
      P2[i][j] = ImKCP;
    }
  }
  for (int i = 0; i < NUM_STATES; i++)
    for (int j = 0; j < NUM_STATES; j++)
      P_[i][j] = P2[i][j];

  // xhat_ = xhat_ + K_*(Sensor - h_);
  for (int i = 0; i < NUM_STATES; i++)
    xhat_[i] += K_[i]*y_correction;

  C[0] = g_*c_theta*s_phi;
  C[1] = g_*s_theta*c_phi;
  C[2] = 0.0f;
  C[3] = 0.0f;
  C[4] = 0.0f;

  // K_    = P_*C_.'*inv(R_ + C_*P_*C_.');
  for (int i = 0; i < NUM_STATES; i++)
  {
    float pc = 0.0f;
    for (int j = 0; j < NUM_STATES; j++)
      pc += P_[i][j]*C[j];
    PCt[i] = pc;
  }
  R_p_CPCt = Racc_; // *****************
  for (int i = 0; i < NUM_STATES; i++)
    R_p_CPCt += C[i]*PCt[i];

  for (int i = 0; i < NUM_STATES; i++)
  {
    float ki = 0.0f;
    for (int j = 0; j < NUM_STATES; j++)
      ki += P_[i][j]*C[j];
    K_[i] = ki/R_p_CPCt;
  }

  // P_    = (eye(5) - K_*C_)*P_;
  for (int i = 0; i < NUM_STATES; i++)
  {
    for (int j = 0; j < NUM_STATES; j++)
    {
      if (i == j)
      {
        ImKC[i][j] = 1.0f - K_[i]*C[j];
      }
      else
      {
        ImKC[i][j] = 0.0f - K_[i]*C[j];
      }
    }
  }
  for (int i = 0; i < NUM_STATES; i++)
  {
    for (int j = 0; j < NUM_STATES; j++)
    {
      float ImKCP  = 0.0f;
      for (int k = 0; k < NUM_STATES; k++)
      {
        ImKCP  += ImKC[i][k]*P_[k][j];
      }
      P2[i][j] = ImKCP;
    }
  }
  for (int i = 0; i < NUM_STATES; i++)
    for (int j = 0; j < NUM_STATES; j++)
      P_[i][j] = P2[i][j];

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
}
} // end namespace pegasus
