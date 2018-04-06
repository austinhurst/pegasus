#include "pegasus/pid.h"

namespace pegasus
{
PID::PID()
{
  // SETUP THE CONTROLLER HERE
  getRosParam("kP_phi",kP_phi_);
  getRosParam("kD_phi",kD_phi_);
  getRosParam("kP_theta",kP_theta_);
  getRosParam("kD_theta",kD_theta_);
  getRosParam("kP_psi",kP_psi_);
  getRosParam("kD_psi",kD_psi_);
  getRosParam("kP_h",kP_h_);
  getRosParam("kD_h",kD_h_);
  getRosParam("kI_h",kI_h_);
  getRosParam("kP_v1",kP_v1_);
  getRosParam("kD_v1",kD_v1_);

  getRosParam("vehicle_description/mass",mass_);
  getRosParam("vehicle_description/g",g_);
  sigma_         = 0.05;
  r_last_        = 0.0;
  rd_            = 0.0;
  Fe_            = mass_*g_;
  hd_            = 0.0;
  h_last_        = 0.0;
  h_integration_ = 0.0;
  e_height_last_ = 0.0;

  u1d_           = v1d_           = w1d_           = 0.0;
  u1_last_       = v1_last_       = w1_last_       = 0.0;

  getRosParam("vehicle_description/motors/K1",K1_);
  getRosParam("vehicle_description/motors/K1",K2_);
  getRosParam("vehicle_description/motors/x",x_);
  getRosParam("vehicle_description/motors/y",y_);
}
void PID::control(const ros::TimerEvent& event)
{
  ros::Time new_time = ros::Time::now();
  mapControlChannels();
  ros::Duration time_step = new_time - last_time_;
  ts_ = time_step.toSec();

  // THRUST
  float F;
  if (flight_mode_ == VELOC_MODE)
  {
    // HEIGHT - PID CONTROL
    float e_height = H_desired_ - (-state_.pd);
    bool saturated = false;
    if (e_height > 10.0)
    {
      e_height = 10.0;
      saturated = true;
    }
    if (e_height < -10.0)
    {
      e_height = -10.0;
      saturated = true;
    }
    hd_ =  ((2.0f*sigma_ - ts_)/(2.0f*sigma_ + ts_))*hd_ + (2.0f/(2.0f*sigma_ + ts_))*(-state_.pd - h_last_);
    h_integration_ = h_integration_ + ts_/2.0*(e_height + e_height_last_);
    if (saturated)
      h_integration_ = 0.0f;
    F = saturate(kP_h_*e_height - kD_h_*hd_ + kI_h_*h_integration_ + Fe_, 0.15*num_motors_*K1_, num_motors_*K1_);
    e_height_last_ = e_height;

    // float Vd_d;
    // float e_height = H_desired_ - (-state_.pd);
    // if (abs(e_height) < 5.0)
    // {
    //   hd_  =  ((2.0f*sigma_ - ts_)/(2.0f*sigma_ + ts_))*hd_ + (2.0f/(2.0f*sigma_ + ts_))*(-state_.pd - h_last_);
    //   Vd_d = -(kP_h_*e_height - kD_h_*hd_);
    // }
    // else
    //   Vd_d = e_height > 0.0f ? -5.0f : 5.0f;
    // if (std::isfinite(Vd_d) == false)
    //   ros::shutdown();

    // Turn Vg_desired and chi_desired into the vehicle-1 frame, u1_desired and v1_desired, w1_desired // rotation
    float c_phi   = cosf(state_.phi);
    float s_phi   = sinf(state_.phi);
    float c_theta = cosf(state_.theta);
    float s_theta = sinf(state_.theta);
    float c_psi   = cosf(state_.psi);
    float s_psi   = sinf(state_.psi);
    float Vn_d    =  Vg_desired_*cosf(chi_desired_);
    float Ve_d    =  Vg_desired_*sinf(chi_desired_);
    float u1_d    =  Vn_d*c_psi + Ve_d*s_psi;
    float v1_d    = -Vn_d*s_psi + Ve_d*c_psi;
    // float w1_d    =  Vd_d;

    // Rotate u, v, w into vehicle-1 frame
    float u1 =  c_theta*state_.u + s_theta*s_phi*state_.v + s_theta*c_phi*state_.w;
    float v1 =                             c_phi*state_.v -         s_phi*state_.w;
    float w1 = -s_theta*state_.u + c_theta*s_phi*state_.v + c_theta*c_phi*state_.w;
    float a1, a2, a3;
    float e_u1, e_v1, e_w1;

    // U1 - PD CONTROL
    e_u1 = u1_d - u1;
    u1d_ = ((2.0f*sigma_ - ts_)/(2.0f*sigma_ + ts_))*u1d_ + (2.0f/(2.0f*sigma_ + ts_))*(u1 - u1_last_);
    a1   = kP_v1_*e_u1 - kD_v1_*u1d_;

    // V1 - PD CONTROL
    e_v1 = v1_d - v1;
    v1d_ = ((2.0f*sigma_ - ts_)/(2.0f*sigma_ + ts_))*v1d_ + (2.0f/(2.0f*sigma_ + ts_))*(v1 - v1_last_);
    a2   = kP_v1_*e_v1 - kD_v1_*v1d_;

    // W1 - PD CONTROL
    // e_w1 = w1_d - w1;
    // w1d_ =  ((2.0f*sigma_ - ts_)/(2.0f*sigma_ + ts_))*w1d_ + (2.0f/(2.0f*sigma_ + ts_))*(w1 - w1_last_);
    // a3   = kP_v1_*e_w1 - kD_v1_*w1d_;
    // a3   = a3 > g_*0.8 ? g_*0.8 : a3;  // Can't let a3 be more than gravity!

    // ROS_WARN("a %f %f", a1, a2);
    // Take acc_desired and turn it into angles desired // model inversion SUM(F) = ma
    // F              = saturate(mass_*(g_ - a3)/(c_theta*c_phi), 0.15*num_motors_*K1_, num_motors_*K1_);  // use the most recent phi and theta
    float max_roll = 45.0f*piD180_;
    float max_pitch = 45.0f*piD180_;
    float max_inverse_roll = sin(max_roll);
    float max_inverse_pitch = sin(max_pitch);
    a2 = saturate(a2,-F*max_inverse_roll/mass_, F*max_inverse_roll/mass_);
    a1 = saturate(a1,-F*max_inverse_pitch*cos(roll_desired_*piD180_)/mass_, F*max_inverse_roll/mass_);
    roll_desired_  = asinf(mass_*a2/F)/piD180_;
    float max_a1 = fabsf(F*max_inverse_pitch*cos(roll_desired_*piD180_)/mass_);
    a1 = saturate(a1,-max_a1, max_a1);
    pitch_desired_ = asinf(-a1*mass_/(F*cosf(roll_desired_*piD180_)))/piD180_;
    // ROS_WARN("F: %f phi_d: %f theta_d: %f", F, roll_desired_, pitch_desired_);
    // then fall through to the manual controls

    u1_last_   =  u1;
    v1_last_   =  v1;
    w1_last_   =  w1;
    h_last_    = -state_.pd;
  }
  else
    F = thrust_desired_*num_motors_*K1_;

  // PHI - PD CONTROL
  float e_phi     = roll_desired_*piD180_ - state_.phi;
  float tau_phi   = kP_phi_*e_phi  - kD_phi_*state_.p;

  // THETA - PD CONTROL
  float e_theta   = pitch_desired_*piD180_ - state_.theta;
  float tau_theta = kP_theta_*e_theta  - kD_theta_*state_.q;

  // PSI - PD Control
  rd_             = ((2.0f*sigma_ - ts_)/(2.0f*sigma_ + ts_))*rd_ + (2.0f/(2.0f*sigma_ + ts_))*(state_.r - r_last_);
  float e_psi     = yaw_rate_desired_*piD180_ - state_.r;
  float tau_psi   = kP_psi_*e_psi  - kD_psi_*rd_;

  // OUTPUT
  mixMotors4(F, tau_phi, tau_theta, tau_psi);
  publishMotorCommand();
  publishDesiredCommand();

  // Age Data
  last_time_ =  new_time;
  r_last_    =  state_.r;
}
void PID::mixMotors4(float F, float t_phi, float t_theta, float t_psi) // TEMP
{
  float d      = F/(4.0f*K1_);                // Start with F      // F = K1*(d1 + d2 + d3 + d4);
  float dphi   = t_phi/(4.0f*y_*K1_);         // Next do tau_phi   // tau_phi = -y*K1(d1 + d2) + y*K1*(d3 + d4);
  float dtheta = t_theta/(4.0f*x_*K1_);       // Then do tau_theta // tau_theta = -x*K1*(d1 + d3) + x*K1*(d2 + d4);
  float dpsi   = t_psi/(4.0f*K2_);            // Then do tau_psi   // tau_psi = -K2*(d1 + d4) + K2*(d2 + d3);
  motors_->m1  = saturate(d - dphi - dtheta - dpsi, 0.0f, 1.0f);
  motors_->m2  = saturate(d - dphi + dtheta + dpsi, 0.0f, 1.0f);
  motors_->m3  = saturate(d + dphi - dtheta + dpsi, 0.0f, 1.0f);
  motors_->m4  = saturate(d + dphi + dtheta - dpsi, 0.0f, 1.0f);
}
float PID::saturate(float value_i, float min, float max)
{
  float  value_o  = value_i < min ? min : value_i;
  return value_o  = value_o > max ? max : value_o;
}
} // end namespace pegasus
