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

  ud_            = vd_            = wd_            = 0.0;
  u_last_        = v_last_        = w_last_        = 0.0;
  u_integration_ = v_integration_ = w_integration_ = 0.0;
  e_u_last_      = e_v_last_      = e_w_last_      = 0.0;

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
    hd_ =  ((2.0f*sigma_ - ts_)/(2.0f*sigma_ + ts_))*hd_ + (2.0f/(2.0f*sigma_ + ts_))*(-state_.pd - h_last_);
    h_integration_ = h_integration_ + ts_/2.0*(e_height + e_height_last_);
    F = kP_h_*e_height - kD_h_*hd_ + kI_h_*h_integration_ + Fe_;
    e_height_last_ = e_height;

    // Turn Vg_desired and chi_desired into u_desired and v_desired, w_desired // rotation
    float c_phi   = cosf(state_.phi);
    float s_phi   = sinf(state_.phi);
    float c_theta = cosf(state_.theta);
    float s_theta = sinf(state_.theta);
    float c_psi   = cosf(state_.psi);
    float s_psi   = sinf(state_.psi);
    float Rb_v[3][3];
    Rb_v[0][0] =  c_theta*c_psi;
    Rb_v[0][1] =  s_phi*s_theta*c_psi - c_phi*s_psi;
    Rb_v[0][2] =  c_phi*s_theta*c_psi + s_phi*s_psi;
    Rb_v[1][0] =  c_theta*s_psi;
    Rb_v[1][1] =  s_phi*s_theta*s_psi + c_phi*c_psi;
    Rb_v[1][2] =  c_phi*s_theta*s_psi - s_phi*c_psi;
    Rb_v[2][0] = -s_theta;
    Rb_v[2][1] =  s_phi*c_theta;
    Rb_v[2][2] =  c_phi*c_theta;
    float Vn_d = Vg_desired_*cosf(chi_desired_);
    float Ve_d = Vg_desired_*sinf(chi_desired_);
    float Vd_d = 0.0f;
    float u_d  = Rb_v[0][0]*Vn_d + Rb_v[1][0]*Ve_d + Rb_v[2][0]*Vd_d;
    float v_d  = Rb_v[0][1]*Vn_d + Rb_v[1][1]*Ve_d + Rb_v[2][1]*Vd_d;
    float w_d  = Rb_v[0][2]*Vn_d + Rb_v[1][2]*Ve_d + Rb_v[2][2]*Vd_d;

    // Turn u, v, w, desired into accelerations (VEHICLE-1 FRAME) desired // controller
    float ax_d, ay_d, az_d;
    // U - PID CONTROL
    float e_u = u_d - state_.u;
    ud_ =  ((2.0f*sigma_ - ts_)/(2.0f*sigma_ + ts_))*ud_ + (2.0f/(2.0f*sigma_ + ts_))*(state_.u - u_last_);
    u_integration_ = u_integration_ + ts_/2.0*(e_u + e_u_last_);
    ax_d = kP_u_*e_u - kD_u_*ud_ + kI_u_*u_integration_;
    e_u_last_ = e_u;

    // V - PID CONTROL
    float e_v = v_d - state_.v;
    vd_ =  ((2.0f*sigma_ - ts_)/(2.0f*sigma_ + ts_))*vd_ + (2.0f/(2.0f*sigma_ + ts_))*(state_.v - v_last_);
    v_integration_ = v_integration_ + ts_/2.0*(e_v + e_v_last_);
    ay_d = kP_v_*e_v - kD_v_*vd_ + kI_v_*v_integration_;
    e_v_last_ = e_v;

    // W - PID CONTROL
    float e_w = w_d - state_.w;
    wd_ =  ((2.0f*sigma_ - ts_)/(2.0f*sigma_ + ts_))*wd_ + (2.0f/(2.0f*sigma_ + ts_))*(state_.w - w_last_);
    w_integration_ = w_integration_ + ts_/2.0*(e_w + e_w_last_);
    az_d = kP_w_*e_w - kD_w_*wd_ + kI_w_*w_integration_;
    e_w_last_ = e_w;

    // Take acc_desired and turn it into angles desired // model inversion SUM(F) = ma
    // F = (g_ - az_d)/(c_phi*c_theta);
    roll_desired_  = atan2f(ay_d*c_theta, g_ - az_d);
    pitch_desired_ = atan2f(ax_d, az_d - g_);
    // then fall through to the manual controls
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
  last_time_     =  new_time;
  r_last_        =  state_.r;
  h_last_        = -state_.pd;
  u_last_        =  state_.u;
  v_last_        =  state_.v;
  w_last_        =  state_.w;
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
