#include "pegasus/pid.h"

namespace pegasus
{
PID::PID()
{
  // SETUP THE CONTROLLER HERE
  // PHI
  if (!(ros::param::get("kP_phi",kP_phi_)))
    ROS_WARN("No param named 'kP_phi'");
  if (!(ros::param::get("kD_phi",kD_phi_)))
    ROS_WARN("No param named 'kD_phi'");
  if (!(ros::param::get("kP_theta",kP_theta_)))
    ROS_WARN("No param named 'kP_theta'");
  if (!(ros::param::get("kD_theta",kD_theta_)))
    ROS_WARN("No param named 'kD_theta'");
  if (!(ros::param::get("kP_psi",kP_psi_)))
    ROS_WARN("No param named 'kP_psi'");
  if (!(ros::param::get("kD_psi",kD_psi_)))
    ROS_WARN("No param named 'kD_psi'");
  sigma_  = 0.5;
  r_last_ = 0.0;

  if (!(ros::param::get("vehicle_description/motors/K1",K1_)))
    ROS_WARN("No param named 'K1");
  if (!(ros::param::get("vehicle_description/motors/K1",K2_)))
    ROS_WARN("No param named 'K2");
  if (!(ros::param::get("vehicle_description/motors/x",x_)))
    ROS_WARN("No param named 'x");
  if (!(ros::param::get("vehicle_description/motors/y",y_)))
    ROS_WARN("No param named 'y");
}
void PID::control(const ros::TimerEvent& event)
{
  ros::Time new_time = ros::Time::now();
  mapControlChannels();
  ros::Duration time_step = new_time - last_time_;
  ts_ = time_step.toSec();

  // THRUST
  float F = thrust_desired_*4.0f*K1_;

  // PHI - PD CONTROL
  float e_phi     = roll_desired_*piD180_ - state_.phi;
  float tau_phi   = kP_phi_*e_phi  - kD_phi_*state_.p;

  // THETA - PD CONTROL
  float e_theta   = pitch_desired_*piD180_ - state_.theta;
  float tau_theta = kP_theta_*e_theta  - kD_theta_*state_.q;

  // PSI - PD Control
  // ROS_INFO("%f",yaw_rate_desired_);
  float rd_       = ((2.0f*sigma_ - ts_)/(2.0f*sigma_ + ts_))*rd_ + (2.0f/(2.0f*sigma_ + ts_))*(state_.r - r_last_);
  float e_psi     = yaw_rate_desired_*piD180_ - state_.r;
  float tau_psi   = kP_psi_*e_psi  - kD_psi_*rd_;

  // OUTPUT
  mixMotors4(F, tau_phi, tau_theta, tau_psi);
  publishMotorCommand();
  publishDesiredCommand();

  // Age Data
  last_time_ = new_time;
  r_last_    = state_.r;
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
