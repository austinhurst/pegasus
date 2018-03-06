#include "pegasus/pid.h"

namespace pegasus
{
PID::PID()
{
  // SETUP THE CONTROLLER HERE
  // PHI
  e_phi_last_ = 0.0f;
  if (!(ros::param::get("kP_phi",kP_phi_)))
    ROS_WARN("No param named 'kP_phi'");
  if (!(ros::param::get("kD_phi",kD_phi_)))
    ROS_WARN("No param named 'kD_phi'");

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

  // Thurst
  float F = thrust_desired_*4.0f*K1_;

  // PHI - PD CONTROL
  float e_phi     = roll_desired_*piD180_ - state_.phi;
  float tau_phi   = kP_phi_*e_phi  - kD_phi_*state_.p;

  float tau_theta = 0.0f;
  float tau_psi   = 0.0f;
  // tau_phi = roll_desired_/60.0f*0.0001*(4.0f*y_*K1_);
  mixMotors4(F, tau_phi, tau_theta, tau_psi);
  publishMotorCommand();
  publishDesiredCommand();
  last_time_ = new_time;

  // Age Data
  e_phi_last_  = e_phi;
}
void PID::mixMotors4(float F, float t_phi, float t_theta, float t_psi) // TEMP
{
  float d      = F/(4.0f*K1_);                // Start with F      // F = K1*(d1 + d2 + d3 + d4);
  float dphi   = t_phi/(4.0f*y_*K1_);         // Next do tau_phi   // tau_phi = -y*K1(d1 + d2) + y*K1*(d3 + d4);
  float dtheta = t_theta/(4.0f*x_*K1_);       // Then do tau_theta // tau_theta = -x*K1*(d1 + d3) + x*K1*(d2 + d4);
  float dpsi   = t_psi/(4.0f*K2_);            // Then do tau_psi   // tau_psi = -K2*(d1 + d4) + K2*(d2 + d3);
  motors_->m1  =  d - dphi - dtheta - dpsi;
  motors_->m2  =  d - dphi + dtheta + dpsi;
  motors_->m3  =  d + dphi - dtheta + dpsi;
  motors_->m4  =  d + dphi + dtheta - dpsi;
}
} // end namespace pegasus
