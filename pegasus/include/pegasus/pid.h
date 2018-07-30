#ifndef PID_H
#define PID_H

#include <pegasus/controller_base.h>

namespace pegasus
{
class PID : public Controller
{
public:
  PID();

private:
  virtual void control(const ros::TimerEvent& event);
  float wrapAngle(float angle_in);
  double ts_;
  float mass_;
  float g_;

  // PHI
  float kP_phi_;
  float kD_phi_;
  float kP_p_;
  float kD_p_;
  float p_last_;
  float pd_;

  // THETA
  float kP_theta_;
  float kD_theta_;
  float kP_q_;
  float kD_q_;
  float q_last_;
  float qd_;

  // PSI
  float kP_psi_;
  float kD_psi_;
  float sigma_;
  float r_last_;
  float rd_;

  // HEIGHT
  float kP_h_;
  float kI_h_;
  float kD_h_;
  float Fe_;
  float hd_;
  float h_last_;
  float h_integration_;
  float e_height_last_;
  float min_thrust_;
  float max_thrust_;



  float kP_v1_;
  float kD_v1_;

  // U1
  float u1d_;
  float u1_last_;
  float e_u_last_;

  // V1
  float v1d_;
  float v1_last_;
  float e_v_last_;

  float max_tilt_;
  float sin_max_tilt_;
  float alt_hold_;

  // Mixer (TEMP)
  void mixMotors4(float F, float t_phi, float t_theta, float t_psi);
  float saturate(float value_i, float min, float max);
  float K1_;  // Thrust constant
  float K2_;  // Torque constant
  float x_;   // x position of motors
  float y_;   // y position of motors
};// end class PID
} // end namespace pegasus

#endif // PID_H
