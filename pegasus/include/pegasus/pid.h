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
  double ts_;
  float mass_;
  float g_;

  // PHI
  float kP_phi_;
  float kD_phi_;

  // THETA
  float kP_theta_;
  float kD_theta_;

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


  float kP_v1_;
  float kD_v1_;

  // U
  float u1d_;
  float u1_last_;
  float e_u_last_;

  // V
  float v1d_;
  float v1_last_;
  float e_v_last_;

  // W
  float w1d_;
  float w1_last_;
  float e_w_last_;


  // Mixer (TEMP)
  void mixMotors4(float F, float t_phi, float t_theta, float t_psi);
  float saturate(float value_i, float min, float max);
  float K1_;
  float K2_;
  float x_;
  float y_;
};// end class PID
} // end namespace pegasus

#endif // PID_H
