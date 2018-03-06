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

  // PHI
  float e_phi_last_;
  float kP_phi_;
  float kD_phi_;


  // Mixer (TEMP)
  void mixMotors4(float F, float t_phi, float t_theta, float t_psi);
  float K1_;
  float K2_;
  float x_;
  float y_;
};// end class PID
} // end namespace pegasus

#endif // PID_H
