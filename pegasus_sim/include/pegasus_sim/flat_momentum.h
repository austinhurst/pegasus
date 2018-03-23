#ifndef FLAT_MOMENTUM_H
#define FLAT_MOMENTUM_H

#include <math.h>
#include <pegasus/state_struct.h>
#include <pegasus_sim/forces_and_moments_base.h>
#include <pegasus_sim/Wind.h>


namespace pegasus_sim
{
class FlatMomentum : public ForcesAndMoments
{
public:
  FlatMomentum();
  ~FlatMomentum();
private:
  virtual void getForcesAndMoments(pegasus::state_struct s,
                                   float& fx, float& fy, float& fz,
                                   float& l, float& m, float& n);

  //********************* NODE HANDLES *********************//
  ros::NodeHandle nh_;         // public node handle for publishing, subscribing

  //************** SUBSCRIBERS AND PUBLISHERS **************//

  //******************** CLASS VARIABLES *******************//
  float g_;                   // value of gravity
  float mass_;                // mass of vehicle in Kg
  float mg_;                  // mass_*g_
  float rho_;                 // density of air kg/m^3
  float S_;                   // area of the flate plate assumption
  float c_;                   // chord of the flat plate assumption
  float b_;                   // span of the flat plate assumption
  float K_delta_t_;           // RPM/throttle percentage
  float KQ_;                  // Propellor Torque constant
  float Vb_;                  // Battery Voltage
  float Kv_;                  // RPM per Volt
  float Rm_;                  // Resistance, Ohms
  float i0_;                  // No load current, Amps
  float Dp_;                  // Diameter of the propellor, INCHES
  float half_rho_S_;          // 0.5*rho_*S_
  float Ap_;                  // Area of the propellor disk
  float piD30_;               // M_PI/180.0
  bool  flown_;               // true if the vehicle has flown, used for inital ground.

  //***************** CALLBACKS AND TIMERS *****************//

  //********************** FUNCTIONS ***********************//

};// end class FlatMomentum
} // end namespace pegasus_sim

#endif // FLAT_MOMENTUM_H
