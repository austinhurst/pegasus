#ifndef FORCES_AND_MOMENTS_H
#define FORCES_AND_MOMENTS_H

#include <math.h>
#include <ros/ros.h>
#include <pegasus/state_struct.h>
#include <pegasus/motor_struct.h>


namespace pegasus_sim
{
class ForcesAndMoments
{
public:
  ForcesAndMoments();
  ~ForcesAndMoments();
  void getForcesAndMoments(pegasus::state_struct s, float& fx, float& fy, float& fz, float& l, float& m, float& n);
private:
  //********************* NODE HANDLES *********************//
  ros::NodeHandle nh_;         // public node handle for publishing, subscribing

  //************** SUBSCRIBERS AND PUBLISHERS **************//
  ros::Subscriber motor_command_subscriber_;
  //******************** CLASS VARIABLES *******************//
protected:
  int num_motors_;            // number of motors
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
  float w_ns_;
  float w_es_;
  float w_ds_;
  float w_xg_;
  float w_yg_;
  float w_zg_;
  float piD30_;

  // Motor descriptions
  pegasus::motor_description m1d_;
  pegasus::motor_description m2d_;
  pegasus::motor_description m3d_;
  pegasus::motor_description m4d_;
  pegasus::motor_description m5d_;
  pegasus::motor_description m6d_;
  pegasus::motor_description m7d_;
  pegasus::motor_description m8d_;

  // Message Variables
protected:
  pegasus::motor_struct *motors_;

  //***************** CALLBACKS AND TIMERS *****************//
  void motorCommandCallback2(const pegasus::MotorCommand2ConstPtr &msg);
  void motorCommandCallback3(const pegasus::MotorCommand3ConstPtr &msg);
  void motorCommandCallback4(const pegasus::MotorCommand4ConstPtr &msg);
  void motorCommandCallback6(const pegasus::MotorCommand6ConstPtr &msg);
  void motorCommandCallback8(const pegasus::MotorCommand8ConstPtr &msg);

  //********************** FUNCTIONS ***********************//
  void initialize_motor(std::string i, pegasus::motor_description *md);
public:
  void eachTimeStep();
};// end class forcesAndMoments
} // end namespace pegasus_sim

#endif // FORCES_AND_MOMENTS_H
