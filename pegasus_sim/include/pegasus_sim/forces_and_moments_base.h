#ifndef FORCES_AND_MOMENTS_BASE_H
#define FORCES_AND_MOMENTS_BASE_H

#include <ros/ros.h>
#include <pegasus/motor_struct.h>
#include <pegasus/state_struct.h>
#include <pegasus_sim/Wind.h>

namespace pegasus_sim
{
class ForcesAndMoments
{
public:
  ForcesAndMoments();
  ~ForcesAndMoments();
  virtual void getForcesAndMoments(pegasus::state_struct s,
                                   float& fx, float& fy, float& fz,
                                   float& l, float& m, float& n);
private:
  //********************* NODE HANDLES *********************//
  ros::NodeHandle nh_;         // public node handle for publishing, subscribing

  //************** SUBSCRIBERS AND PUBLISHERS **************//
  ros::Subscriber motor_command_subscriber_;
  ros::Subscriber wind_subscriber_;

  //******************** CLASS VARIABLES *******************//
protected:
  // Wind Variables
  float w_ns_;                // Wind north steady velocity
  float w_es_;                // Wind east steady velocity
  float w_ds_;                // Wind down steady velocity
  float w_xg_;                // wind x gust velocity
  float w_yg_;                // Wind y gust velocity
  float w_zg_;                // Wind z gust velocity

  // Motor Variables
  int num_motors_;            // number of motors

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
  pegasus::motor_struct *motors_;

  //***************** CALLBACKS AND TIMERS *****************//
  void motorCommandCallback2(const pegasus::MotorCommand2ConstPtr &msg);
  void motorCommandCallback3(const pegasus::MotorCommand3ConstPtr &msg);
  void motorCommandCallback4(const pegasus::MotorCommand4ConstPtr &msg);
  void motorCommandCallback6(const pegasus::MotorCommand6ConstPtr &msg);
  void motorCommandCallback8(const pegasus::MotorCommand8ConstPtr &msg);
  void windCallback(const pegasus_sim::WindConstPtr &msg);

  //********************** FUNCTIONS ***********************//
  void initializeMotor(std::string i, pegasus::motor_description *md);

};// end class ForcesAndMoments
} // end namespace pegasus_sim

#endif // FORCES_AND_MOMENTS_BASE_H
