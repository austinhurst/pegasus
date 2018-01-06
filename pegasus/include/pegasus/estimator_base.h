#ifndef ESTIMATOR_BASE_H
#define ESTIMATOR_BASE_H

#include <string>
#include <ros/ros.h>
#include <pegasus/MotorCommand4.h>
#include <pegasus/VehicleState.h>
#include <pegasus/state_struct.h>

namespace pegasus
{
class Estimator
{
public:
  Estimator();

private:
  //********************* NODE HANDLES *********************//
  ros::NodeHandle nh_;         // public node handle for publishing, subscribing

  //************** SUBSCRIBERS AND PUBLISHERS **************//
  ros::Subscriber motor_command_subscriber_;
  ros::Subscriber sonor_subscriber_;
  ros::Subscriber gps_subscriber_;
  ros::Subscriber barom_subscriber_;
  ros::Subscriber accel_subscriber_;
  ros::Subscriber gyro_subscriber_;
  ros::Subscriber mag_subscriber_;

  ros::Subscriber truth_subscriber_;  // Only used in simulation when use_truth is true

  ros::Publisher state_hat_publisher_;

  //******************** CLASS VARIABLES *******************//

protected:
  pegasus::state_struct state_hat_;
  ros::Time last_time_;

  // Message Variables
  // motor_command variables
  float m1_;
  float m2_;
  float m3_;
  float m4_;

  // sonor variables

  // gps variables

  // barom variables

  // accel variables

  // gyro variables

  // mag variables
  

private:
  pegasus::VehicleState state_hat_msg_;
  geometry_msgs::TransformStamped odom_trans_;

  //***************** CALLBACKS AND TIMERS *****************//
  void motorCommandCallback(const pegasus::MotorCommand4ConstPtr &msg);
  void propogate(const ros::TimerEvent& event);
  ros::Timer propogate_timer_;
  void updateViz(const ros::WallTimerEvent& event);
  ros::WallTimer update_viz_timer_;

  //********************** FUNCTIONS ***********************//
  void addUncertainty(float* var);
protected:
  virtual pegasus::state_struct derivative(pegasus::state_struct state);

};// end class EquationsOfMotion
} // end namespace pegasus_sim

#endif // EQUATIONS_OF_MOTION_BASE_H
