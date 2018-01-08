#ifndef ESTIMATOR_BASE_H
#define ESTIMATOR_BASE_H

#include <string>
#include <ros/ros.h>
#include <pegasus/VehicleState.h>
#include <pegasus/state_struct.h>
#include <pegasus/motor_struct.h>

namespace pegasus
{
class Estimator
{
public:
  Estimator();
  ~Estimator();
private:
  //********************* NODE HANDLES *********************//
  ros::NodeHandle nh_;         // public node handle for publishing, subscribing

  //************** SUBSCRIBERS AND PUBLISHERS **************//
  ros::Subscriber motor_command_subscriber_;
  // ros::Subscriber sonor_subscriber_;
  // ros::Subscriber gps_subscriber_;
  // ros::Subscriber barom_subscriber_;
  // ros::Subscriber accel_subscriber_;
  // ros::Subscriber gyro_subscriber_;
  // ros::Subscriber mag_subscriber_;

  ros::Subscriber truth_subscriber_;  // Only used in simulation when use_truth is true
protected:
  ros::Publisher state_hat_publisher_;

  //******************** CLASS VARIABLES *******************//
  pegasus::state_struct state_hat_;
  ros::Time last_time_;

  // Message Variables
  // motor_command variables
  motor_struct *motors_;

  // sonor variables

  // gps variables

  // barom variables

  // accel variables

  // gyro variables

  // mag variables


private:
  pegasus::VehicleState state_hat_msg_;

  //***************** CALLBACKS AND TIMERS *****************//
  void motorCommandCallback2(const MotorCommand2ConstPtr &msg);
  void motorCommandCallback3(const MotorCommand3ConstPtr &msg);
  void motorCommandCallback4(const MotorCommand4ConstPtr &msg);
  void motorCommandCallback6(const MotorCommand6ConstPtr &msg);
  void motorCommandCallback8(const MotorCommand8ConstPtr &msg);

  // void sonorCallback(const SonorConstPtr &msg);
  // void gpsCallback(const GPSConstPtr &msg);
  // void baromCallback(const BaromConstPtr &msg);
  // void accelCallback(const AccelConstPtr &msg);
  // void gyroCallback(const GyroConstPtr &msg);
  // void magCallback(const MagConstPtr &msg);

protected:
  virtual void estimate(const ros::TimerEvent& event);
  ros::Timer estimate_timer_;

protected:
  void truthCallback(const VehicleStateConstPtr &msg);            // Only used in simulation when use_truth is true

  //********************** FUNCTIONS ***********************//


};// end class Estimator
} // end namespace pegasus

#endif // ESTIMATOR_BASE_H
