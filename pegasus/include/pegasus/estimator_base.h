#ifndef ESTIMATOR_BASE_H
#define ESTIMATOR_BASE_H

#include <string>
#include <ros/ros.h>
#include <tf/LinearMath/Quaternion.h>
#include <tf/transform_datatypes.h>
#include <pegasus/VehicleState.h>
#include <pegasus/state_struct.h>
#include <pegasus/motor_struct.h>
#include <pegasus/Sonar.h>
#include <pegasus/GPS.h>
#include <pegasus/Barometer.h>
#include <sensor_msgs/Imu.h>

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
  ros::Subscriber sonar_subscriber_;
  ros::Subscriber gps_subscriber_;
  ros::Subscriber barometer_subscriber_;
  ros::Subscriber imu_subscriber_;

  ros::Subscriber truth_subscriber_;  // Only used in simulation when use_truth is true

protected:
  ros::Publisher state_hat_publisher_;

  //******************** CLASS VARIABLES *******************//
  pegasus::state_struct state_hat_;
  ros::Time last_time_;

  // Message Variables
  // motor_command variables
  motor_struct *motors_;

  // sonar variables
  float sonar_distance_;

  // gps variables
  bool   gps_fix_;
  int    gps_NumSat_;
  double gps_latitude_;
  double gps_longitude_;
  double gps_altitude_;
  double gps_speed_;
  double gps_ground_course_;
  double gps_covariance_;

  // barometer variables
  float barometer_pressure_;

  // imu variables
  tf::Quaternion imu_orientation_;
  double imu_orientation_covariance_[9];
  double imu_angular_velocity_[3];
  double imu_angular_velocity_covariance_[9];
  double imu_linear_acceleration_[3];
  double imu_linear_acceleration_covariance_[9];


private:
  pegasus::VehicleState state_hat_msg_;

  //***************** CALLBACKS AND TIMERS *****************//
  void motorCommandCallback2(const MotorCommand2ConstPtr &msg);
  void motorCommandCallback3(const MotorCommand3ConstPtr &msg);
  void motorCommandCallback4(const MotorCommand4ConstPtr &msg);
  void motorCommandCallback6(const MotorCommand6ConstPtr &msg);
  void motorCommandCallback8(const MotorCommand8ConstPtr &msg);

  void sonarCallback(const SonarConstPtr &msg);
  void gpsCallback(const GPSConstPtr &msg);
  void barometerCallback(const BarometerConstPtr &msg);
  void imuCallback(const sensor_msgs::ImuConstPtr &msg);

protected:
  virtual void estimate(const ros::TimerEvent& event);
  ros::Timer estimate_timer_;

protected:
  void truthCallback(const VehicleStateConstPtr &msg);            // Only used in simulation when use_truth is true

  //********************** FUNCTIONS ***********************//


};// end class Estimator
} // end namespace pegasus

#endif // ESTIMATOR_BASE_H
