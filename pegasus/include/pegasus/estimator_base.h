#ifndef ESTIMATOR_BASE_H
#define ESTIMATOR_BASE_H

#include <string>
#include <ros/ros.h>
#include <tf/LinearMath/Quaternion.h>
#include <tf/transform_datatypes.h>
#include <pegasus/VehicleState.h>
#include <pegasus/state_struct.h>
#include <pegasus/motor_struct.h>
#include <pegasus/gps_struct.h>
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

protected:
  ros::Publisher state_hat_publisher_;

  //******************** CLASS VARIABLES *******************//
  float g_;
  float mass_;
  float K_delta_t_;
  float KQ_;
  float Vb_;
  float Kv_;
  float Rm_;
  float i0_;
  float Dp_;
  float rho_;
  float Ap_;
  float piD30_;

  pegasus::state_struct state_hat_;
  ros::Time last_time_;

  // Message Variables
  // motor_command variables
  motor_struct *motors_;
  int num_motors_;
  // Motor descriptions
  motor_description m1d_;
  motor_description m2d_;
  motor_description m3d_;
  motor_description m4d_;
  motor_description m5d_;
  motor_description m6d_;
  motor_description m7d_;
  motor_description m8d_;
  float fx_p_;
  float fy_p_;
  float fz_p_;
  float l_p_;
  float m_p_;
  float n_p_;

  // sonar variables
  float sonar_distance_;

  // gps variables
  gps_struct gps_converter_;
  float gps_N_;
  float gps_E_;
  float gps_D_;
  bool  gps_fix_;
  float gps_speed_;
  float gps_ground_course_;

  // barometer variables
  float barometer_pressure_;

  // imu variables
  tf::Quaternion imu_orientation_;
  float imu_orientation_covariance_[9];
  float imu_angular_velocity_[3];
  float imu_angular_velocity_covariance_[9];
  float imu_linear_acceleration_[3];
  float imu_linear_acceleration_covariance_[9];

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
  virtual void predict(const ros::TimerEvent& event);
  virtual void correctBarometer();
  virtual void correctGPS();
  virtual void correctIMU();
  ros::Timer estimate_timer_;
  //********************** FUNCTIONS ***********************//
private:
  void initializeMotor(std::string i, pegasus::motor_description *md);
protected:
  void getF();
};// end class Estimator
} // end namespace pegasus

#endif // ESTIMATOR_BASE_H
