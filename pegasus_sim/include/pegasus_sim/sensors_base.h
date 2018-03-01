#ifndef SENSORS_BASE_H
#define SENSORS_BASE_H

#include <stdlib.h>
#include <math.h>
#include <ros/ros.h>
#include <pegasus_sim/forces_and_moments_base.h>
#include <pegasus/gps_struct.h>
#include <pegasus/VehicleState.h>
#include <pegasus/state_struct.h>
#include <pegasus/Sonar.h>
#include <pegasus/GPS.h>
#include <pegasus/Barometer.h>
#include <sensor_msgs/Imu.h>
#include <geometry_msgs/Quaternion.h>

namespace pegasus_sim
{
class SimSensors
{
public:
  SimSensors();
  ~SimSensors();
private:
  //********************* NODE HANDLES *********************//
  ros::NodeHandle nh_;         // public node handle for publishing, subscribing

  //************** SUBSCRIBERS AND PUBLISHERS **************//
  ros::Subscriber truth_subscriber_;
protected:
  ros::Publisher sonar_publisher_;
  ros::Publisher gps_publisher_;
  ros::Publisher imu_publisher_;
  ros::Publisher barometer_publisher_;

  //******************** CLASS VARIABLES *******************//
  // Message Variables
  pegasus::state_struct truth_;
  pegasus::gps_struct gps_converter_;
  pegasus::Sonar sonar_msg_;
  pegasus::GPS gps_msg_;
  pegasus::Barometer barometer_msg_;
  sensor_msgs::Imu imu_msg_;
  ForcesAndMoments *f_and_m_obj_;

private:
  //***************** CALLBACKS AND TIMERS *****************//
  void truthCallback(const pegasus::VehicleState &msg);

protected:
  virtual void sendSonar(const ros::TimerEvent& event);
  ros::Timer sonar_timer_;
  virtual void sendIMU(const ros::TimerEvent& event);
  ros::Timer imu_timer_;
  virtual void sendGPS(const ros::TimerEvent& event);
  ros::Timer gps_timer_;
  virtual void sendBarometer(const ros::TimerEvent& event);
  ros::Timer barometer_timer_;

  //********************** FUNCTIONS ***********************//
  float rnd();
  float norm_rnd(float mu, float sigma);
};// end class SimSensors
} // end namespace pegasus_sim

#endif // SENSORS_BASE_H
