#ifndef CONTROLLER_BASE_H
#define CONTROLLER_BASE_H

#include <iostream>
#include <fstream>


#include <cmath>
#include <map>
#include <string>
#include <ros/ros.h>
#include <pegasus/VehicleState.h>
#include <pegasus/state_struct.h>
#include <pegasus/motor_struct.h>
#include <pegasus/DesiredControl.h>

#include <rosflight_msgs/RCRaw.h>


#define ANGLE_MODE 0
#define RATES_MODE 1
#define VELOC_MODE 2
#define AUTO__MODE 3


namespace pegasus
{
class Controller
{
public:
  Controller();
  ~Controller();
private:
  //********************* NODE HANDLES *********************//
  ros::NodeHandle nh_;         // public node handle for publishing, subscribing

  //************** SUBSCRIBERS AND PUBLISHERS **************//
  ros::Subscriber vehicle_state_subscriber_;
  ros::Subscriber rx_subscriber_;

  ros::Publisher motor_command_publisher_;
  ros::Publisher desired_command_publisher_;

  //******************** CLASS VARIABLES *******************//
protected:
  bool armed_;
  int flight_mode_;
  pegasus::state_struct state_;
  ros::Time last_time_;

  float roll_desired_;
  float pitch_desired_;
  float thrust_desired_;

  float yaw_rate_desired_;
  float roll_rate_desired_;
  float pitch_rate_desired_;
  float Vg_desired_;
  float chi_desired_;
  float H_desired_;

  motor_struct *motors_;
  int num_motors_;
  float piD180_;

private:
  // rx Channel Variables
  int rx_[8];
  int aileron_stick_;
  int elevator_stick_;
  int throttle_stick_;
  int rudder_stick_;
  int aux1_stick_;
  int aux2_stick_;
  int aux3_stick_;
  int aux4_stick_;
  std::map<int, float> A_angle_map_;
  std::map<int, float> E_angle_map_;
  std::map<int, float> A_rate_map_;
  std::map<int, float> E_rate_map_;
  std::map<int, float> R_rate_map_;
  std::map<int, float> T_map_;
  std::map<int, float> H_map_;

  int A_channel_;
  int E_channel_;
  int T_channel_;
  int R_channel_;
  int aux1_channel_;
  int aux2_channel_;
  int aux3_channel_;
  int aux4_channel_;
  int arming_channel_;

  // Arming Aux Channel
  int arm_aux_channel_;
  int min_arm_us_;
  int max_arm_us_;
  int arm_throttle_max_;

  // Mode Aux Channel
  int mode_aux_channel_;
  int min_angle_mode_;
  int max_angle_mode_;
  int min_rates_mode_;
  int max_rates_mode_;
  int min_veloc_mode_;
  int max_veloc_mode_;
  int min_auto_us_;
  int max_auto_us_;

  // VELOC_MODE variables
  int A_mid_us_;
  int E_mid_us_;

  //***************** CALLBACKS AND TIMERS *****************//
  void vehicleStateCallback(const VehicleStateConstPtr &msg);
  void rxCallback(const rosflight_msgs::RCRaw &msg);
  void serviceAuxChannels(const ros::TimerEvent& event);
  ros::Timer aux_timer_;
protected:
  virtual void control(const ros::TimerEvent& event);
  ros::Timer control_timer_;
  //********************** FUNCTIONS ***********************//
  void pullParameters();
  void publishMotorCommand();
  void publishDesiredCommand();
  void mapControlChannels();
  void buildStickMap();
  void mapAuxChannels();
  void setChannels(std::string channel_map);
  void disarm();
  void getRosParam(std::string parameter_name,      int    &param);
  void getRosParam(std::string parameter_name,      float  &param);
  void getRosParam(std::string parameter_name,      double &param);
  void getRosParam(std::string parameter_name, std::string &param);

};// end class Controller
} // end namespace pegasus

#endif // CONTROLLER_BASE_H
