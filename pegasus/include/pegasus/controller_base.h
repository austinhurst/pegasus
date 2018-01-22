#ifndef CONTROLLER_BASE_H
#define CONTROLLER_BASE_H

#include <iostream>
#include <fstream>


#include <cmath>
#include <map>
#include <string>
#include <ros/ros.h>
#include <pegasus/VehicleState.h>
#include <pegasus/RX8.h>
#include <pegasus/state_struct.h>
#include <pegasus/motor_struct.h>


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


private:
  motor_struct *motors_;

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
  std::map<int, float> angle_map_;
  std::map<int, float> rate_map_;
  std::map<int, float> thrust_map_;

  int A_channel_;
  int E_channel_;
  int T_channel_;
  int R_channel_;
  int aux1_channel_;
  int aux2_channel_;
  int aux3_channel_;
  int aux4_channel_;
  int arming_channel_;
  int num_motors_;

  int min_us_;
  int mid_us_;
  int max_us_;

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

  // RC Override Aux Channel
  int rc_override_channel_;
  int min_auto_us_;
  int max_auto_us_;

  //***************** CALLBACKS AND TIMERS *****************//
  void vehicleStateCallback(const VehicleStateConstPtr &msg);
  void rxCallback(const RX8ConstPtr &msg);
  void serviceAuxChannels(const ros::TimerEvent& event);
  ros::Timer aux_timer_;
protected:
  virtual void control(const ros::TimerEvent& event);
  ros::Timer control_timer_;
  //********************** FUNCTIONS ***********************//
  void pullParameters();
  void publishMotorCommand();
  void mapControlChannels();
  void buildStickMap();
  void mapAuxChannels();
  void setChannels(std::string channel_map);
  void disarm();

};// end class Controller
} // end namespace pegasus

#endif // CONTROLLER_BASE_H
