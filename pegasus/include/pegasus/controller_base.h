#ifndef CONTROLLER_BASE_H
#define CONTROLLER_BASE_H

#include <string>
#include <ros/ros.h>
#include <pegasus/VehicleState.h>
#include <pegasus/RX8.h>
#include <pegasus/state_struct.h>
#include <pegasus/motor_struct.h>

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
  pegasus::state_struct state_;
  ros::Time last_time_;
  motor_struct *motors_;

  float rx_[8];
  float aileron_stick_;
  float elevator_stick_;
  float throttle_stick_;
  float rudder_stick_;
  float aux1_stick_;
  float aux2_stick_;
  float aux3_stick_;
  float aux4_stick_;

private:
  // rx Channel Variables
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

  float min_us_;
  float mid_us_;
  float max_us_;

  // Arming Variables
  int arm_aux_channel_;
  float min_arm_us_;
  float max_arm_us_;
  float arm_throttle_max_;

  // Mode Aux Channel
  int mode_aux_channel_;
  float min_angle_mode_;
  float max_angle_mode_;
  float min_rates_mode_;
  float max_rates_mode_;
  float min_veloc_mode_;
  float max_veloc_mode_;

  // RC Override Aux Channel
  int rc_override_channel_;
  float min_rc_us_;
  float max_rc_us_;
  float min_auto_us_;
  float max_auto_us_;

  //***************** CALLBACKS AND TIMERS *****************//
  void vehicleStateCallback(const VehicleStateConstPtr &msg);
  void rxCallback(const RX8ConstPtr &msg);
  void checkArmingChannel(const ros::TimerEvent& event);
  ros::Timer arming_timer_;
protected:
  virtual void control(const ros::TimerEvent& event);
  ros::Timer control_timer_;
  //********************** FUNCTIONS ***********************//
  void pullParameters();
  void publishMotorCommand();
  void mapControlChannels();
  void mapAuxChannels();
  void setChannels(std::string channel_map);
  void disarm();

};// end class Controller
} // end namespace pegasus

#endif // CONTROLLER_BASE_H
