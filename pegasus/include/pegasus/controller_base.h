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
  // Incoming State Variable
protected:
  pegasus::state_struct state_;
  ros::Time last_time_;
  motor_struct *motors_;

  float rx_ch1_;
  float rx_ch2_;
  float rx_ch3_;
  float rx_ch4_;
  float rx_ch5_;
  float rx_ch6_;
  float rx_ch7_;
  float rx_ch8_;

private:
  int num_motors_;

  //***************** CALLBACKS AND TIMERS *****************//
    void vehicleStateCallback(const VehicleStateConstPtr &msg);
    void rxCallback(const RX8ConstPtr &msg);
protected:
  virtual void control(const ros::TimerEvent& event);
  ros::Timer control_timer_;

  //********************** FUNCTIONS ***********************//
  void publishMotorCommand();

};// end class Controller
} // end namespace pegasus

#endif // CONTROLLER_BASE_H
