#ifndef EQUATIONS_OF_MOTION_H
#define EQUATIONS_OF_MOTION_H

#include <string>
#include <stdlib.h>
#include <ros/ros.h>
#include <tf/transform_broadcaster.h>
#include <tf/LinearMath/Quaternion.h>
#include <pegasus/MotorCommand4.h>
#include <pegasus/VehicleState.h>
#include <pegasus/state_struct.h>

namespace pegasus_sim
{
class EquationsOfMotion
{
public:
  EquationsOfMotion();

private:
  //********************* NODE HANDLES *********************//
  ros::NodeHandle nh_;         // public node handle for publishing, subscribing

  //************** SUBSCRIBERS AND PUBLISHERS **************//
  ros::Subscriber motor_command_subscriber_;

  ros::Publisher truth_publisher_;
  tf::TransformBroadcaster pose_broadcaster_;

  //******************** CLASS VARIABLES *******************//
  // TODO: Vehicle Description (things like motor mass, to be used by 'derivative')
  float alpha_;                // uncertainty level (0.1 is 10% uncertainty)


  // Truth Variables
  pegasus::state_struct state_;

  // RK4 Variables
  pegasus::state_struct k1_;
  pegasus::state_struct k2_;
  pegasus::state_struct k3_;
  pegasus::state_struct k4_;
  ros::Time last_time_;

  // Message Variables
  float m1_;
  float m2_;
  float m3_;
  float m4_;

  pegasus::VehicleState truth_msg_;
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

#endif // EQUATIONS_OF_MOTION_H
