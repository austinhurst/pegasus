#ifndef EQUATIONS_OF_MOTION_BASE_H
#define EQUATIONS_OF_MOTION_BASE_H

#include <cmath>
#include <string>
#include <stdlib.h>
#include <ros/ros.h>
#include <pegasus_sim/forces_and_moments.h>
#include <tf/transform_broadcaster.h>
#include <tf/LinearMath/Quaternion.h>
#include <pegasus/VehicleState.h>
#include <pegasus/state_struct.h>

namespace pegasus_sim
{
class EquationsOfMotion
{
public:
  EquationsOfMotion();
  ~EquationsOfMotion();
private:
  //********************* NODE HANDLES *********************//
  ros::NodeHandle nh_;         // public node handle for publishing, subscribing

  //************** SUBSCRIBERS AND PUBLISHERS **************//
  ros::Publisher truth_publisher_;
  tf::TransformBroadcaster pose_broadcaster_;

  //******************** CLASS VARIABLES *******************//
  float alpha_;                // uncertainty level (0.1 is 10% uncertainty)
protected:
  float mass_;                // Mass of the vehicle
  float Jx_;                  // moment  of inertia about i^b in Kg*m^2
  float Jy_;                  // moment  of inertia about j^b in Kg*m^2
  float Jz_;                  // moment  of inertia about k^b in Kg*m^2
  float Jxy_;                 // product of inertia about i^b in Kg*m^2
  float Jxz_;                 // product of inertia about j^b in Kg*m^2
  float Jyz_;                 // product of inertia about k^b in Kg*m^2
  // Truth Variables
  pegasus::state_struct state_;

  ForcesAndMoments* f_and_m_obj_;
  
private:
  // RK4 Variables
  pegasus::state_struct k1_;
  pegasus::state_struct k2_;
  pegasus::state_struct k3_;
  pegasus::state_struct k4_;
  ros::Time last_time_;

private:
  pegasus::VehicleState truth_msg_;
  geometry_msgs::TransformStamped odom_trans_;

  //***************** CALLBACKS AND TIMERS *****************//
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
