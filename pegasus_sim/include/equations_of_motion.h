#ifndef EQUATIONS_OF_MOTION_H
#define EQUATIONS_OF_MOTION_H

#include <ros/ros.h>
#include <tf/transform_broadcaster.h>
#include <tf/LinearMath/Quaternion.h>

namespace pegasus
{
class EquationsOfMotion
{
public:
  EquationsOfMotion();

private:

  //********************* NODE HANDLES *********************//
  ros::NodeHandle nh_;           // public node handle for publishing, subscribing
  ros::NodeHandle nh_private_;   // private node handle for pulling parameters

  //************** SUBSCRIBERS AND PUBLISHERS **************//
  // ros::Subscriber motor_command_subscriber_;

  // ros::Publisher truth_publisher_;
  tf::TransformBroadcaster pose_broadcaster_;

  //********************** PARAMETERS **********************//

  //******************** CLASS VARIABLES *******************//
  // Vehicle Description
  double alpha_;              // uncertainty level (0.1 is 10% uncertainty)

  // Message Variables
  double m1_;
  double m2_;
  double m3_;
  double m4_;

  geometry_msgs::TransformStamped odom_trans_;

  // Truth Variables

  //***************** CALLBACKS AND TIMERS *****************//
  // void motorCommandCallback(const pegasus::MotorCommandConstPtr &msg);
  void propogate(const ros::TimerEvent& event);
  ros::Timer propogate_timer_;
  void updateViz(const ros::TimerEvent& event);
  ros::Timer update_viz_timer_;

  //********************** FUNCTIONS ***********************//
  void addUncertainty(double* var);

};// end class eom
} // end namespace pegasus

#endif // EQUATIONS_OF_MOTION_H
