#include "equations_of_motion.h"

namespace pegasus
{
EquationsOfMotion::EquationsOfMotion() :
  nh_(ros::NodeHandle()),
  nh_private_(ros::NodeHandle("~"))
{
  //************** SUBSCRIBERS AND PUBLISHERS **************//
  // motor_command_subscriber_ = nh_.subscribe("motor_command",1,&EquationsOfMotion::motorCommandCallback, this);

  //********************** PARAMETERS **********************//

  //******************** CLASS VARIABLES *******************//
  odom_trans_.header.frame_id = "odom";
  odom_trans_.child_frame_id = "base_link";

  //***************** CALLBACKS AND TIMERS *****************//
  propogate_timer_ = nh_.createTimer(ros::Duration(0.1), &EquationsOfMotion::propogate, this);
  update_viz_timer_ = nh_.createTimer(ros::Duration(0.05), &EquationsOfMotion::updateViz, this);

  //********************** FUNCTIONS ***********************//
  //add_uncertainty(&var);

} // end constructor
void EquationsOfMotion::propogate(const ros::TimerEvent&)
{
  // Runge-Kutta 4th Order - Compute truth and publish

}
void EquationsOfMotion::updateViz(const ros::TimerEvent&)
{
  // Pull in State (truth), translate to quaternion, broadcast tf
  odom_trans_.header.stamp = ros::Time::now();
  odom_trans_.transform.translation.x = 0.0;
  odom_trans_.transform.translation.y = 0.0;
  odom_trans_.transform.translation.z = 3.0;
  tf::Quaternion q(0.0, 0.0, 1.0, 0.0);
  q.normalize();
  tf::quaternionTFToMsg(q,odom_trans_.transform.rotation);
  pose_broadcaster_.sendTransform(odom_trans_);
}
// void EquationsOfMotion::motorCommandCallback(const pegasus::MotorCommandConstPtr &msg)
// {
//   m1_ = msg->m1;
//   m2_ = msg->m2;
//   m3_ = msg->m3;
//   m4_ = msg->m4;
// }
void EquationsOfMotion::addUncertainty(double* var)
{
  // double rand =
  double rand = 0.0;
  *var =  *var + (rand*2.0*alpha_ - alpha_)*(*var);
}
} // end namespace pegasus

//********************************************************//
//************************ MAIN **************************//
//********************************************************//
int main(int argc, char** argv)
{
  ros::init(argc, argv, "equations_of_motion");
  ros::NodeHandle nh;

  pegasus::EquationsOfMotion eom_obj;

  ros::spin();

  return 0;
} // end main
