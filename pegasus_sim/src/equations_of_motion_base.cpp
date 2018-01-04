#include "pegasus_sim/equations_of_motion_base.h"

#include "pegasus_sim/simple_dynamic_model.h"

namespace pegasus_sim
{
EquationsOfMotion::EquationsOfMotion() :
  nh_(ros::NodeHandle())
{
  //************** SUBSCRIBERS AND PUBLISHERS **************//
  motor_command_subscriber_ = nh_.subscribe("/pegasus/motor_command",1,&EquationsOfMotion::motorCommandCallback, this);

  truth_publisher_ = nh_.advertise<pegasus::VehicleState>("/pegasus_sim/truth",1);

  //********************** PARAMETERS **********************//
  // Simulation Parameters
  float propogate_rate, update_viz_rate;
  if (!(ros::param::get("propogate_rate",propogate_rate)))
    ROS_WARN("No param named 'propogate_rate'");
  if (!(ros::param::get("update_viz_rate",update_viz_rate)))
    ROS_WARN("No param named 'update_viz_rate'");
  if (!(ros::param::get("alpha",alpha_)))
    ROS_WARN("No param named 'alpha");

  // Vehicle Parameters (TODO: pull in the vehicle description parameters)

  // Initial Vehicle State (TODO: Pull in from parameter server)
  state_.N = 0.0;
  state_.E = 0.0;
  state_.D = -3.0;

  //******************** CLASS VARIABLES *******************//
  odom_trans_.header.frame_id = "odom";
  odom_trans_.child_frame_id = "base_link";
  srand(time(0));
  last_time_ = ros::Time::now();

  //***************** CALLBACKS AND TIMERS *****************//
  propogate_timer_ = nh_.createTimer(ros::Duration(1.0/propogate_rate), &EquationsOfMotion::propogate, this);
  update_viz_timer_ = nh_.createWallTimer(ros::WallDuration(1.0/update_viz_rate), &EquationsOfMotion::updateViz, this);

  //********************** FUNCTIONS ***********************//
  //addUncertainty(&vehicle_parameter_); // TODO: Do this for each appropriate Vehicle Parameter

} // end constructor
void EquationsOfMotion::propogate(const ros::TimerEvent&)
{
  // Runge-Kutta 4th Order - Compute Truth
  ros::Time new_time = ros::Time::now();
  float h = (new_time - last_time_).toSec();
  k1_ = derivative(state_              );
  k2_ = derivative(state_ + k1_*(h/2.0));
  k3_ = derivative(state_ + k2_*(h/2.0));
  k4_ = derivative(state_ + k3_*h      );
  state_ = state_ + (k1_ + k2_*2.0 + k3_*2.0 + k4_)*(h/6.0);

  // Publish Truth TODO: finish all the states in truth
  truth_msg_.N = state_.N;
  truth_msg_.E = state_.E;
  truth_msg_.D = state_.D;
  truth_publisher_.publish(truth_msg_);

  last_time_ = new_time;
}
pegasus::state_struct EquationsOfMotion::derivative(pegasus::state_struct)
{
  ROS_ERROR("CHILD CLASS FUNCTION 'derivative' WAS NOT CALLED.");
}
void EquationsOfMotion::updateViz(const ros::WallTimerEvent&)
{
  // Pull in State (truth), translate to quaternion, broadcast tf
  odom_trans_.header.stamp = ros::Time::now();
  odom_trans_.transform.translation.x =  state_.N;
  odom_trans_.transform.translation.y =  state_.E;
  odom_trans_.transform.translation.z = -state_.D;
  tf::Quaternion q(0.0, 0.0, 1.0, 0.0);           // TODO: translate state_ to a quaternion
  q.normalize();
  tf::quaternionTFToMsg(q,odom_trans_.transform.rotation);
  pose_broadcaster_.sendTransform(odom_trans_);
}
void EquationsOfMotion::motorCommandCallback(const pegasus::MotorCommand4ConstPtr &msg)
{
  m1_ = msg->m1;
  m2_ = msg->m2;
  m3_ = msg->m3;
  m4_ = msg->m4;
}
void EquationsOfMotion::addUncertainty(float* var)
{
  float random = ((float) rand()/RAND_MAX);
  *var =  *var + (random*2.0*alpha_ - alpha_)*(*var);
}
} // end namespace pegasus_sim

//********************************************************//
//************************ MAIN **************************//
//********************************************************//
int main(int argc, char** argv)
{
  ros::init(argc, argv, "equations_of_motion");
  ros::NodeHandle nh("pegasus_sim");

  pegasus_sim::EquationsOfMotion *eom_obj;
  std::string model_type;
  if (!(ros::param::get("model_type",model_type)))
    ROS_WARN("No param named 'model_type'");
  if (model_type == "simple")
    eom_obj = new pegasus_sim::SimpleDynamicModel;
  else
    ROS_ERROR("NO DYNAMIC MODEL INITIALIZED");

  ros::spin();

  delete eom_obj;
  return 0;
} // end main
