#include <pegasus_sim/equations_of_motion_base.h>

#include <pegasus_sim/dynamic_model.h>
#include <pegasus_sim/sensor_models.h>
#include <pegasus_sim/forces_and_moments.h>

namespace pegasus_sim
{
EquationsOfMotion::EquationsOfMotion() :
  nh_(ros::NodeHandle())
{
  //********************** PARAMETERS **********************//
  // Simulation Parameters
  float propogate_rate, update_viz_rate;
  if (!(ros::param::get("sim/propogate_rate",propogate_rate)))
    ROS_WARN("No param named 'propogate_rate'");
  if (!(ros::param::get("/pegasus/ground_station/update_viz_rate",update_viz_rate)))
    ROS_WARN("No param named 'update_viz_rate'");
  if (!(ros::param::get("sim/alpha",alpha_)))
    ROS_WARN("No param named 'alpha");
  if (!(ros::param::get("/pegasus/vehicle_description/mass",mass_)))
    ROS_WARN("No param named 'mass");
  if (!(ros::param::get("/pegasus/vehicle_description/Jx",Jx_)))
    ROS_WARN("No param named 'Jx");
  if (!(ros::param::get("/pegasus/vehicle_description/Jy",Jy_)))
    ROS_WARN("No param named 'Jy");
  if (!(ros::param::get("/pegasus/vehicle_description/Jz",Jz_)))
    ROS_WARN("No param named 'Jz");
  if (!(ros::param::get("/pegasus/vehicle_description/Jxy",Jxy_)))
    ROS_WARN("No param named 'Jxy");
  if (!(ros::param::get("/pegasus/vehicle_description/Jxz",Jxz_)))
    ROS_WARN("No param named 'num_motors");
  if (!(ros::param::get("/pegasus/vehicle_description/Jyz",Jyz_)))
    ROS_WARN("No param named 'Jyz");

  // Initial Vehicle State (TODO: Pull in from parameter server)
  state_.pn = 0.0f;
  state_.pe = 0.0f;
  state_.pd = -3.0f;
  state_.phi = 0.0*3.141592653/180.0;
  state_.theta = 0.0*3.141592653/180.0;
  state_.psi = 0.0*3.141592653/180.0;

  //************** SUBSCRIBERS AND PUBLISHERS **************//
  truth_publisher_ = nh_.advertise<pegasus::VehicleState>("/pegasus_sim/truth",1);

  //******************** CLASS VARIABLES *******************//

  odom_trans_.header.frame_id = "odom";
  odom_trans_.child_frame_id = "base_link";
  srand(time(0));
  last_time_ = ros::Time::now();

  //***************** CALLBACKS AND TIMERS *****************//
  propogate_timer_ = nh_.createTimer(ros::Duration(1.0/propogate_rate), &EquationsOfMotion::propogate, this);
  update_viz_timer_ = nh_.createWallTimer(ros::WallDuration(1.0/update_viz_rate), &EquationsOfMotion::updateViz, this);

  //********************** FUNCTIONS ***********************//
  // addUncertainty(&Jx_);
  // addUncertainty(&Jy_);
  // addUncertainty(&Jz_);
  // addUncertainty(&Jxy_);
  // addUncertainty(&Jxz_);
  // addUncertainty(&Jyz_);
}
EquationsOfMotion::~EquationsOfMotion()
{

}
void EquationsOfMotion::propogate(const ros::TimerEvent&)
{
  // Runge-Kutta 4th Order - Compute Truth
  ros::Time new_time = ros::Time::now();
  float h = (new_time - last_time_).toSec();
  f_and_m_obj_->eachTimeStep();
  k1_ = derivative(state_               );
  k2_ = derivative(state_ + k1_*(h/2.0f));
  k3_ = derivative(state_ + k2_*(h/2.0f));
  k4_ = derivative(state_ + k3_*h       );
  state_ = state_ + (k1_ + k2_*2.0f + k3_*2.0f + k4_)*(h/6.0f);
  truth_publisher_.publish(state_.msg());
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
  odom_trans_.transform.translation.x =  state_.pe;
  odom_trans_.transform.translation.y =  state_.pn;
  odom_trans_.transform.translation.z = -state_.pd;
  // odom_trans_.transform.translation.x =  0.0;
  // odom_trans_.transform.translation.y =  0.0;
  // odom_trans_.transform.translation.z =  0.0;
  // Euler Angles in NED to Quaternion in NED to Quaternion in XYZ
  float qx,qy,qz,qw;
  qx =   cosf(state_.psi/2.0f)*sinf(state_.theta/2.0f)*cosf(state_.phi/2.0f)\
       + sinf(state_.psi/2.0f)*cosf(state_.theta/2.0f)*sinf(state_.phi/2.0f);
  qy =   cosf(state_.psi/2.0f)*cosf(state_.theta/2.0f)*sinf(state_.phi/2.0f)\
       - sinf(state_.psi/2.0f)*sinf(state_.theta/2.0f)*cosf(state_.phi/2.0f);
  qz = - sinf(state_.psi/2.0f)*cosf(state_.theta/2.0f)*cosf(state_.phi/2.0f)\
       + cosf(state_.psi/2.0f)*sinf(state_.theta/2.0f)*sinf(state_.phi/2.0f);
  qw =   cosf(state_.psi/2.0f)*cosf(state_.theta/2.0f)*cosf(state_.phi/2.0f)\
       + sinf(state_.psi/2.0f)*sinf(state_.theta/2.0f)*sinf(state_.phi/2.0f);
  tf::Quaternion q(qx,qy,qz,qw);
  tf::quaternionTFToMsg(q,odom_trans_.transform.rotation);
  pose_broadcaster_.sendTransform(odom_trans_);
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

  pegasus_sim::ForcesAndMoments f_and_m_obj;
  pegasus_sim::EquationsOfMotion *eom_obj;
  std::string model_type;
  if (!(ros::param::get("sim/model_type",model_type)))
    ROS_WARN("No param named 'model_type'");
  if (model_type == "simple")
    eom_obj = new pegasus_sim::DynamicModel(&f_and_m_obj);
  else
    ROS_ERROR("NO DYNAMIC MODEL INITIALIZED");

  pegasus_sim::SensorModels sensors_obj(&f_and_m_obj);

  ros::spin();

  delete eom_obj;
  return 0;
} // end main
