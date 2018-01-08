#include "pegasus/estimator_base.h"

#include "pegasus/luenberger_observer.h"

namespace pegasus
{
Estimator::Estimator() :
  nh_(ros::NodeHandle())
{
  //********************** PARAMETERS **********************//
  // Estimator Parameters
  bool use_truth, simulating;
  float estimate_rate(1.0);
  int num_motors;
  if (!(ros::param::get("est/use_truth",use_truth)))
    ROS_WARN("No param named 'use_truth'");
  if (!(ros::param::get("/simulating",simulating)))
    ROS_WARN("No param named 'simulating'");
  if (!(ros::param::get("vehicle_description/num_motors",num_motors)))
    ROS_WARN("No param named 'num_motors'");
  if (use_truth && !simulating)
  {
    use_truth = false;
    ROS_WARN("IF NOT SIMULATING, TRUTH IS NOT ACCESSIBLE: use_truth = false");
  }
  if(!simulating || !use_truth)
  {
    std::string est_type;
    if (!(ros::param::get("est/est_type",est_type)))
      ROS_WARN("No param named 'est_type'");
    if (!(ros::param::get("est/" + est_type + "/estimate_rate",estimate_rate)))
      ROS_WARN("No param named 'estimate_rate'");
  }

  //************** SUBSCRIBERS AND PUBLISHERS **************//
  if (num_motors == 2)
    motor_command_subscriber_ = nh_.subscribe("motor_command", 1, &Estimator::motorCommandCallback2, this);
  else if (num_motors == 3)
    motor_command_subscriber_ = nh_.subscribe("motor_command", 1, &Estimator::motorCommandCallback3, this);
  else if (num_motors == 4)
    motor_command_subscriber_ = nh_.subscribe("motor_command", 1, &Estimator::motorCommandCallback4, this);
  else if (num_motors == 6)
    motor_command_subscriber_ = nh_.subscribe("motor_command", 1, &Estimator::motorCommandCallback6, this);
  else if (num_motors == 8)
    motor_command_subscriber_ = nh_.subscribe("motor_command", 1, &Estimator::motorCommandCallback8, this);
  else
    ROS_ERROR("PARAM 'num_motors' IS FAULTY. POSSIBLY INCOMPATIBLE NUMBER OF MOTORS");

  if (simulating && use_truth)
  {
    truth_subscriber_ = nh_.subscribe("/pegasus_sim/truth",1,&Estimator::truthCallback, this);
  }
  else
  {

  }

  state_hat_publisher_ = nh_.advertise<VehicleState>("state_hat",1);

  // Initial Vehicle State (TODO: Pull in from parameter server)
  state_hat_.N = 0.0;
  state_hat_.E = 0.0;
  state_hat_.D = 0.0;

  //******************** CLASS VARIABLES *******************//
  if (num_motors == 2)
    motors_ = new pegasus::motor_struct_2;
  else if (num_motors == 3)
    motors_ = new pegasus::motor_struct_3;
  else if (num_motors == 4)
    motors_ = new pegasus::motor_struct_4;
  else if (num_motors == 6)
    motors_ = new pegasus::motor_struct_6;
  else if (num_motors == 8)
    motors_ = new pegasus::motor_struct_8;
  else
    ROS_ERROR("THE STRUCT 'motors_' WAS NOT INITIALIZED. POSSIBLY INCOMPATIBLE NUMBER OF MOTORS");

  last_time_ = ros::Time::now();

  //***************** CALLBACKS AND TIMERS *****************//
  if (!simulating || !use_truth)
    estimate_timer_ = nh_.createTimer(ros::Duration(1.0/estimate_rate), &Estimator::estimate, this);
  //********************** FUNCTIONS ***********************//


}
Estimator::~Estimator()
{
  delete motors_;
}
void Estimator::estimate(const ros::TimerEvent& event)
{
  ROS_ERROR("CHILD CLASS FUNCTION 'estimate' WAS NOT CALLED.");
}
void Estimator::motorCommandCallback2(const MotorCommand2ConstPtr &msg)
{
  motors_->msg2struct(msg);
}
void Estimator::motorCommandCallback3(const MotorCommand3ConstPtr &msg)
{
  motors_->msg2struct(msg);
}
void Estimator::motorCommandCallback4(const MotorCommand4ConstPtr &msg)
{
  motors_->msg2struct(msg);
}
void Estimator::motorCommandCallback6(const MotorCommand6ConstPtr &msg)
{
  motors_->msg2struct(msg);
}
void Estimator::motorCommandCallback8(const MotorCommand8ConstPtr &msg)
{
  motors_->msg2struct(msg);
}
void Estimator::truthCallback(const VehicleStateConstPtr &msg)          // Only used in simulation when use_truth is true
{
  ros::Time new_time = ros::Time::now();
  state_hat_.msg2struct(msg);
  state_hat_publisher_.publish(*msg);
  last_time_ = new_time;
}
} // end namespace pegasus

//********************************************************//
//************************ MAIN **************************//
//********************************************************//
int main(int argc, char** argv)
{
  ros::init(argc, argv, "estimator");
  ros::NodeHandle nh("pegasus");

  pegasus::Estimator *est_obj;
  std::string est_type;
  if (!(ros::param::get("est/est_type",est_type)))
    ROS_WARN("No param named 'est_type'");
  if (est_type == "luenberger")
    est_obj = new pegasus::LuenbergerObserver;
  else
    ROS_ERROR("NO ESTIMATOR INITIALIZED");

  ros::spin();

  // delete est_obj;
  return 0;
} // end main
