#include <pegasus/estimator_base.h>

#include <pegasus/luenberger_observer.h>

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
    sonar_subscriber_ = nh_.subscribe("sonar", 1, &Estimator::sonarCallback, this);
    gps_subscriber_ = nh_.subscribe("gps", 1, &Estimator::gpsCallback, this);
    barometer_subscriber_ = nh_.subscribe("barometer", 1, &Estimator::barometerCallback, this);
    imu_subscriber_ = nh_.subscribe("imu", 1, &Estimator::imuCallback, this);
  }

  state_hat_publisher_ = nh_.advertise<VehicleState>("state_hat",1);

  // Initial Vehicle State (TODO: Pull in from parameter server)

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
void Estimator::sonarCallback(const SonarConstPtr &msg)
{
  sonar_distance_ = msg->distance;
}
void Estimator::gpsCallback(const GPSConstPtr &msg)
{
  gps_fix_           = msg->fix;
  gps_NumSat_        = msg->NumSat;
  gps_latitude_      = msg->latitude;
  gps_longitude_     = msg->longitude;
  gps_altitude_      = msg->altitude;
  gps_speed_         = msg->speed;
  gps_ground_course_ = msg->ground_course;
  gps_covariance_    = msg->covariance;
}
void Estimator::barometerCallback(const BarometerConstPtr &msg)
{
  barometer_pressure_ = msg->pressure;
}
void Estimator::imuCallback(const sensor_msgs::ImuConstPtr &msg)
{
  //tf::transformMsgToTF(&(msg->orientation), imu_orientation_); // TODO: This line doesn't work... I am not sure what data type to transform it into.
  imu_angular_velocity_[0]                 = msg->angular_velocity.x;
  imu_angular_velocity_[1]                 = msg->angular_velocity.y;
  imu_angular_velocity_[2]                 = msg->angular_velocity.z;
  imu_linear_acceleration_[0]              = msg->linear_acceleration.x;
  imu_linear_acceleration_[1]              = msg->linear_acceleration.y;
  imu_linear_acceleration_[2]              = msg->linear_acceleration.z;
  for (unsigned int i = 0; i < 3; i++)
  {
    imu_orientation_covariance_[i]         = msg->orientation_covariance[i];
    imu_angular_velocity_covariance_[i]    = msg->angular_velocity_covariance[i];
    imu_linear_acceleration_covariance_[i] = msg->linear_acceleration_covariance[i];
  }
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

  delete est_obj;
  return 0;
} // end main
