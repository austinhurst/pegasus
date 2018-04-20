#include <pegasus/estimator_base.h>

#include <pegasus/kalman_filter.h>

namespace pegasus
{
Estimator::Estimator() :
  nh_(ros::NodeHandle())
{
  //********************** PARAMETERS **********************//
  // Estimator Parameters
  float estimate_rate;
  if (!(ros::param::get("vehicle_description/num_motors",num_motors_)))
    ROS_WARN("No param named 'num_motors'");
  std::string est_type;
  if (!(ros::param::get("est/est_type",est_type)))
    ROS_WARN("No param named 'est_type'");
  if (!(ros::param::get("est/" + est_type + "/estimate_rate",estimate_rate)))
    ROS_WARN("No param named 'estimate_rate'");
  if (!(ros::param::get("vehicle_description/g",g_)))
    ROS_WARN("No param named 'g");
  if (!(ros::param::get("vehicle_description/mass",mass_)))
    ROS_WARN("No param named 'mass");
  if (!(ros::param::get("/pegasus/vehicle_description/motors/K_delta_t",K_delta_t_)))
    ROS_WARN("No param named 'K_delta_t");
  if (!(ros::param::get("vehicle_description/motors/KQ",KQ_)))
    ROS_WARN("No param named 'KQ");
  if (!(ros::param::get("vehicle_description/motors/Vb",Vb_)))
    ROS_WARN("No param named 'Vb");
  if (!(ros::param::get("vehicle_description/motors/Kv",Kv_)))
    ROS_WARN("No param named 'Kv");
  if (!(ros::param::get("vehicle_description/motors/Rm",Rm_)))
    ROS_WARN("No param named 'Rm");
  if (!(ros::param::get("vehicle_description/motors/i0",i0_)))
    ROS_WARN("No param named 'i0");
  if (!(ros::param::get("vehicle_description/motors/Dp",Dp_)))
    ROS_WARN("No param named 'Dp");
  if (!(ros::param::get("vehicle_description/rho",rho_)))
    ROS_WARN("No param named 'rho");

  Dp_          = Dp_*0.0254f;
  Ap_          = M_PI*Dp_*Dp_/4.0f;
  piD30_       = M_PI/30.0f;

  //************** SUBSCRIBERS AND PUBLISHERS **************//
  if (num_motors_ == 2)
    motor_command_subscriber_ = nh_.subscribe("motor_command", 1, &Estimator::motorCommandCallback2, this);
  else if (num_motors_ == 3)
    motor_command_subscriber_ = nh_.subscribe("motor_command", 1, &Estimator::motorCommandCallback3, this);
  else if (num_motors_ == 4)
    motor_command_subscriber_ = nh_.subscribe("motor_command", 1, &Estimator::motorCommandCallback4, this);
  else if (num_motors_ == 6)
    motor_command_subscriber_ = nh_.subscribe("motor_command", 1, &Estimator::motorCommandCallback6, this);
  else if (num_motors_ == 8)
    motor_command_subscriber_ = nh_.subscribe("motor_command", 1, &Estimator::motorCommandCallback8, this);
  else
    ROS_ERROR("PARAM 'num_motors' IS FAULTY. POSSIBLY INCOMPATIBLE NUMBER OF MOTORS");


  bool use_sonar, use_gps, use_imu, use_barometer;
  if (!(ros::param::get("vehicle_description/sensors/use_sonar",use_sonar)))
    ROS_WARN("No param named 'use_sonar'");
  if (!(ros::param::get("vehicle_description/sensors/use_gps",use_gps)))
    ROS_WARN("No param named 'use_gps'");
  if (!(ros::param::get("vehicle_description/sensors/use_imu",use_imu)))
    ROS_WARN("No param named 'use_imu'");
  if (!(ros::param::get("vehicle_description/sensors/use_barometer",use_barometer)))
    ROS_WARN("No param named 'use_barometer'");

  if (use_sonar)
    sonar_subscriber_     = nh_.subscribe("sonar", 1, &Estimator::sonarCallback, this);
  if (use_gps)
    gps_subscriber_       = nh_.subscribe("gps", 1, &Estimator::gpsCallback, this);
  if (use_barometer)
    barometer_subscriber_ = nh_.subscribe("barometer", 1, &Estimator::barometerCallback, this);
  if (use_imu)
    imu_subscriber_       = nh_.subscribe("imu", 1, &Estimator::imuCallback, this);

  state_hat_publisher_  = nh_.advertise<VehicleState>("state_hat",1);

  // Initial Vehicle State
  float latitudeN0, longitudeE0, heightM0;
  if (!(ros::param::get("initial/pn",state_hat_.pn)))
    ROS_WARN("No param named 'pn'");
  if (!(ros::param::get("initial/pe",state_hat_.pe)))
    ROS_WARN("No param named 'pe'");
  if (!(ros::param::get("initial/pd",state_hat_.pd)))
    ROS_WARN("No param named 'pd'");
  if (!(ros::param::get("initial/u",state_hat_.u)))
    ROS_WARN("No param named 'u'");
  if (!(ros::param::get("initial/v",state_hat_.v)))
    ROS_WARN("No param named 'v'");
  if (!(ros::param::get("initial/w",state_hat_.w)))
    ROS_WARN("No param named 'w'");
  if (!(ros::param::get("initial/phi",state_hat_.phi)))
    ROS_WARN("No param named 'phi'");
  if (!(ros::param::get("initial/theta",state_hat_.theta)))
    ROS_WARN("No param named 'theta'");
  if (!(ros::param::get("initial/psi",state_hat_.psi)))
    ROS_WARN("No param named 'psi'");
  if (!(ros::param::get("initial/p",state_hat_.p)))
    ROS_WARN("No param named 'p'");
  if (!(ros::param::get("initial/q",state_hat_.q)))
    ROS_WARN("No param named 'q'");
  if (!(ros::param::get("initial/r",state_hat_.r)))
    ROS_WARN("No param named 'r'");
  if (!(ros::param::get("/latitudeN0",latitudeN0)))
    ROS_WARN("No param named 'latitudeN0'");
  if (!(ros::param::get("/longitudeE0",longitudeE0)))
    ROS_WARN("No param named 'longitudeE0'");
  if (!(ros::param::get("/heightM0",heightM0)))
    ROS_WARN("No param named 'heightM0'");
  gps_converter_.set_reference(latitudeN0, longitudeE0, heightM0);

  //******************** CLASS VARIABLES *******************//
  if (num_motors_ == 2)
    motors_ = new pegasus::motor_struct_2;
  else if (num_motors_ == 3)
    motors_ = new pegasus::motor_struct_3;
  else if (num_motors_ == 4)
    motors_ = new pegasus::motor_struct_4;
  else if (num_motors_ == 6)
    motors_ = new pegasus::motor_struct_6;
  else if (num_motors_ == 8)
    motors_ = new pegasus::motor_struct_8;
  else
    ROS_ERROR("THE STRUCT 'motors_' WAS NOT INITIALIZED. POSSIBLY INCOMPATIBLE NUMBER OF MOTORS");
  for (int i = 0; i < num_motors_; i++)
  {
    switch (i)
    {
      case 0: initializeMotor("1", &m1d_); break;
      case 1: initializeMotor("2", &m2d_); break;
      case 2: initializeMotor("3", &m3d_); break;
      case 3: initializeMotor("4", &m4d_); break;
      case 4: initializeMotor("5", &m5d_); break;
      case 5: initializeMotor("6", &m6d_); break;
      case 6: initializeMotor("7", &m7d_); break;
      case 7: initializeMotor("8", &m8d_); break;
    }
  }
  last_time_ = ros::Time::now();

  //***************** CALLBACKS AND TIMERS *****************//
  estimate_timer_ = nh_.createTimer(ros::Duration(1.0/estimate_rate), &Estimator::predict, this);
  //********************** FUNCTIONS ***********************//


}
Estimator::~Estimator()
{
  delete motors_;
}
void Estimator::predict(const ros::TimerEvent& event)
{
  ROS_ERROR("CHILD CLASS FUNCTION 'predict' WAS NOT CALLED.");
}
void Estimator::correctBarometer()
{
  ROS_ERROR("CHILD CLASS FUNCTION 'correctBarometer' WAS NOT CALLED.");
}
void Estimator::correctGPS()
{
  ROS_ERROR("CHILD CLASS FUNCTION 'correctGPS' WAS NOT CALLED.");
}
void Estimator::correctIMU()
{
  ROS_ERROR("CHILD CLASS FUNCTION 'correctIMU' WAS NOT CALLED.");
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
  float lat_N, lon_E, h_M;
  gps_fix_           = msg->fix;
  lat_N              = msg->latitude;
  lon_E              = msg->longitude;
  h_M                = msg->altitude;
  gps_speed_         = msg->speed;
  gps_ground_course_ = msg->ground_course;
  gps_converter_.gps2ned(lat_N, lon_E, h_M, gps_N_, gps_E_, gps_D_);
  correctGPS();
}
void Estimator::barometerCallback(const BarometerConstPtr &msg)
{
  barometer_pressure_ = msg->pressure;
  correctBarometer();
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
  // for (unsigned int i = 0; i < 3; i++)
  // {
  //   imu_orientation_covariance_[i]         = msg->orientation_covariance[i];
  //   imu_angular_velocity_covariance_[i]    = msg->angular_velocity_covariance[i];
  //   imu_linear_acceleration_covariance_[i] = msg->linear_acceleration_covariance[i];
  // }
  correctIMU();
}
void Estimator::initializeMotor(std::string i, motor_description* md)
{
  bool ccw;
  if (!(ros::param::get("vehicle_description/motors/m" + i + "/x",md->x)))
    ROS_WARN("No param named 'm%s/x",i.c_str());
  if (!(ros::param::get("vehicle_description/motors/m" + i + "/y",md->y)))
    ROS_WARN("No param named 'm%s/y",i.c_str());
  if (!(ros::param::get("vehicle_description/motors/m" + i + "/z",md->z)))
    ROS_WARN("No param named 'm%s/z",i.c_str());
  if (!(ros::param::get("vehicle_description/motors/m" + i + "/Tx",md->Tx)))
    ROS_WARN("No param named 'm%s/Tx",i.c_str());
  if (!(ros::param::get("vehicle_description/motors/m" + i + "/Ty",md->Ty)))
    ROS_WARN("No param named 'm%s/Ty",i.c_str());
  if (!(ros::param::get("vehicle_description/motors/m" + i + "/Tz",md->Tz)))
    ROS_WARN("No param named 'm%s/Tz",i.c_str());
  if (!(ros::param::get("vehicle_description/motors/m" + i + "/ccw",ccw)))
    ROS_WARN("No param named 'm%s/ccw",i.c_str());
  md->dir = ccw ? -1.0f : 1.0f;
}
void Estimator::getF()
{
  // Force due to Propultion
  fx_p_ = 0.0f;
  fy_p_ = 0.0f;
  fz_p_ = 0.0f;
  l_p_  = 0.0f;
  m_p_  = 0.0f;
  n_p_  = 0.0f;

  for (int ii = 0; ii < num_motors_; ii++)
  {
    pegasus::motor_description *md;
    float delta_m;
    switch (ii)
    {
      case 0: md = &m1d_; delta_m = motors_->m1; break;
      case 1: md = &m2d_; delta_m = motors_->m2; break;
      case 2: md = &m3d_; delta_m = motors_->m3; break;
      case 3: md = &m4d_; delta_m = motors_->m4; break;
      case 4: md = &m5d_; delta_m = motors_->m5; break;
      case 5: md = &m6d_; delta_m = motors_->m6; break;
      case 6: md = &m7d_; delta_m = motors_->m7; break;
      case 7: md = &m8d_; delta_m = motors_->m8; break;
    }

    delta_m  =  delta_m < 0.0f ? 0.0f : delta_m;

    // Model of a DC Motor
    float omega, i, Qm, P_shaft, T, Q;
    omega   = K_delta_t_*delta_m;
    i       = (Vb_ - omega/Kv_)/Rm_;
    Qm      = (i-i0_)/(Kv_*piD30_);
    P_shaft = Qm*omega*piD30_;

    // Momentum Theory
    T = powf(P_shaft*P_shaft*2.0f*rho_*Ap_, 1.0f/3.0f);
    Q = T/6.30f*md->dir;

    // Put the Forces and Torques into the correct orientation
    fx_p_ += T*md->Tx;
    fy_p_ += T*md->Ty;
    fz_p_ += T*md->Tz;
    l_p_  += T*md->Tz*md->y - T*md->Ty*md->z + Q*md->Tx;
    m_p_  += T*md->Tx*md->z - T*md->Tz*md->x + Q*md->Ty;
    n_p_  += T*md->Ty*md->x - T*md->Tx*md->y + Q*md->Tz;
  }
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
  if (est_type == "kalman")
    est_obj = new pegasus::KalmanFilter;
  else
    ROS_ERROR("NO ESTIMATOR INITIALIZED");

  ros::spin();

  delete est_obj;
  return 0;
} // end main
