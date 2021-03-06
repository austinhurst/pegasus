#include <pegasus/controller_base.h>

#include <pegasus/pid.h>

namespace pegasus
{
Controller::Controller() :
  nh_(ros::NodeHandle())
{
  armed_  = false;
  piD180_ = M_PI/180.0f;
  //********************** PARAMETERS **********************//
  float control_rate, aux_rate;
  bool simulating, use_truth;
  getRosParam("control_rate", control_rate);
  getRosParam("vehicle_description/num_motors", num_motors_);
  getRosParam("rx/aux_rate", aux_rate);
  getRosParam("use_truth", use_truth);
  getRosParam("/simulating", simulating);
  pullParameters();

  //************** SUBSCRIBERS AND PUBLISHERS **************//
  if (use_truth && !simulating)
  {
    use_truth = false;
    ROS_WARN("IF NOT SIMULATING, TRUTH IS NOT ACCESSIBLE: use_truth = false");
  }
  if (simulating && use_truth)
    vehicle_state_subscriber_ = nh_.subscribe("/pegasus_sim/truth",1,&Controller::vehicleStateCallback, this);
  else
    vehicle_state_subscriber_ = nh_.subscribe("state_hat",1,&Controller::vehicleStateCallback, this);
  rx_subscriber_ = nh_.subscribe("/rosflight/rc_raw",1,&Controller::rxCallback, this);

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

  if (num_motors_ == 2)
    motor_command_publisher_ = nh_.advertise<pegasus::MotorCommand2>("motor_command",1);
  else if (num_motors_ == 3)
    motor_command_publisher_ = nh_.advertise<pegasus::MotorCommand3>("motor_command",1);
  else if (num_motors_ == 4)
    motor_command_publisher_ = nh_.advertise<pegasus::MotorCommand4>("motor_command",1);
  else if (num_motors_ == 6)
    motor_command_publisher_ = nh_.advertise<pegasus::MotorCommand6>("motor_command",1);
  else if (num_motors_ == 8)
    motor_command_publisher_ = nh_.advertise<pegasus::MotorCommand8>("motor_command",1);
  else
    ROS_ERROR("PARAM 'num_motors' IS FAULTY. POSSIBLY INCOMPATIBLE NUMBER OF MOTORS");

  desired_command_publisher_  = nh_.advertise<pegasus::DesiredControl>("desired_command",1);

  //******************** CLASS VARIABLES *******************//
  last_time_ = ros::Time::now();
  thrust_desired_     = 0.0f;
  yaw_rate_desired_   = 0.0f;
  roll_rate_desired_  = 0.0f;
  pitch_rate_desired_ = 0.0f;
  roll_desired_       = 0.0f;
  pitch_desired_      = 0.0f;
  Vg_desired_         = 0.0f;
  chi_desired_        = 0.0f;
  H_desired_          = 0.0f;

  //***************** CALLBACKS AND TIMERS *****************//
  control_timer_ = nh_.createTimer(ros::Duration(1.0/control_rate), &Controller::control, this);
  aux_timer_     = nh_.createTimer(ros::Duration(1.0/aux_rate), &Controller::serviceAuxChannels, this);

  //********************** FUNCTIONS ***********************//
  buildStickMap();
}
Controller::~Controller()
{
  delete motors_;
}
void Controller::control(const ros::TimerEvent& event)
{
  ROS_ERROR("CHILD CLASS FUNCTION 'control' WAS NOT CALLED.");
}
void Controller::vehicleStateCallback(const VehicleStateConstPtr &msg)
{
  state_.msg2struct(msg);
}
void Controller::rxCallback(const rosflight_msgs::RCRaw &msg)
{
  rx_[0] = msg.values[0];
  rx_[1] = msg.values[1];
  rx_[2] = msg.values[2];
  rx_[3] = msg.values[3];
  rx_[4] = msg.values[4];
  rx_[5] = msg.values[5];
  rx_[6] = msg.values[6];
  rx_[7] = msg.values[7];
}
void Controller::buildStickMap()
{
  float max_angle, angle_expo, rate_expo, thrust_expo, rc, trc,  a_super_rate, r_super_rate, t_super_rate, max_height;
  int A_min_us, A_max_us, E_min_us, E_max_us, T_min_us, T_mid_us, T_max_us;
  int R_min_us, R_mid_us, R_max_us;
  getRosParam("rx/aileron/min_us", A_min_us);
  getRosParam("rx/aileron/mid_us",A_mid_us_);
  getRosParam("rx/aileron/max_us",A_max_us);
  getRosParam("rx/elevator/min_us",E_min_us);
  getRosParam("rx/elevator/mid_us",E_mid_us_);
  getRosParam("rx/elevator/max_us",E_max_us);
  getRosParam("rx/thrust/min_us",T_min_us);
  getRosParam("rx/thrust/mid_us",T_mid_us);
  getRosParam("rx/thrust/max_us",T_max_us);
  getRosParam("rx/rudder/min_us",R_min_us);
  getRosParam("rx/rudder/mid_us",R_mid_us);
  getRosParam("rx/rudder/max_us",R_max_us);
  getRosParam("rx/arming/min_arm_us",min_arm_us_);
  getRosParam("rx/arming/max_arm_us",max_arm_us_);
  if (min_arm_us_ >= max_arm_us_ ||(min_arm_us_ <= T_min_us && max_arm_us_ >= T_max_us))
    ROS_ERROR("ARMING RANGE SET INCORRECTLY");
  getRosParam("rx/arming/arm_throttle_max",arm_throttle_max_);
  if (arm_throttle_max_ > T_min_us + 25.0)
    ROS_ERROR("THROTTLE ARMING RANGE SET INCORRECTLY (NOT WITHIN 25 OF 'min_us'");
  getRosParam("rx/curve/angle_mode/max_angle",max_angle);
  max_angle = max_angle*piD180_;
  getRosParam("rx/curve/angle_mode/expo",angle_expo);
  getRosParam("rx/curve/rate_mode/expo",rate_expo);
  getRosParam("rx/curve/thrust/expo",thrust_expo);
  getRosParam("rx/curve/rate_mode/rc",rc);
  getRosParam("rx/curve/thrust/rc",trc);
  getRosParam("rx/curve/angle_mode/super_rate",a_super_rate);
  getRosParam("rx/curve/rate_mode/super_rate",r_super_rate);
  getRosParam("rx/curve/thrust/super_rate",t_super_rate);
  getRosParam("rx/curve/height/max_height",max_height);
  getRosParam("rx/curve/velocity/max_velocity",max_velocity_);
  float shorter;
  if (A_max_us - A_mid_us_ < A_mid_us_ - A_min_us && A_max_us - A_mid_us_ < 500.0f)
    shorter = A_max_us - A_mid_us_;
  else if (A_mid_us_ - A_min_us < 500.0f)
    shorter = A_mid_us_ - A_min_us;
  else
    shorter = 500.0f;
  for (int i = A_min_us; i <= A_max_us; i++)
  {
    if (i == A_mid_us_)
    {
      A_angle_map_[i]  = 0.0f;
      A_rate_map_[i]   = 0.0f;
    }
    else
    {
    float mid_us_ = A_mid_us_;
    A_angle_map_[i]  = ( pow((abs(i-mid_us_)/shorter),pow(10.0,angle_expo))*max_angle\
                       + pow((abs(i-mid_us_)/shorter),pow(10.0, .8))*max_angle*0.4f*a_super_rate)\
                       *(i-mid_us_)/abs(i-mid_us_);
    A_rate_map_[i]   = ( pow((abs(i-mid_us_)/shorter),pow(10.0,rate_expo))*400.0f*piD180_*rc\
                       + pow((abs(i-mid_us_)/shorter),pow(10.0, .8))*400.0f*piD180_*rc*0.4f*r_super_rate)\
                       *(i-mid_us_)/abs(i-mid_us_);
    }
  }
  if (E_max_us - E_mid_us_ < E_mid_us_ - E_min_us && E_max_us - E_mid_us_ < 500.0f)
    shorter = E_max_us - E_mid_us_;
  else if (E_mid_us_ - E_min_us < 500.0f)
    shorter = E_mid_us_ - E_min_us;
  else
    shorter = 500.0f;
  for (int i = E_min_us; i <= E_max_us; i++)
  {
    if (i == E_mid_us_)
    {
      E_angle_map_[i]  = 0.0f;
      E_rate_map_[i]   = 0.0f;
    }
    else
    {
    float mid_us_ = E_mid_us_;
    E_angle_map_[i]  = ( pow((abs(i-mid_us_)/shorter),pow(10.0,angle_expo))*max_angle\
                       + pow((abs(i-mid_us_)/shorter),pow(10.0, .8))*max_angle*0.4f*a_super_rate)\
                       *(mid_us_ - i)/abs(i-mid_us_);
    E_rate_map_[i]   = ( pow((abs(i-mid_us_)/shorter),pow(10.0,rate_expo))*400.0f*piD180_*rc\
                       + pow((abs(i-mid_us_)/shorter),pow(10.0, .8))*400.0f*piD180_*rc*0.4f*r_super_rate)\
                       *(mid_us_ - i)/abs(i-mid_us_);
    }
  }
  for (int i = T_min_us; i <= T_max_us; i++)
  {
    float mid_us_ = T_mid_us;
    T_map_[i] = ( pow((i - T_min_us)/((float) T_max_us - (float) T_min_us),pow(10.0,thrust_expo))*trc
                + pow((i - T_min_us)/((float) T_max_us - (float) T_min_us),pow(10.0, 0.8))*trc*0.4f*t_super_rate);
  }
  if (R_max_us - R_mid_us < R_mid_us - R_min_us && R_max_us - R_mid_us < 500.0f)
    shorter = R_max_us - R_mid_us;
  else if (R_mid_us - R_min_us < 500.0f)
    shorter = R_mid_us - R_min_us;
  else
    shorter = 500.0f;
  for (int i = R_min_us; i <= R_max_us; i++)
  {
    if (i == R_mid_us)
    {
      R_rate_map_[i]   = 0.0f;
    }
    else
    {
    float mid_us_ = R_mid_us;
    R_rate_map_[i]   = ( pow((abs(i-mid_us_)/shorter),pow(10.0,rate_expo))*400.0f*piD180_*rc\
                       + pow((abs(i-mid_us_)/shorter),pow(10.0, .8))*400.0f*piD180_*rc*0.4f*r_super_rate)\
                       *(i-mid_us_)/abs(i-mid_us_);
    }
  }
  for (int i = T_min_us; i <= T_max_us; i++)
    H_map_[i] = (float (i - T_min_us))/(float (T_max_us - T_min_us))*max_height;
}
void Controller::mapControlChannels()
{
  aileron_stick_      = rx_[A_channel_];
  elevator_stick_     = rx_[E_channel_];
  throttle_stick_     = rx_[T_channel_];
  rudder_stick_       = rx_[R_channel_];

  switch (flight_mode_)
  {
    case ANGLE_MODE:
      roll_desired_       = A_angle_map_[aileron_stick_];
      pitch_desired_      = E_angle_map_[elevator_stick_];
      thrust_desired_     = T_map_[throttle_stick_];
      yaw_rate_desired_   = R_rate_map_[rudder_stick_];
    break;
    case VELOC_MODE:
      H_desired_          = H_map_[throttle_stick_];
      Vg_desired_         = sqrtf(powf(aileron_stick_  - A_mid_us_, 2.0f) \
                                + powf(elevator_stick_ - E_mid_us_, 2.0f))/500.*max_velocity_;
      chi_desired_        = atan2f(aileron_stick_ - A_mid_us_, elevator_stick_ - E_mid_us_);
      yaw_rate_desired_   = R_rate_map_[rudder_stick_];
    break;
    case RATES_MODE:
      roll_rate_desired_  = A_rate_map_[aileron_stick_];
      pitch_rate_desired_ = E_rate_map_[elevator_stick_];
      thrust_desired_     = T_map_[throttle_stick_];
      yaw_rate_desired_   = R_rate_map_[rudder_stick_];
    break;
    case AUTO__MODE:
      // TODO
    break;
  }
}
void Controller::mapAuxChannels()
{
  aux1_stick_ = rx_[aux1_channel_];
  aux2_stick_ = rx_[aux2_channel_];
  aux3_stick_ = rx_[aux3_channel_];
  aux4_stick_ = rx_[aux4_channel_];
}
void Controller::serviceAuxChannels(const ros::TimerEvent& event)
{
  mapAuxChannels();
  float aux_sticks[4] = {aux1_stick_, aux2_stick_, aux3_stick_, aux4_stick_};

  // Arming
  if (aux_sticks[arm_aux_channel_ - 1] >= min_arm_us_ && aux_sticks[arm_aux_channel_ - 1] <= max_arm_us_)
  {
    if (!armed_)
    {
      mapControlChannels();
      if (throttle_stick_ <= arm_throttle_max_)
        armed_ = true;
    }
  }
  else
    disarm();

  int last_flight_mode = flight_mode_;
  // Flight Mode
  if      (aux_sticks[mode_aux_channel_ - 1] > min_auto_us_    && aux_sticks[mode_aux_channel_ - 1] < max_auto_us_)
    flight_mode_ = AUTO__MODE;
  else if (aux_sticks[mode_aux_channel_ - 1] > min_angle_mode_ && aux_sticks[mode_aux_channel_ - 1] < max_angle_mode_)
    flight_mode_ = ANGLE_MODE;
  else if (aux_sticks[mode_aux_channel_ - 1] > min_rates_mode_ && aux_sticks[mode_aux_channel_ - 1] < max_rates_mode_)
    flight_mode_ = RATES_MODE;
  else if (aux_sticks[mode_aux_channel_ - 1] > min_veloc_mode_ && aux_sticks[mode_aux_channel_ - 1] < max_veloc_mode_)
    flight_mode_ = VELOC_MODE;
  if (flight_mode_ != last_flight_mode)
  {
    switch (last_flight_mode)
    {
      case ANGLE_MODE:
        roll_desired_       = 0.0f;
        pitch_desired_      = 0.0f;
        thrust_desired_     = 0.0f;
        yaw_rate_desired_   = 0.0f;
      break;
      case VELOC_MODE:
        H_desired_          = 0.0f;
        Vg_desired_         = 0.0f;
        chi_desired_        = 0.0f;
        yaw_rate_desired_   = 0.0f;
      break;
      case RATES_MODE:
        roll_rate_desired_  = 0.0f;
        pitch_rate_desired_ = 0.0f;
        thrust_desired_     = 0.0f;
        yaw_rate_desired_   = 0.0f;
      break;
      case AUTO__MODE:
        // TODO
      break;
    }
  }
}
void Controller::publishDesiredCommand()
{
  DesiredControl des_msg;
  des_msg.thrust_desired     = thrust_desired_;
  des_msg.yaw_rate_desired   = yaw_rate_desired_;
  des_msg.roll_rate_desired  = roll_rate_desired_;
  des_msg.pitch_rate_desired = pitch_rate_desired_;
  des_msg.roll_desired       = roll_desired_;
  des_msg.pitch_desired      = pitch_desired_;
  des_msg.Vg_desired         = Vg_desired_;
  des_msg.chi_desired        = chi_desired_;
  des_msg.H_desired          = H_desired_;
  desired_command_publisher_.publish(des_msg);
}
void Controller::publishMotorCommand()
{
  if (armed_)
  {
    switch (num_motors_)  // Note each case needs scope
    {
    case 2:
    {
      MotorCommand2 motor_msg;
      motor_msg.m1 = motors_->m1;
      motor_msg.m2 = motors_->m2;
      motor_command_publisher_.publish(motor_msg);
      break;
    }
    case 3:
    {
      MotorCommand3 motor_msg;
      motor_msg.m1 = motors_->m1;
      motor_msg.m2 = motors_->m2;
      motor_msg.m3 = motors_->m3;
      motor_command_publisher_.publish(motor_msg);
      break;
    }
    case 4:
    {
      MotorCommand4 motor_msg;
      motor_msg.m1 = motors_->m1;
      motor_msg.m2 = motors_->m2;
      motor_msg.m3 = motors_->m3;
      motor_msg.m4 = motors_->m4;
      motor_command_publisher_.publish(motor_msg);
      break;
    }
    case 6:
    {
      MotorCommand6 motor_msg;
      motor_msg.m1 = motors_->m1;
      motor_msg.m2 = motors_->m2;
      motor_msg.m3 = motors_->m3;
      motor_msg.m4 = motors_->m4;
      motor_msg.m5 = motors_->m5;
      motor_msg.m6 = motors_->m6;
      motor_command_publisher_.publish(motor_msg);
      break;
    }
    case 8:
    {
      MotorCommand8 motor_msg;
      motor_msg.m1 = motors_->m1;
      motor_msg.m2 = motors_->m2;
      motor_msg.m3 = motors_->m3;
      motor_msg.m4 = motors_->m4;
      motor_msg.m5 = motors_->m5;
      motor_msg.m6 = motors_->m6;
      motor_msg.m7 = motors_->m7;
      motor_msg.m8 = motors_->m8;
      motor_command_publisher_.publish(motor_msg);
      break;
    }
    }
  }
  else // if NOT armed_
    disarm();
}
void Controller::disarm()
{
  armed_ = false;
  switch (num_motors_)  // Note each case needs scope
  {
  case 2:
  {
    MotorCommand2 motor_msg;
    motor_msg.m1 = 0.0;
    motor_msg.m2 = 0.0;
    motor_command_publisher_.publish(motor_msg);
    break;
  }
  case 3:
  {
    MotorCommand3 motor_msg;
    motor_msg.m1 = 0.0;
    motor_msg.m2 = 0.0;
    motor_msg.m3 = 0.0;
    motor_command_publisher_.publish(motor_msg);
    break;
  }
  case 4:
  {
    MotorCommand4 motor_msg;
    motor_msg.m1 = 0.0;
    motor_msg.m2 = 0.0;
    motor_msg.m3 = 0.0;
    motor_command_publisher_.publish(motor_msg);
    break;
  }
  case 6:
  {
    MotorCommand6 motor_msg;
    motor_msg.m1 = 0.0;
    motor_msg.m2 = 0.0;
    motor_msg.m3 = 0.0;
    motor_msg.m4 = 0.0;
    motor_msg.m5 = 0.0;
    motor_msg.m6 = 0.0;
    motor_command_publisher_.publish(motor_msg);
    break;
  }
  case 8:
  {
    MotorCommand8 motor_msg;
    motor_msg.m1 = 0.0;
    motor_msg.m2 = 0.0;
    motor_msg.m3 = 0.0;
    motor_msg.m4 = 0.0;
    motor_msg.m5 = 0.0;
    motor_msg.m6 = 0.0;
    motor_msg.m7 = 0.0;
    motor_msg.m8 = 0.0;
    motor_command_publisher_.publish(motor_msg);
    break;
  }
  }
}
void Controller::setChannels(std::string channel_map)
{
  for (unsigned int i = 0; i < 8; i++)
  {
    switch (channel_map[i])
    {
    case 'A':
      A_channel_ = i;
      break;
    case 'E':
      E_channel_ = i;
      break;
    case 'T':
      T_channel_ = i;
      break;
    case 'R':
      R_channel_ = i;
      break;
    case '1':
      aux1_channel_ = i;
      break;
    case '2':
      aux2_channel_ = i;
      break;
    case '3':
      aux3_channel_ = i;
      break;
    case '4':
      aux4_channel_ = i;
      break;
    }
  }
}
void Controller::pullParameters()
{
  std::string channel_map;
  getRosParam("rx/channel_map",channel_map);
  setChannels(channel_map);

  getRosParam("rx/arming/arm_aux_channel",arm_aux_channel_);
  if (arm_aux_channel_ != 1 && arm_aux_channel_ != 2 && arm_aux_channel_ != 3 && arm_aux_channel_ != 4)
    ROS_ERROR("ARMING AUX CHANNEL IS NOT SET APPROPRIATELY");
  getRosParam("rx/modes/mode_aux_channel",mode_aux_channel_);
  getRosParam("rx/modes/min_angle_mode",min_angle_mode_);
  getRosParam("rx/modes/max_angle_mode",max_angle_mode_);
  getRosParam("rx/modes/min_rates_mode",min_rates_mode_);
  getRosParam("rx/modes/max_rates_mode",max_rates_mode_);
  getRosParam("rx/modes/min_veloc_mode",min_veloc_mode_);
  getRosParam("rx/modes/max_veloc_mode",max_veloc_mode_);
  getRosParam("rx/modes/min_auto_us",min_auto_us_);
  getRosParam("rx/modes/max_auto_us",max_auto_us_);
}
void Controller::getRosParam(std::string parameter_name, int &param)
{
  if (!(ros::param::get(parameter_name ,param)))
    ROS_WARN("%s",("No param named '" + parameter_name + "'").c_str());
}
void Controller::getRosParam(std::string parameter_name, float &param)
{
  if (!(ros::param::get(parameter_name ,param)))
    ROS_WARN("%s",("No param named '" + parameter_name + "'").c_str());
}
void Controller::getRosParam(std::string parameter_name, double &param)
{
  if (!(ros::param::get(parameter_name ,param)))
    ROS_WARN("%s",("No param named '" + parameter_name + "'").c_str());
}
void Controller::getRosParam(std::string parameter_name, bool &param)
{
  if (!(ros::param::get(parameter_name ,param)))
    ROS_WARN("%s",("No param named '" + parameter_name + "'").c_str());
}
void Controller::getRosParam(std::string parameter_name, std::string &param)
{
  if (!(ros::param::get(parameter_name ,param)))
    ROS_WARN("%s",("No param named '" + parameter_name + "'").c_str());
}
} // end namespace pegasus

//********************************************************//
//************************ MAIN **************************//
//********************************************************//
int main(int argc, char** argv)
{
  ros::init(argc, argv, "controller");
  ros::NodeHandle nh("pegasus");

  pegasus::Controller *controller_obj;
  std::string control_type;
  if (!(ros::param::get("control_type",control_type)))
    ROS_WARN("No param named 'control_type'");
  if (control_type == "PID")
    controller_obj = new pegasus::PID;
  else
    ROS_ERROR("NO CONTROLLER INITIALIZED");

  ros::spin();

  delete controller_obj;
  return 0;
} // end main
