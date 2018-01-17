#include "pegasus/controller_base.h"

#include "pegasus/pd.h"

namespace pegasus
{
Controller::Controller() :
  nh_(ros::NodeHandle())
{
  armed_ = false;

  //********************** PARAMETERS **********************//
  float control_rate, arming_rate;
  int num_motors_;
  if (!(ros::param::get("control_rate",control_rate)))
    ROS_WARN("No param named 'control_rate'");
  if (!(ros::param::get("vehicle_description/num_motors",num_motors_)))
    ROS_WARN("No param named 'num_motors");
  if (!(ros::param::get("rx/arming/arming_rate",arming_rate)))
    ROS_WARN("No param named 'arming_rate'");
  pullParameters();

  //************** SUBSCRIBERS AND PUBLISHERS **************//
  vehicle_state_subscriber_ = nh_.subscribe("state_hat",1,&Controller::vehicleStateCallback, this);
  rx_subscriber_ = nh_.subscribe("rx",1,&Controller::rxCallback, this);

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

  //******************** CLASS VARIABLES *******************//
  last_time_ = ros::Time::now();

  //***************** CALLBACKS AND TIMERS *****************//
  control_timer_ = nh_.createTimer(ros::Duration(1.0/control_rate), &Controller::control, this);
  arming_timer_ = nh_.createTimer(ros::Duration(1.0/arming_rate), &Controller::checkArmingChannel, this);
  //********************** FUNCTIONS ***********************//

}
Controller::~Controller()
{

}
void Controller::control(const ros::TimerEvent& event)
{
  ROS_ERROR("CHILD CLASS FUNCTION 'control' WAS NOT CALLED.");
}
void Controller::vehicleStateCallback(const VehicleStateConstPtr &msg)
{
  state_.msg2struct(msg);
}
void Controller::rxCallback(const RX8ConstPtr &msg)
{
  rx_[0] = msg->ch1;
  rx_[1] = msg->ch2;
  rx_[2] = msg->ch3;
  rx_[3] = msg->ch4;
  rx_[4] = msg->ch5;
  rx_[5] = msg->ch6;
  rx_[6] = msg->ch7;
  rx_[7] = msg->ch8;
}
void Controller::mapControlChannels()
{
  aileron_stick_  = rx_[A_channel_];
  elevator_stick_ = rx_[E_channel_];
  throttle_stick_ = rx_[T_channel_];
  rudder_stick_   = rx_[R_channel_];
}
void Controller::mapAuxChannels()
{
  aux1_stick_ = rx_[aux1_channel_];
  aux2_stick_ = rx_[aux2_channel_];
  aux3_stick_ = rx_[aux3_channel_];
  aux4_stick_ = rx_[aux4_channel_];
}
void Controller::checkArmingChannel(const ros::TimerEvent& event)
{
  mapAuxChannels();
  float aux_sticks[4] = {aux1_stick_, aux2_stick_, aux3_stick_, aux4_stick_};
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
  if (!(ros::param::get("rx/channel_map",channel_map)))
    ROS_WARN("No param named 'channel_map'");
  setChannels(channel_map);
  if (!(ros::param::get("rx/min_us",min_us_)))
    ROS_WARN("No param named 'min_us'");
  if (!(ros::param::get("rx/mid_us",mid_us_)))
    ROS_WARN("No param named 'mid_us'");
  if (!(ros::param::get("rx/max_us",max_us_)))
    ROS_WARN("No param named 'max_us'");
  if (!(ros::param::get("rx/arming/arm_aux_channel",arm_aux_channel_)))
    ROS_WARN("No param named 'arm_aux_channel'");
  if (arm_aux_channel_ != 1 && arm_aux_channel_ != 2 && arm_aux_channel_ != 3 && arm_aux_channel_ != 4)
    ROS_ERROR("ARMING AUX CHANNEL IS NOT SET APPROPRIATELY");
  if (!(ros::param::get("rx/arming/min_arm_us",min_arm_us_)))
    ROS_WARN("No param named 'min_arm_us'");
  if (!(ros::param::get("rx/arming/max_arm_us",max_arm_us_)))
    ROS_WARN("No param named 'max_arm_us'");
  if (min_arm_us_ >= max_arm_us_ ||(min_arm_us_ <= min_us_ && max_arm_us_ >= max_us_))
    ROS_ERROR("ARMING RANGE SET INCORRECTLY");
  if (!(ros::param::get("rx/arming/arm_throttle_max",arm_throttle_max_)))
    ROS_WARN("No param named 'arm_throttle_max'");
  if (arm_throttle_max_ > min_us_ + 25.0)
    ROS_ERROR("THROTTLE ARMING RANGE SET INCORRECTLY (NOT WITHIN 25 OF 'min_us'");

  if (!(ros::param::get("rx/modes/mode_aux_channel",mode_aux_channel_)))
    ROS_WARN("No param named 'mode_aux_channel'");
  if (!(ros::param::get("rx/modes/min_angle_mode",min_angle_mode_)))
    ROS_WARN("No param named 'min_angle_mode'");
  if (!(ros::param::get("rx/modes/max_angle_mode",max_angle_mode_)))
    ROS_WARN("No param named 'max_angle_mode'");
  if (!(ros::param::get("rx/modes/min_rates_mode",min_rates_mode_)))
    ROS_WARN("No param named 'min_rates_mode'");
  if (!(ros::param::get("rx/modes/max_rates_mode",max_rates_mode_)))
    ROS_WARN("No param named 'max_rates_mode'");
  if (!(ros::param::get("rx/modes/min_veloc_mode",min_veloc_mode_)))
    ROS_WARN("No param named 'min_veloc_mode'");
  if (!(ros::param::get("rx/modes/max_veloc_mode",max_veloc_mode_)))
    ROS_WARN("No param named 'max_veloc_mode'");
  if (!(ros::param::get("rx/modes/rc_override_channel",rc_override_channel_)))
    ROS_WARN("No param named 'rc_override_channel'");
  if (!(ros::param::get("rx/modes/min_rc_us",min_rc_us_)))
    ROS_WARN("No param named 'min_rc_us'");
  if (!(ros::param::get("rx/modes/max_rc_us",max_rc_us_)))
    ROS_WARN("No param named 'max_rc_us'");
  if (!(ros::param::get("rx/modes/min_auto_us",min_auto_us_)))
    ROS_WARN("No param named 'min_auto_us'");
  if (!(ros::param::get("rx/modes/max_auto_us",max_auto_us_)))
    ROS_WARN("No param named 'max_auto_us'");
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
  if (control_type == "PD")
    controller_obj = new pegasus::PD;
  else
    ROS_ERROR("NO CONTROLLER INITIALIZED");

  ros::spin();

  delete controller_obj;
  return 0;
} // end main
