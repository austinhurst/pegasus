#include "pegasus/controller_base.h"

#include "pegasus/pd.h"

namespace pegasus
{
Controller::Controller() :
  nh_(ros::NodeHandle())
{
  //********************** PARAMETERS **********************//
  // Simulation Parameters
  float control_rate;
  int num_motors_;
  if (!(ros::param::get("control_rate",control_rate)))
    ROS_WARN("No param named 'control_rate'");
  if (!(ros::param::get("vehicle_description/num_motors",num_motors_)))
    ROS_WARN("No param named 'num_motors");

  //************** SUBSCRIBERS AND PUBLISHERS **************//
  vehicle_state_subscriber_ = nh_.subscribe("/pegasus/vehicle_state",1,&Controller::vehicleStateCallback, this);
  rx_subscriber_ = nh_.subscribe("/pegasus/rx",1,&Controller::rxCallback, this);

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
    motor_command_publisher_ = nh_.advertise<pegasus::MotorCommand2>("/pegasus/motor_command",1);
  else if (num_motors_ == 3)
    motor_command_publisher_ = nh_.advertise<pegasus::MotorCommand3>("/pegasus/motor_command",1);
  else if (num_motors_ == 4)
    motor_command_publisher_ = nh_.advertise<pegasus::MotorCommand4>("/pegasus/motor_command",1);
  else if (num_motors_ == 6)
    motor_command_publisher_ = nh_.advertise<pegasus::MotorCommand6>("/pegasus/motor_command",1);
  else if (num_motors_ == 8)
    motor_command_publisher_ = nh_.advertise<pegasus::MotorCommand8>("/pegasus/motor_command",1);
  else
    ROS_ERROR("PARAM 'num_motors' IS FAULTY. POSSIBLY INCOMPATIBLE NUMBER OF MOTORS");

  //******************** CLASS VARIABLES *******************//
  last_time_ = ros::Time::now();

  //***************** CALLBACKS AND TIMERS *****************//
  control_timer_ = nh_.createTimer(ros::Duration(1.0/control_rate), &Controller::control, this);

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
  rx_ch1_ = msg->ch1;
  rx_ch2_ = msg->ch2;
  rx_ch3_ = msg->ch3;
  rx_ch4_ = msg->ch4;
  rx_ch5_ = msg->ch5;
  rx_ch6_ = msg->ch6;
  rx_ch7_ = msg->ch7;
  rx_ch8_ = msg->ch8;
}
void Controller::publishMotorCommand()
{
  if (num_motors_ == 2)
  {
    MotorCommand2 motor_msg;
    motor_msg.m1 = motors_->m1;
    motor_msg.m2 = motors_->m2;
    motor_command_publisher_.publish(motor_msg);
  }
  else if (num_motors_ == 3)
  {
    MotorCommand3 motor_msg;
    motor_msg.m1 = motors_->m1;
    motor_msg.m2 = motors_->m2;
    motor_msg.m3 = motors_->m3;
    motor_command_publisher_.publish(motor_msg);
  }
  else if (num_motors_ == 4)
  {
    MotorCommand4 motor_msg;
    motor_msg.m1 = motors_->m1;
    motor_msg.m2 = motors_->m2;
    motor_msg.m3 = motors_->m3;
    motor_command_publisher_.publish(motor_msg);
  }
  else if (num_motors_ == 6)
  {
    MotorCommand6 motor_msg;
    motor_msg.m1 = motors_->m1;
    motor_msg.m2 = motors_->m2;
    motor_msg.m3 = motors_->m3;
    motor_msg.m4 = motors_->m4;
    motor_msg.m5 = motors_->m5;
    motor_msg.m6 = motors_->m6;
    motor_command_publisher_.publish(motor_msg);
  }
  else if (num_motors_ == 8)
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
  }
}
} // end namespace pegasus_sim

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
