#include <pegasus_sim/forces_and_moments_base.h>

namespace pegasus_sim
{
ForcesAndMoments::ForcesAndMoments() :
  nh_(ros::NodeHandle())
{
  //********************** PARAMETERS **********************//
  if (!(ros::param::get("/pegasus/vehicle_description/num_motors",num_motors_)))
    ROS_WARN("No param named 'num_motors");

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

  //************** SUBSCRIBERS AND PUBLISHERS **************//
  wind_subscriber_ = nh_.subscribe("/pegasus_sim/wind",1,&ForcesAndMoments::windCallback, this);

  if (num_motors_ == 2)
    motor_command_subscriber_=nh_.subscribe("/pegasus/motor_command",1,&ForcesAndMoments::motorCommandCallback2, this);
  else if (num_motors_ == 3)
    motor_command_subscriber_=nh_.subscribe("/pegasus/motor_command",1,&ForcesAndMoments::motorCommandCallback3, this);
  else if (num_motors_ == 4)
    motor_command_subscriber_=nh_.subscribe("/pegasus/motor_command",1,&ForcesAndMoments::motorCommandCallback4, this);
  else if (num_motors_ == 6)
    motor_command_subscriber_=nh_.subscribe("/pegasus/motor_command",1,&ForcesAndMoments::motorCommandCallback6, this);
  else if (num_motors_ == 8)
    motor_command_subscriber_=nh_.subscribe("/pegasus/motor_command",1,&ForcesAndMoments::motorCommandCallback8, this);
  else
    ROS_ERROR("PARAM 'num_motors' IS FAULTY. POSSIBLY INCOMPATIBLE NUMBER OF MOTORS");

  //******************** CLASS VARIABLES *******************//
  w_ns_ = 0.0f;
  w_es_ = 0.0f;
  w_ds_ = 0.0f;
  w_xg_ = 0.0f;
  w_yg_ = 0.0f;
  w_zg_ = 0.0f;

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

  //***************** CALLBACKS AND TIMERS *****************//

  //********************** FUNCTIONS ***********************//

}
ForcesAndMoments::~ForcesAndMoments()
{
    delete motors_;
}
void ForcesAndMoments::getForcesAndMoments(pegasus::state_struct s,
                                           float& fx, float& fy, float& fz,
                                           float& l, float& m, float& n)
{
  ROS_ERROR("CHILD CLASS FUNCTION 'getForcesAndMoments' WAS NOT CALLED.");
}
void ForcesAndMoments::motorCommandCallback2(const pegasus::MotorCommand2ConstPtr &msg)
{
  motors_->msg2struct(msg);
}
void ForcesAndMoments::motorCommandCallback3(const pegasus::MotorCommand3ConstPtr &msg)
{
  motors_->msg2struct(msg);
}
void ForcesAndMoments::motorCommandCallback4(const pegasus::MotorCommand4ConstPtr &msg)
{
  motors_->msg2struct(msg);
}
void ForcesAndMoments::motorCommandCallback6(const pegasus::MotorCommand6ConstPtr &msg)
{
  motors_->msg2struct(msg);
}
void ForcesAndMoments::motorCommandCallback8(const pegasus::MotorCommand8ConstPtr &msg)
{
  motors_->msg2struct(msg);
}
void ForcesAndMoments::initializeMotor(std::string i, pegasus::motor_description* md)
{
  bool ccw;
  if (!(ros::param::get("/pegasus/vehicle_description/motors/m" + i + "/x",md->x)))
    ROS_WARN("No param named 'm%s/x",i.c_str());
  if (!(ros::param::get("/pegasus/vehicle_description/motors/m" + i + "/y",md->y)))
    ROS_WARN("No param named 'm%s/y",i.c_str());
  if (!(ros::param::get("/pegasus/vehicle_description/motors/m" + i + "/z",md->z)))
    ROS_WARN("No param named 'm%s/z",i.c_str());
  if (!(ros::param::get("/pegasus/vehicle_description/motors/m" + i + "/Tx",md->Tx)))
    ROS_WARN("No param named 'm%s/Tx",i.c_str());
  if (!(ros::param::get("/pegasus/vehicle_description/motors/m" + i + "/Ty",md->Ty)))
    ROS_WARN("No param named 'm%s/Ty",i.c_str());
  if (!(ros::param::get("/pegasus/vehicle_description/motors/m" + i + "/Tz",md->Tz)))
    ROS_WARN("No param named 'm%s/Tz",i.c_str());
  if (!(ros::param::get("/pegasus/vehicle_description/motors/m" + i + "/ccw",ccw)))
    ROS_WARN("No param named 'm%s/ccw",i.c_str());
  md->dir = ccw ? -1.0f : 1.0f;
}
void ForcesAndMoments::windCallback(const pegasus_sim::WindConstPtr &msg)
{
  w_ns_ = msg->w_ns;
  w_es_ = msg->w_es;
  w_ds_ = msg->w_ds;
  w_xg_ = msg->w_xg;
  w_yg_ = msg->w_yg;
  w_zg_ = msg->w_zg;
}
} // end namespace pegasus_sim
