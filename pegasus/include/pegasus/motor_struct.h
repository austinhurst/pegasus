#ifndef MOTOR_STRUCT_H
#define MOTOR_STRUCT_H

#include <pegasus/MotorCommand2.h>
#include <pegasus/MotorCommand3.h>
#include <pegasus/MotorCommand4.h>
#include <pegasus/MotorCommand6.h>
#include <pegasus/MotorCommand8.h>

namespace pegasus
{
  struct motor_struct
  {
    //*********************** VARIABLES **********************//
    // EDIT EVERY FUNCTION IF YOU ADD A VARIABLE TO THE STRUCT
    float m1;
    float m2;
    float m3;
    float m4;
    float m5;
    float m6;
    float m7;
    float m8;
    motor_struct()
    {
      m1 = 0.0;
      m2 = 0.0;
      m3 = 0.0;
      m4 = 0.0;
      m5 = 0.0;
      m6 = 0.0;
      m7 = 0.0;
      m8 = 0.0;
    }
    //****************** OPERATOR FUNCTIONS ******************//

    //********************** FUNCTIONS ***********************//
    virtual void msg2struct(const MotorCommand2 msg_in)
    {

    }
    virtual void msg2struct(const MotorCommand3 msg_in)
    {

    }
    virtual void msg2struct(const MotorCommand4 msg_in)
    {

    }
    virtual void msg2struct(const MotorCommand6 msg_in)
    {

    }
    virtual void msg2struct(const MotorCommand8 msg_in)
    {

    }
    virtual void msg2struct(const MotorCommand2ConstPtr &msg_in)
    {

    }
    virtual void msg2struct(const MotorCommand3ConstPtr &msg_in)
    {

    }
    virtual void msg2struct(const MotorCommand4ConstPtr &msg_in)
    {

    }
    virtual void msg2struct(const MotorCommand6ConstPtr &msg_in)
    {

    }
    virtual void msg2struct(const MotorCommand8ConstPtr &msg_in)
    {

    }
  };

  //********************************************************//
  //********************** CHILDREN ************************//
  //********************************************************//

  struct motor_struct_2 : public motor_struct
  {
    //*********************** VARIABLES **********************//
    // EDIT EVERY FUNCTION IF YOU ADD A VARIABLE TO THE STRUCT
    motor_struct_2()
    {

    }
    //****************** OPERATOR FUNCTIONS ******************//

    //********************** FUNCTIONS ***********************//
    void msg2struct(const MotorCommand2 msg_in)
    {
      m1 = msg_in.m1;
      m2 = msg_in.m2;
    }
    void msg2struct(const MotorCommand2ConstPtr &msg_in)
    {
      m1 = msg_in->m1;
      m2 = msg_in->m2;
    }
  };
  struct motor_struct_3 : public motor_struct
  {
    //*********************** VARIABLES **********************//
    // EDIT EVERY FUNCTION IF YOU ADD A VARIABLE TO THE STRUCT
    motor_struct_3()
    {

    }
    //****************** OPERATOR FUNCTIONS ******************//

    //********************** FUNCTIONS ***********************//
    void msg2struct(const MotorCommand3 msg_in)
    {
      m1 = msg_in.m1;
      m2 = msg_in.m2;
      m3 = msg_in.m3;
    }
    void msg2struct(const MotorCommand3ConstPtr &msg_in)
    {
      m1 = msg_in->m1;
      m2 = msg_in->m2;
      m3 = msg_in->m3;
    }
  };
  struct motor_struct_4 : public motor_struct
  {
    //*********************** VARIABLES **********************//
    // EDIT EVERY FUNCTION IF YOU ADD A VARIABLE TO THE STRUCT
    motor_struct_4()
    {

    }
    //****************** OPERATOR FUNCTIONS ******************//

    //********************** FUNCTIONS ***********************//
    void msg2struct(const MotorCommand4 msg_in)
    {
      m1 = msg_in.m1;
      m2 = msg_in.m2;
      m3 = msg_in.m3;
      m4 = msg_in.m4;
    }
    void msg2struct(const MotorCommand4ConstPtr &msg_in)
    {
      m1 = msg_in->m1;
      m2 = msg_in->m2;
      m3 = msg_in->m3;
      m4 = msg_in->m4;
    }
  };
  struct motor_struct_6 : public motor_struct
  {
    //*********************** VARIABLES **********************//
    // EDIT EVERY FUNCTION IF YOU ADD A VARIABLE TO THE STRUCT
    motor_struct_6()
    {

    }
    //****************** OPERATOR FUNCTIONS ******************//

    //********************** FUNCTIONS ***********************//
    void msg2struct(const MotorCommand6 msg_in)
    {
      m1 = msg_in.m1;
      m2 = msg_in.m2;
      m3 = msg_in.m3;
      m4 = msg_in.m4;
      m5 = msg_in.m5;
      m6 = msg_in.m6;
    }
    void msg2struct(const MotorCommand6ConstPtr &msg_in)
    {
      m1 = msg_in->m1;
      m2 = msg_in->m2;
      m3 = msg_in->m3;
      m4 = msg_in->m4;
      m5 = msg_in->m5;
      m6 = msg_in->m6;
    }
  };
  struct motor_struct_8 : public motor_struct
  {
    //*********************** VARIABLES **********************//
    // EDIT EVERY FUNCTION IF YOU ADD A VARIABLE TO THE STRUCT
    motor_struct_8()
    {

    }
    //****************** OPERATOR FUNCTIONS ******************//

    //********************** FUNCTIONS ***********************//
    void msg2struct(const MotorCommand8 msg_in)
    {
      m1 = msg_in.m1;
      m2 = msg_in.m2;
      m3 = msg_in.m3;
      m4 = msg_in.m4;
      m5 = msg_in.m5;
      m6 = msg_in.m6;
      m7 = msg_in.m7;
      m8 = msg_in.m8;
    }
    void msg2struct(const MotorCommand8ConstPtr &msg_in)
    {
      m1 = msg_in->m1;
      m2 = msg_in->m2;
      m3 = msg_in->m3;
      m4 = msg_in->m4;
      m5 = msg_in->m5;
      m6 = msg_in->m6;
      m7 = msg_in->m7;
      m8 = msg_in->m8;
    }
  };
  struct motor_description
  {
    //*********************** VARIABLES **********************//
    float x;      // x position of the motor
    float y;      // y position of the motor
    float z;      // z position of the motor
    float dir;    // direction of the motor, 1.0 or -1.0
    float Tx;     // component of the thrust and torque in the x direction
    float Ty;     // component of the thrust and torque in the y direction
    float Tz;     // component of the thrust and torque in the z direction

    motor_description()
    {

    }
    //****************** OPERATOR FUNCTIONS ******************//

    //********************** FUNCTIONS ***********************//

  };
} // end namespace pegasus

#endif // MOTOR_STRUCT_H
