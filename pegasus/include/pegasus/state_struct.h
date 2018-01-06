#ifndef STATE_STRUCT_H
#define STATE_STRUCT_H

#include <pegasus/VehicleState.h>


namespace pegasus
{
  struct state_struct
  {
    //*********************** VARIABLE ***********************//
    // EDIT EVERY FUNCTION IF YOU ADD A VARIABLE TO THE STRUCT
    float N;
    float E;
    float D;

  private:
    VehicleState _msg_;
  public:
    state_struct()
    {
      N = 0.0;
      E = 0.0;
      D = 0.0;
    }
    //****************** OPERATOR FUNCTIONS ******************//
    state_struct operator+(const state_struct s)
    {
      state_struct n;
      n.N = s.N + N;
      n.E = s.E + E;
      n.D = s.D + D;
      return n;
    }
    state_struct operator*(const float num)
    {
      state_struct n;
      n.N = num*N;
      n.E = num*E;
      n.D = num*D;
      return n;
    }
    //********************** FUNCTIONS ***********************//
    VehicleState msg()
    {
      _msg_.N = N;
      _msg_.E = E;
      _msg_.D = D;
      return _msg_;
    }
    void msg2struct(const VehicleState msg_in)
    {
      N = msg_in.N;
      E = msg_in.N;
      D = msg_in.N;
    }
    void msg2struct(const VehicleStateConstPtr &msg_in)
    {
      N = msg_in->N;
      E = msg_in->N;
      D = msg_in->N;
    }
  };
} // end namespace pegasus

#endif // STATE_STRUCT_H
