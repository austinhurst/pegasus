#ifndef STATE_STRUCT_H
#define STATE_STRUCT_H

#include <pegasus/VehicleState.h>


namespace pegasus
{
  struct state_struct
  {
    //*********************** VARIABLE ***********************//
    // EDIT EVERY FUNCTION IF YOU ADD A VARIABLE TO THE STRUCT OR MESSAGE
    float pn;        // Inertial North position
    float pe;        // Inertial East position
    float pd;        // Inertial Down position
    float u;         // Body frame velocity along i^(body)
    float v;         // Body frame velocity along j^(body)
    float w;         // Body frame velocity along k^(body)
    float phi;       // Roll, angle rotated around i^(vehicle-2)
    float theta;     // Pitch, angle rotated around j^(vehicle-1)
    float psi;       // Yaw, angle rotated around k^(vehicle)
    float p;         // Roll rate around i^(body)
    float q;         // Pitch rate around j^(body)
    float r;         // Yaw rate around k^(body)

  private:
    VehicleState _msg_;
  public:
    state_struct()
    {
      pn    = 0.0f;
      pe    = 0.0f;
      pd    = 0.0f;
      u     = 0.0f;
      v     = 0.0f;
      w     = 0.0f;
      phi   = 0.0f;
      theta = 0.0f;
      psi   = 0.0f;
      p     = 0.0f;
      q     = 0.0f;
      r     = 0.0f;
    }
    //****************** OPERATOR FUNCTIONS ******************//
    state_struct operator+(const state_struct s)
    {
      state_struct n;
      n.pn    = s.pn    + pn;
      n.pe    = s.pe    + pe;
      n.pd    = s.pd    + pd;
      n.u     = s.u     + u;
      n.v     = s.v     + v;
      n.w     = s.w     + w;
      n.phi   = s.phi   + phi;
      n.theta = s.theta + theta;
      n.psi   = s.psi   + psi;
      n.p     = s.p     + p;
      n.q     = s.q     + q;
      n.r     = s.r     + r;
      return n;
    }
    state_struct operator*(const float num)
    {
      state_struct n;
      n.pn    = pn    * num;
      n.pe    = pe    * num;
      n.pd    = pd    * num;
      n.u     = u     * num;
      n.v     = v     * num;
      n.w     = w     * num;
      n.phi   = phi   * num;
      n.theta = theta * num;
      n.psi   = psi   * num;
      n.p     = p     * num;
      n.q     = q     * num;
      n.r     = r     * num;
      return n;
    }
    //********************** FUNCTIONS ***********************//
    VehicleState msg()
    {
      _msg_.pn    = pn;
      _msg_.pe    = pe;
      _msg_.pd    = pd;
      _msg_.u     = u;
      _msg_.v     = v;
      _msg_.w     = w;
      _msg_.phi   = phi;
      _msg_.theta = theta;
      _msg_.psi   = psi;
      _msg_.p     = p;
      _msg_.q     = q;
      _msg_.r     = r;
      return _msg_;
    }
    void msg2struct(const VehicleState msg_in)
    {
      pn    = msg_in.pn;
      pe    = msg_in.pe;
      pd    = msg_in.pd;
      u     = msg_in.u;
      v     = msg_in.v;
      w     = msg_in.w;
      phi   = msg_in.phi;
      theta = msg_in.theta;
      psi   = msg_in.psi;
      p     = msg_in.p;
      q     = msg_in.q;
      r     = msg_in.r;
    }
    void msg2struct(const VehicleStateConstPtr &msg_in)
    {
      pn    = msg_in->pn;
      pe    = msg_in->pe;
      pd    = msg_in->pd;
      u     = msg_in->u;
      v     = msg_in->v;
      w     = msg_in->w;
      phi   = msg_in->phi;
      theta = msg_in->theta;
      psi   = msg_in->psi;
      p     = msg_in->p;
      q     = msg_in->q;
      r     = msg_in->r;
    }
  };
} // end namespace pegasus

#endif // STATE_STRUCT_H
