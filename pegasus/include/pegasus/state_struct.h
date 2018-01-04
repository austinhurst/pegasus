#ifndef STATE_STRUCT_H
#define STATE_STRUCT_H

namespace pegasus
{
  struct state_struct
  {
    float N;
    float E;
    float D;
    state_struct()
    {
      N = 0.0;
      E = 0.0;
      D = 0.0;
    }
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
  };
} // end namespace pegasus

#endif // STATE_STRUCT_H
