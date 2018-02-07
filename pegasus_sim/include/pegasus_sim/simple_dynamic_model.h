#ifndef SIMPLE_DYNAMIC_MODEL_H
#define SIMPLE_DYNAMIC_MODEL_H

#include <pegasus_sim/equations_of_motion_base.h>

namespace pegasus_sim
{
class SimpleDynamicModel : public EquationsOfMotion
{
public:
  SimpleDynamicModel();

private:
  virtual pegasus::state_struct derivative(pegasus::state_struct s);
  virtual void eachTimeStep();
  float invJ11_;
  float invJ12_;
  float invJ13_;
  float invJ21_;
  float invJ22_;
  float invJ23_;
  float invJ31_;
  float invJ32_;
  float invJ33_;
  float w_ns_;
  float w_es_;
  float w_ds_;
  float w_xg_;
  float w_yg_;
  float w_zg_;
  float half_rho_S_;

};// end class SimpleDynamicModel
} // end namespace pegasus_sim

#endif // SIMPLE_DYNAMIC_MODEL_H
