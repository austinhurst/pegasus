#ifndef DYNAMIC_MODEL_H
#define DYNAMIC_MODEL_H

#include <pegasus_sim/equations_of_motion_base.h>

namespace pegasus_sim
{
class DynamicModel : public EquationsOfMotion
{
public:
  DynamicModel();

private:
  virtual pegasus::state_struct derivative(pegasus::state_struct s);
  float invJ11_;
  float invJ12_;
  float invJ13_;
  float invJ21_;
  float invJ22_;
  float invJ23_;
  float invJ31_;
  float invJ32_;
  float invJ33_;
  float piD30_;

};// end class DynamicModel
} // end namespace pegasus_sim

#endif // DYNAMIC_MODEL_H
