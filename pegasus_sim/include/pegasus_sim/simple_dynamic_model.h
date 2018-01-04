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

};// end class SimpleDynamicModel
} // end namespace pegasus_sim

#endif // SIMPLE_DYNAMIC_MODEL_H
