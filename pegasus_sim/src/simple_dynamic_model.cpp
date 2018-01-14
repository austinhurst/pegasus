#include "pegasus_sim/simple_dynamic_model.h"

namespace pegasus_sim
{
SimpleDynamicModel::SimpleDynamicModel()
{

}
pegasus::state_struct SimpleDynamicModel::derivative(pegasus::state_struct s)
{
  // IMPLEMENT MODEL EQUATIONS OF MOTION HERE:
  pegasus::state_struct derivative_of;
  
  return derivative_of;
}
} // end namespace pegasus_sim
