#ifndef PD_H
#define PD_H

#include <pegasus/controller_base.h>

namespace pegasus
{
class PD : public Controller
{
public:
  PD();

private:
  virtual void control(const ros::TimerEvent& event);

};// end class PD
} // end namespace pegasus

#endif // PD_H
