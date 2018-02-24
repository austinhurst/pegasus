#ifndef PID_H
#define PID_H

#include <pegasus/controller_base.h>

namespace pegasus
{
class PID : public Controller
{
public:
  PID();

private:
  virtual void control(const ros::TimerEvent& event);

};// end class PID
} // end namespace pegasus

#endif // PID_H
