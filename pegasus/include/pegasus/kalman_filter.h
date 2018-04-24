#ifndef KALMAN_FILTER_H
#define KALMAN_FILTER_H

#include <pegasus/estimator_base.h>
#include <math.h>

#define NUM_STATES 5 // phi, theta, psi, u1, v1

namespace pegasus
{
struct LPF
{
  float last_value_;
  float cutoff_frequency_;
  LPF()
  {
    last_value_ = 0.0f;
  }
  float filter(float new_data, float ts)
  {
    float alpha   = exp(-cutoff_frequency_*ts);
    float new_val = alpha*last_value_ + (1.0f - alpha)*new_data;
    last_value_   = new_val;
    return new_val;

  }
}; // end struct LPF
class KalmanFilter : public Estimator
{
public:
  KalmanFilter();

private:
  virtual void predict(const ros::TimerEvent& event);
  virtual void correctBarometer();
  virtual void correctGPS();
  virtual void correctIMU();

  float xhat_[NUM_STATES];
  int N_;
  float f_[NUM_STATES];
  float A_[NUM_STATES][NUM_STATES];
  float P_[NUM_STATES][NUM_STATES];
  float Q_[NUM_STATES][NUM_STATES];
  float K_[NUM_STATES];

  float RVg_;
  float Rchi_;
  float Racc_;

  ros::Time gyro_last_time_;
  LPF gyro_p_;
  LPF gyro_q_;
  LPF gyro_r_;

  ros::Time barometer_last_time_;
  LPF barometer_;

};// end class KalmanFilter
} // end namespace pegasus

#endif // KALMAN_FILTER_H
