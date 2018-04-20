#ifndef GPS_STRUCT_H
#define GPS_STRUCT_H

#include <math.h>

namespace pegasus
{
  struct gps_struct
  {
    //*********************** VARIABLES **********************//
  private:
    float piD180_;   // pi/180
    float a_;        // length of Earth's semi-major axis in meters
    float b_;        // length of Earth's semi-minor axis in meters
    float e2_;       // e = first numerical eccentricity, e2_ = e^2
    float R_[3][3];  // rotational matrix from ECEF to NED
    float xr_;       // ECEF x coordinate of reference point
    float yr_;       // ECEF y coordinate of reference point
    float zr_;       // ECEF z coordinate of reference point
    float epsilon_;  // used in ned2gps

  public:
    gps_struct()
    {
      piD180_         = M_PI/180.0;
      a_              = 6378137.0;
      b_              = 6356752.3142;
      e2_             = 1.0 - pow((b_/a_), 2.0);
    }
    //****************** OPERATOR FUNCTIONS ******************//

    //********************** FUNCTIONS ***********************//
    void set_reference(float r_lat_deg, float r_lon_deg, float r_height_m)
    {
      float r_phi    = r_lat_deg*piD180_;     // reference latitude  (N)
      float r_lambda = r_lon_deg*piD180_;     // reference longitude (E)
      float r_height = r_height_m;            // reference height in meters
      float nu0      = a_/sqrt(1.0 - e2_*sin(r_phi)*sin(r_phi));
      float s_r_phi  = sin(r_phi);
      float c_r_phi  = cos(r_phi);
      float s_r_lam  = sin(r_lambda);
      float c_r_lam  = cos(r_lambda);
      R_[0][0]        = -s_r_phi*c_r_lam;
      R_[0][1]        = -s_r_phi*s_r_lam;
      R_[0][2]        =  c_r_phi;
      R_[1][0]        = -s_r_lam;
      R_[1][1]        =  c_r_lam;
      R_[1][2]        =  0.0;
      R_[2][0]        = -c_r_phi*c_r_lam;
      R_[2][1]        = -c_r_phi*s_r_lam;
      R_[2][2]        = -s_r_phi;
      xr_             = (nu0 + r_height)*c_r_phi*c_r_lam;
      yr_             = (nu0 + r_height)*c_r_phi*s_r_lam;
      zr_             = (nu0*(1.0 - e2_) + r_height)*s_r_phi;
      epsilon_        = e2_/(1.0 - e2_);
    }
    void gps2ned(float lat_N, float lon_E, float h_M, float& N, float& E, float& D)
    {
      float phi      = lat_N*piD180_;
      float lambda   = lon_E*piD180_;

      // Convert the angles into Earth Centered Earth Fixed Reference Frame
      float s_phi    = sin(phi);
      float c_phi    = cos(phi);
      float s_lam    = sin(lambda);
      float c_lam    = cos(lambda);
      float nu       = a_/sqrt(1.0 - e2_*s_phi*s_phi);
      float x        = (nu + h_M)*c_phi*c_lam;
      float y        = (nu + h_M)*c_phi*s_lam;
      float z        = (nu*(1.0 - e2_) + h_M)*s_phi;

      // Find the difference between the point x, y, z to the reference point in ECEF
      float dx       = x - xr_;
      float dy       = y - yr_;
      float dz       = z - zr_;

      N               = R_[0][0]*dx + R_[0][1]*dy + R_[0][2]*dz;
      E               = R_[1][0]*dx + R_[1][1]*dy + R_[1][2]*dz;
      D               = R_[2][0]*dx + R_[2][1]*dy + R_[2][2]*dz;
    }
    void ned2gps(float N, float E, float D, float& lat_N, float& lon_E, float& h_M)
    {
      // Convert from NED to ECEF
      float dx       = R_[0][0]*N + R_[1][0]*E + R_[2][0]*D;
      float dy       = R_[0][1]*N + R_[1][1]*E + R_[2][1]*D;
      float dz       = R_[0][2]*N + R_[1][2]*E + R_[2][2]*D;
      float x        = dx + xr_;
      float y        = dy + yr_;
      float z        = dz + zr_;

      // Convert from ECEF to GPS
      float p        = sqrt(x*x + y*y);
      float q        = atan2(z*a_, p*b_);
      float phi      = atan2(z + epsilon_*b_*pow(sin(q),3.0),p - e2_*a_*pow(cos(q),3.0));
      float lambda   = atan2(y,x);
      float nu       = a_/sqrt(1.0 - e2_*sin(phi)*sin(phi));

      h_M             = p/cos(phi) - nu;
      lat_N           = phi/piD180_;
      lon_E           = lambda/piD180_;
    }
  };
} // end namespace pegasus
#endif // GPS_STRUCT_H
