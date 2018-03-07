#include <pegasus_sim/flat_momentum.h>

namespace pegasus_sim
{
FlatMomentum::FlatMomentum()
{
  if (!(ros::param::get("/pegasus/vehicle_description/g",g_)))
    ROS_WARN("No param named 'g");
  if (!(ros::param::get("/pegasus/vehicle_description/mass",mass_)))
    ROS_WARN("No param named 'mass");
  if (!(ros::param::get("/pegasus/vehicle_description/motors/K_delta_t",K_delta_t_)))
    ROS_WARN("No param named 'K_delta_t");
  if (!(ros::param::get("/pegasus/vehicle_description/motors/KQ",KQ_)))
    ROS_WARN("No param named 'KQ");
  if (!(ros::param::get("/pegasus/vehicle_description/motors/Vb",Vb_)))
    ROS_WARN("No param named 'Vb");
  if (!(ros::param::get("/pegasus/vehicle_description/motors/Kv",Kv_)))
    ROS_WARN("No param named 'Kv");
  if (!(ros::param::get("/pegasus/vehicle_description/motors/Rm",Rm_)))
    ROS_WARN("No param named 'Rm");
  if (!(ros::param::get("/pegasus/vehicle_description/motors/i0",i0_)))
    ROS_WARN("No param named 'i0");
  if (!(ros::param::get("/pegasus/vehicle_description/motors/Dp",Dp_)))
    ROS_WARN("No param named 'Dp");
  if (!(ros::param::get("/pegasus/vehicle_description/rho",rho_)))
    ROS_WARN("No param named 'rho");
  if (!(ros::param::get("/pegasus/vehicle_description/S",S_)))
    ROS_WARN("No param named 'S");
  if (!(ros::param::get("/pegasus/vehicle_description/b",b_)))
    ROS_WARN("No param named 'b");
  if (!(ros::param::get("/pegasus/vehicle_description/c",c_)))
    ROS_WARN("No param named 'c");

  Dp_          = Dp_*0.0254f;
  half_rho_S_  = 0.5f*rho_*S_;
  mg_          = mass_*g_;
  Ap_          = M_PI*Dp_*Dp_/4.0f;
  piD30_       = M_PI/30.0f;

  //************** SUBSCRIBERS AND PUBLISHERS **************//

  //******************** CLASS VARIABLES *******************//

}
FlatMomentum::~FlatMomentum()
{

}
void FlatMomentum::getForcesAndMoments(pegasus::state_struct s,
                                           float& fx, float& fy, float& fz,
                                           float& l , float& m , float& n)
{
  float c_phi   = cosf(s.phi);
  float s_phi   = sinf(s.phi);
  float c_theta = cosf(s.theta);
  float s_theta = sinf(s.theta);
  float c_psi   = cosf(s.psi);
  float s_psi   = sinf(s.psi);

  float Rb_v[3][3];
  Rb_v[0][0] =  c_theta*c_psi;
  Rb_v[0][1] =  s_phi*s_theta*c_psi - c_phi*s_psi;
  Rb_v[0][2] =  c_phi*s_theta*c_psi + s_phi*s_psi;
  Rb_v[1][0] =  c_theta*s_psi;
  Rb_v[1][1] =  s_phi*s_theta*s_psi + c_phi*c_psi;
  Rb_v[1][2] =  c_phi*s_theta*s_psi - s_phi*c_psi;
  Rb_v[2][0] = -s_theta;
  Rb_v[2][1] =  s_phi*c_theta;
  Rb_v[2][2] =  c_phi*c_theta;

  //********** FORCES AND MOMENTS *************//
  // Force due to grvity
  float fx_g = mg_*Rb_v[2][0];
  float fy_g = mg_*Rb_v[2][1];
  float fz_g = mg_*Rb_v[2][2];

  // Force due to wind
  float fx_w, fy_w, fz_w, l_w, m_w, n_w;

  // Compute wind in body frame
  float w_x = w_ns_*Rb_v[0][0] + w_es_*Rb_v[1][0] + w_ds_*Rb_v[2][0] + w_xg_;
  float w_y = w_ns_*Rb_v[0][1] + w_es_*Rb_v[1][1] + w_ds_*Rb_v[2][1] + w_yg_;
  float w_z = w_ns_*Rb_v[0][2] + w_es_*Rb_v[1][2] + w_ds_*Rb_v[2][2] + w_zg_;

  // Compute body frame air vector
  float ur = s.u - w_x;
  float vr = s.v - w_y;
  float wr = s.w - w_z;
  float Va = sqrtf(ur*ur + vr*vr + wr*wr);

  if (Va == 0.0f)
  {
    fx_w = 0.0f;
    fy_w = 0.0f;
    fz_w = 0.0f;
    l_w  = 0.0f;
    m_w  = 0.0f;
    n_w  = 0.0f;
  }
  else
  {
    float alpha, gamma, beta;
    if (ur == 0.0f && wr == 0.0f)
      alpha = 0.0f;
    else if (ur == 0.0f)
      alpha = M_PI*wr/fabs(wr);
    else
      alpha = atan2f(wr,ur);
    if (vr == 0.0f)
      gamma = 0.0f;
    else
      gamma = atan2f(wr,vr);
    beta    = asinf(vr/Va);

    float c_alpha   = cosf(alpha);
    float s_alpha   = sinf(alpha);
    float c_gamma   = cosf(gamma);
    float s_gamma   = sinf(gamma);
    float sgn_alpha = (alpha == 0.0f) ? 0.0f : alpha/fabs(alpha);
    float sgn_gamma = (gamma == 0.0f) ? 0.0f : gamma/fabs(gamma);
    float sgn_wr    = (wr    == 0.0f) ? 0.0f : wr   /fabs(wr);

    // Compute Coefficients
    float CL_alpha =  2.0f*s_alpha*s_alpha*c_alpha*sgn_alpha;
    float CD_alpha =  2.0f*powf(s_alpha,3.0f);
    float CM_alpha = -s_alpha*s_alpha*sgn_alpha;
    float CL_gamma =  2.0f*s_gamma*s_gamma*c_gamma*sgn_gamma;
    float CD_gamma =  2.0f*powf(s_gamma,3.0f);
    float CM_gamma = -s_gamma*s_gamma*sgn_gamma;

    // Compute Flift, Fdrag, moments for both alpha and gamma
    float f_C_alpha = (ur*ur + wr*wr)*half_rho_S_;
    float f_C_gamma = (vr*vr + wr*wr)*half_rho_S_;
    float fl_alpha  = f_C_alpha*CL_alpha;
    float fd_alpha  = f_C_alpha*CD_alpha;
    float m_alpha   = f_C_alpha*CM_alpha*c_;
    float fl_gamma  = f_C_gamma*CL_gamma;
    float fd_gamma  = f_C_gamma*CD_gamma;
    float m_gamma   = f_C_gamma*CM_gamma*b_;

    // Put forces into body frame
    fx_w = -c_alpha*fd_alpha + s_alpha*fl_alpha;
    fy_w = -c_gamma*fd_gamma + s_gamma*fl_gamma;
    fz_w = -s_alpha*fd_alpha - c_alpha*fl_alpha - s_gamma*fd_gamma - c_gamma*fl_gamma;
    // l_w  =  0.0f;
    // m_w  =  0.0f;
    // TODO: figure out if these moments are okay or not.
    l_w  = -m_gamma;
    m_w  =  m_alpha;
    n_w  =  0.0f;
  }
  // Force due to Propultion
  float fx_p, fy_p, fz_p, l_p, m_p, n_p;
  fx_p = 0.0f;
  fy_p = 0.0f;
  fz_p = 0.0f;
  l_p  = 0.0f;
  m_p  = 0.0f;
  n_p  = 0.0f;

  for (int ii = 0; ii < num_motors_; ii++)
  {
    pegasus::motor_description *md;
    float delta_m;
    switch (ii)
    {
      case 0: md = &m1d_; delta_m = motors_->m1; break;
      case 1: md = &m2d_; delta_m = motors_->m2; break;
      case 2: md = &m3d_; delta_m = motors_->m3; break;
      case 3: md = &m4d_; delta_m = motors_->m4; break;
      case 4: md = &m5d_; delta_m = motors_->m5; break;
      case 5: md = &m6d_; delta_m = motors_->m6; break;
      case 6: md = &m7d_; delta_m = motors_->m7; break;
      case 7: md = &m8d_; delta_m = motors_->m8; break;
    }

    delta_m  =  delta_m < 0.0f ? 0.0f : delta_m;

    // Model of a DC Motor
    float omega, i, Qm, P_shaft, T, Q;
    omega   = K_delta_t_*delta_m;
    i       = (Vb_ - omega/Kv_)/Rm_;
    Qm      = (i-i0_)/(Kv_*piD30_);
    P_shaft = Qm*omega*piD30_;

    // Momentum Theory
    T = pow(P_shaft*P_shaft*2.0f*rho_*Ap_, 1.0f/3.0f);
    Q = T/6.30f*md->dir;

    // Put the Forces and Torques into the correct orientation
    fx_p += T*md->Tx;
    fy_p += T*md->Ty;
    fz_p += T*md->Tz;
    l_p  += T*md->Tz*md->y - T*md->Ty*md->z + Q*md->Tx;
    m_p  += T*md->Tx*md->z - T*md->Tz*md->x + Q*md->Ty;
    n_p  += T*md->Ty*md->x - T*md->Tx*md->y + Q*md->Tz;
  }

  // Total Forces
  fx = fx_g + fx_w + fx_p;
  fy = fy_g + fy_w + fy_p;
  fz = fz_g + fz_w + fz_p;
  l  =        l_w  + l_p;
  m  =        m_w  + m_p;
  n  =        n_w  + n_p;

  if (s.pd == 0.0f)
    fz -= mg_;
}
} // end namespace pegasus_sim
