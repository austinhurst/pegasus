#include "pegasus_sim/simple_dynamic_model.h"

namespace pegasus_sim
{
SimpleDynamicModel::SimpleDynamicModel()
{
  if (!(ros::param::get("sim/wind/w_ns",w_ns_)))
    ROS_WARN("No param named 'w_ns'");
  if (!(ros::param::get("sim/wind/w_es",w_es_)))
    ROS_WARN("No param named 'w_es'");
  if (!(ros::param::get("sim/wind/w_ds",w_ds_)))
    ROS_WARN("No param named 'w_ds'");
  if (!(ros::param::get("sim/cd_flat_plate",cd_flat_plate_)))
    ROS_WARN("No param named 'cd_flat_plate'");

  float det_J = Jx_*(Jy_*Jz_ - Jyz_*Jyz_) - Jxy_*(Jxy_*Jz_ + Jxz_*Jyz_) - Jxz_*(Jxy_*Jyz_ + Jxz_*Jy_);
  invJ11_ = (Jy_ *Jz_  - Jyz_*Jyz_)/det_J;
  invJ12_ = (Jxy_*Jz_  + Jxz_*Jyz_)/det_J;
  invJ13_ = (Jxy_*Jyz_ + Jxz_*Jy_ )/det_J;
  invJ21_ = invJ12_;
  invJ22_ = (Jx_ *Jz_  - Jxz_*Jxz_)/det_J;
  invJ23_ = (Jx_ *Jyz_ + Jxy_*Jxz_)/det_J;
  invJ31_ = invJ13_;
  invJ32_ = invJ23_;
  invJ33_ = (Jx_ * Jy_ - Jxy_*Jxy_)/det_J;
  half_rho_S_   = 0.5f*rho_*S_;
}
pegasus::state_struct SimpleDynamicModel::derivative(pegasus::state_struct s)
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
  float fx_g = -mass_*g_*s_theta;
  float fy_g =  mass_*g_*c_theta*s_phi;
  float fz_g =  mass_*g_*c_theta*c_phi;

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
    if (ur == 0.0f)
      alpha = 0.0f;
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
    float sgn_wr    = (wr == 0.0f) ? 0.0f : wr/fabs(wr);

    float CL_alpha = 2.0f*s_alpha*s_alpha*c_alpha*sgn_alpha;
    float CD_alpha = 2.0f*powf(s_alpha,3.0f);
    float CM_alpha = -s_alpha*s_alpha*sgn_alpha;
    float CL_gamma = 2.0f*s_gamma*s_gamma*c_gamma*sgn_gamma;
    float CD_gamma = 2.0f*powf(s_gamma,3.0f);
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
    float fd_z      = half_rho_S_*wr*wr*cd_flat_plate_*sgn_wr;

    // Put forces into body frame
    fx_w = -c_alpha*fd_alpha + s_alpha*fl_alpha;
    fy_w = -c_gamma*fd_gamma + s_gamma*fl_gamma;
    fz_w = -s_alpha*fd_alpha - c_alpha*fl_alpha - s_gamma*fd_gamma - c_gamma*fl_gamma - fd_z;
    l_w  = -m_gamma;
    m_w  =  m_alpha;
    n_w  = 0.0f;
  }
  // Force due to Propultion
  float fx_p, fy_p, fz_p, l_p, m_p, n_p;
  fx_p = 0.0f;
  fy_p = 0.0f;
  fz_p = 0.0f;
  l_p  = 0.0f;
  m_p  = 0.0f;
  n_p  = 0.0f;

  // Total Forces
  float fx = fx_g + fx_w + fx_p;
  float fy = fy_g + fy_w + fy_p;
  float fz = fz_g + fz_w + fz_p;
  float l  =        l_w  + l_p;
  float m  =        m_w  + m_p;
  float n  =        n_w  + n_p;

  //********** EQUATIONS OF MOTION ************//
  pegasus::state_struct derivative_of;

  // KINEMATICS
  // (pn_dot, pe_dot, pd_dot)^T = Rb_v*(u, v, w)^T
  derivative_of.pn = c_theta*c_psi*s.u + (s_phi*s_theta*c_psi-c_phi*s_psi)*s.v + (c_phi*s_theta*c_psi+s_phi*s_psi)*s.w;
  derivative_of.pe = c_theta*s_psi*s.u + (s_phi*s_theta*s_psi-c_phi*c_psi)*s.v + (c_phi*s_theta*s_psi+s_phi*c_psi)*s.w;
  derivative_of.pd =      -s_theta*s.u +                     s_phi*c_theta*s.v +                     c_phi*c_theta*s.w;

  // solve for (phi_dot, theta_dot, psi_dot)^T -->  (p, q, r)^T = (phi_dot, 0, 0)^T + Rv2_b*(0, theta_dot, 0)^T + Rv1_b*(0, 0, psi_dot)^T
  float t_theta       = tanf(s.theta);
  float sec_theta     = 1.0f/c_theta;
  derivative_of.phi   = s.p +   s_phi*t_theta*s.q +   c_phi*t_theta*s.r;
  derivative_of.theta =                 c_phi*s.q -           s_phi*s.r;
  derivative_of.psi   =       s_phi*sec_theta*s.q + c_phi*sec_theta*s.r;

  // DYNAMICS
  // (u_dot, v_dot, w_dot)^T = (r*v - q*w, p*w - r*u, q*u - p*v)^T + 1/m*(fx, fy, fz)^T
  derivative_of.u = s.r*s.v - s.q*s.w + fx/mass_;
  derivative_of.v = s.p*s.w - s.r*s.u + fy/mass_;
  derivative_of.w = s.q*s.u - s.p*s.v + fz/mass_;

  // (p_dot, q_dot, r_dot)^T = J^-1*(-omega_b/i^b x (J*omega_b/i^b) + m^b)          omega_b/i^b = (p,q,r)^T
  float negwJwm_i = -s.q*(-Jxz_*s.p - Jyz_*s.q + Jz_ *s.r) + s.r*(-Jxy_*s.p + Jy_ *s.q - Jyz_*s.r) + l;
  float negwJwm_j =  s.p*(-Jxz_*s.p - Jyz_*s.q + Jz_ *s.r) - s.r*( Jx_ *s.p - Jxy_*s.q - Jxz_*s.r) + m;
  float negwJwm_k = -s.p*(-Jxy_*s.p + Jy_ *s.q - Jyz_*s.r) + s.q*( Jx_ *s.p - Jxy_*s.q - Jxz_*s.r) + n;
  derivative_of.p = invJ11_*negwJwm_i + invJ12_*negwJwm_j + invJ13_*negwJwm_k;
  derivative_of.q = invJ21_*negwJwm_i + invJ22_*negwJwm_j + invJ23_*negwJwm_k;
  derivative_of.r = invJ31_*negwJwm_i + invJ32_*negwJwm_j + invJ33_*negwJwm_k;

  return derivative_of;
}
void SimpleDynamicModel::eachTimeStep()
{
  // function for calculating parameters only once each timestep
  // Calculate Wind Gust Speeds
  w_xg_ = 0.0;
  w_yg_ = 0.0;
  w_zg_ = 0.0;
}
} // end namespace pegasus_simx
