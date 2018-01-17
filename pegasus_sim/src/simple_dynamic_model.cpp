#include "pegasus_sim/simple_dynamic_model.h"

namespace pegasus_sim
{
SimpleDynamicModel::SimpleDynamicModel()
{
  float det_J = Jx_*(Jy_*Jz_ - Jyz_*Jyz_) - Jxy_*(Jxy_*Jz_ + Jxz_*Jyz_) - Jxz_*(Jxy_*Jyz_ + Jxz_*Jy_);
  invJ11_ = (Jy_ *Jz_  - Jyz_*Jyz_)/det_J;
  invJ12_ = (Jxy_*Jz_  + Jxz_*Jyz_)/det_J;
  invJ13_ = (Jxy_*Jyz_ + Jxz_*Jy_ )/det_J;
  invJ21_ = invJ12_;
  invJ22_ = (Jx_ *Jz_  - Jxz_*Jxz_)/det_J;
  invJ23_ = (Jx_ *Jyz_ + Jxy_*Jxz_)/det_J;
  invJ31_ = invJ13_;
  invJ32_ = invJ23_;
  invJ33_ = (Jx_ * Jy_  - Jxy_*Jxy_)/det_J;
}
pegasus::state_struct SimpleDynamicModel::derivative(pegasus::state_struct s)
{
  // IMPLEMENT MODEL EQUATIONS OF MOTION HERE:
  pegasus::state_struct derivative_of;

  // KINEMATICS
  // (pn_dot, pe_dot, pd_dot)^T = Rb_v*(u, v, w)^T
  float c_phi   = cosf(s.phi);
  float s_phi   = sinf(s.phi);
  float c_theta = cosf(s.theta);
  float s_theta = sinf(s.theta);
  float c_psi   = cosf(s.psi);
  float s_psi   = sinf(s.psi);
  derivative_of.pn = c_theta*c_psi*s.u + (s_phi*s_theta*c_psi-c_phi*s_psi)*s.v + (c_phi*s_theta*c_psi+s_phi*s_psi)*s.w;
  derivative_of.pe = c_theta*s_psi*s.u + (s_phi*s_theta*s_psi-c_phi*c_psi)*s.v + (c_phi*s_theta*s_psi+s_phi*c_psi)*s.w;
  derivative_of.pd =      -s_theta*s.u +                     s_phi*c_theta*s.v +                     c_phi*c_theta*s.w;

  // solve for (phi_dot, theta_dot, psi_dot)^T -->  (p, q, r)^T = (phi_dot, 0, 0)^T + Rv2_b*(0, theta_dot, 0)^T + Rv1_b*(0, 0, psi_dot)^T
  float t_theta = tanf(s.theta);
  float sec_theta = 1.0f/c_theta;
  derivative_of.phi   = s.p +   s_phi*t_theta*s.q +   c_phi*t_theta*s.r;
  derivative_of.theta =                 c_phi*s.q -           s_phi*s.r;
  derivative_of.psi   =       s_phi*sec_theta*s.q + c_phi*sec_theta*s.r;


  // DYNAMICS
  // TODO: Get the right forces and moments

    // (u_dot, v_dot, w_dot)^T = (r*v - q*w, p*w - r*u, q*u - p*v)^T + 1/m*(fx, fy, fz)^T
  float fx = 0.0;
  float fy = 0.0;
  float fz = 0.0;
  float l  = 0.0;
  float m  = 0.0;
  float n  = 0.0;
  derivative_of.u = s.r*s.v - s.q*s.w + fx/mass_;
  derivative_of.v = s.p*s.w - s.r*s.u + fy/mass_;
  derivative_of.w = s.p*s.u - s.p*s.v + fz/mass_;

  // (p_dot, q_dot, r_dot)^T = J^-1*(-omega_b/i^b x (J*omega_b/i^b) + m^b)          omega_b/i^b = (p,q,r)^T
  float negwJwm_i = -s.q*(-Jxz_*s.p - Jyz_*s.q + Jz_ *s.r) + s.r*(-Jxy_*s.p + Jy_ *s.q - Jyz_*s.r) + l;
  float negwJwm_j =  s.p*(-Jxz_*s.p - Jyz_*s.q + Jz_ *s.r) - s.r*( Jx_ *s.p - Jxy_*s.q - Jxz_*s.r) + m;
  float negwJwm_k = -s.p*(-Jxy_*s.p + Jy_ *s.q - Jyz_*s.r) + s.q*( Jx_ *s.p - Jxy_*s.q - Jxz_*s.r) + n;
  derivative_of.p = invJ11_*negwJwm_i + invJ12_*negwJwm_j + invJ13_*negwJwm_k;
  derivative_of.q = invJ21_*negwJwm_i + invJ22_*negwJwm_j + invJ23_*negwJwm_k;
  derivative_of.r = invJ31_*negwJwm_i + invJ32_*negwJwm_j + invJ33_*negwJwm_k;

  return derivative_of;
}
} // end namespace pegasus_simx
