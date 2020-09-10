// Dynamics used for EKF
#ifndef DYNAMICS_EKF_H
#define DYNAMICS_EKF_H

#include <math.h>
#include <vector>
#include <stdint.h>
#include <cstring>

static const uint32_t STATE_LENGTH = 7;
static const uint32_t INPUT_LENGTH = 2;
static const uint32_t MEASUREMENT_LENGTH = 4;

static const double model[15] = {44.798,            //mb
                                 2.485,             //mw
                                 0.055936595310797, //Jw
                                 -0.02322718759275, //a2
                                 0.166845864363019, //c2
                                 3.604960049044268, //A2
                                 2*3.836289730154863, //B2
                                 1.069672194414735, //C2
                                 1.5, //K
                                 0.195,             //r
                                 0.5,               //L
                                 9.81,              //gGravity
                                 1.1,                //FricCoeff 3.185188257847262
                                 1.0e-3,            //velEps
                                 1.225479467549329  //FricCoeff 1.225479467549329
                                 };

static double rt_powd_snf(double u0, double u1)
{
 double y;
 y = pow(u0, u1);
 return y;
}


inline void dynamics(const double t,
              const double X[STATE_LENGTH],
              const double U[INPUT_LENGTH],
                    double xDot[STATE_LENGTH])
{
   double g[STATE_LENGTH*INPUT_LENGTH];
   double f[STATE_LENGTH];
   double Fric;
   double a_tmp;
   double b_a_tmp;
   double f_tmp;
   double b_f_tmp;
   double c_f_tmp;
   double d_f_tmp;
   double e_f_tmp;
   double f_f_tmp;
   double g_f_tmp;
   double h_f_tmp;
   double i_f_tmp;
   double j_f_tmp;
   double k_f_tmp;
   double l_f_tmp;
   double m_f_tmp;
   double n_f_tmp;
   double o_f_tmp;
   double p_f_tmp;
   double q_f_tmp;
   double r_f_tmp;
   double s_f_tmp;
   double t_f_tmp;
   double u_f_tmp;
   double v_f_tmp;
   double f_tmp_tmp;
   double b_f_tmp_tmp;
   double w_f_tmp;
   double x_f_tmp;

   /*  */
   Fric = X[3] - X[6] * model[9];
   Fric = model[12] * tanh(Fric / model[13]) + model[14] * Fric;
   a_tmp = cos(X[5]);
   b_a_tmp = sin(X[5]);
   f[0] = X[3] * cos(X[2]);
   f[1] = X[3] * sin(X[2]);
   f[2] = X[4];
   f_tmp = model[3] * model[3];
   b_f_tmp = model[9] * model[9];
   c_f_tmp = model[4] * model[4];
   d_f_tmp = model[0] * model[0];
   e_f_tmp = 4.0 * f_tmp;
   f_f_tmp = 4.0 * c_f_tmp;
   g_f_tmp = X[4] * X[4];
   h_f_tmp = X[6] * X[6];
   i_f_tmp = 4.0 * h_f_tmp + 3.0 * g_f_tmp;
   j_f_tmp = cos(2.0 * X[5]);
   k_f_tmp = cos(3.0 * X[5]);
   l_f_tmp = rt_powd_snf(model[3], 3.0);
   m_f_tmp = 4.0 * model[6] * model[4] * model[0];
   n_f_tmp = rt_powd_snf(model[4], 3.0);
   o_f_tmp = sin(2.0 * X[5]);
   p_f_tmp = model[5] * model[4] * model[0] * model[9] * g_f_tmp;
   q_f_tmp = -model[4] * model[7] * model[0] * model[9] * g_f_tmp;
   r_f_tmp = sin(3.0 * X[5]);
   s_f_tmp = 3.0 * f_tmp * model[4] * d_f_tmp * model[9] * g_f_tmp;
   t_f_tmp = -4.0 * model[3] * model[4];
   u_f_tmp = 2.0 * model[3] * model[4];
   v_f_tmp = f_tmp * d_f_tmp;
   f_tmp_tmp = v_f_tmp * b_f_tmp;
   b_f_tmp_tmp = c_f_tmp * d_f_tmp * b_f_tmp;
   f_f_tmp = 1.0 / ((((((((((4.0 * model[6] * model[2] + e_f_tmp * model[2] *
     model[0]) + f_f_tmp * model[2] * model[0]) + 2.0 * model[6] * model[0] *
     b_f_tmp) + f_tmp_tmp) + b_f_tmp_tmp) + 4.0 * model[6] * model[1] * b_f_tmp)
                       + e_f_tmp * model[0] * model[1] * b_f_tmp) + f_f_tmp *
                      model[0] * model[1] * b_f_tmp) + (f_tmp + -c_f_tmp) *
                     d_f_tmp * b_f_tmp * j_f_tmp) + u_f_tmp * d_f_tmp * b_f_tmp *
                    o_f_tmp);
   w_f_tmp = 2.0 * f_tmp;
   f[3] = 0.5 * model[9] * f_f_tmp * (((((((((((((((((((((((-8.0 * model[6] *
     Fric + -8.0 * f_tmp * Fric * model[0]) + -8.0 * c_f_tmp * Fric * model[0]) +
     model[0] * model[9] * ((((-8.0 * model[4] * Fric + model[3] * (-model[5] +
     model[7]) * g_f_tmp) + 4.0 * model[3] * model[6] * (h_f_tmp + g_f_tmp)) +
     l_f_tmp * model[0] * i_f_tmp) + model[3] * c_f_tmp * model[0] * i_f_tmp) *
     a_tmp) + t_f_tmp * model[11] * d_f_tmp * model[9] * j_f_tmp) + model[3] *
     model[5] * model[0] * model[9] * g_f_tmp * k_f_tmp) + -model[3] * model[7] *
     model[0] * model[9] * g_f_tmp * k_f_tmp) + l_f_tmp * d_f_tmp * model[9] *
     g_f_tmp * k_f_tmp) + -3.0 * model[3] * c_f_tmp * d_f_tmp * model[9] *
     g_f_tmp * k_f_tmp) + 8.0 * model[3] * Fric * model[0] * model[9] * b_a_tmp)
     + m_f_tmp * h_f_tmp * model[9] * b_a_tmp) + e_f_tmp * model[4] * d_f_tmp *
     h_f_tmp * model[9] * b_a_tmp) + 4.0 * n_f_tmp * d_f_tmp * h_f_tmp * model[9]
     * b_a_tmp) + p_f_tmp * b_a_tmp) + m_f_tmp * model[9] * g_f_tmp * b_a_tmp) +
     q_f_tmp * b_a_tmp) + s_f_tmp * b_a_tmp) + 3.0 * n_f_tmp * d_f_tmp * model[9]
     * g_f_tmp * b_a_tmp) + w_f_tmp * model[11] * d_f_tmp * model[9] * o_f_tmp) +
     -2.0 * c_f_tmp * model[11] * d_f_tmp * model[9] * o_f_tmp) + p_f_tmp *
     r_f_tmp) + q_f_tmp * r_f_tmp) + s_f_tmp * r_f_tmp) + -n_f_tmp * d_f_tmp *
     model[9] * g_f_tmp * r_f_tmp);
   e_f_tmp = model[10] * model[10];
   i_f_tmp = -2.0 * f_tmp;
   k_f_tmp = 2.0 * c_f_tmp;
   l_f_tmp = i_f_tmp * model[0];
   m_f_tmp = k_f_tmp * model[0];
   n_f_tmp = f_tmp * model[0];
   c_f_tmp *= model[0];
   p_f_tmp = model[4] * model[0];
   q_f_tmp = model[2] * e_f_tmp;
   e_f_tmp *= model[1];
   r_f_tmp = 2.0 * (model[7] + n_f_tmp);
   s_f_tmp = 2.0 * (model[5] + c_f_tmp);
   u_f_tmp *= model[0];
   f[4] = b_f_tmp * X[4] * ((-2.0 * model[3] * model[0] * X[3] * a_tmp + t_f_tmp *
     model[0] * X[6] * j_f_tmp) + -2.0 * (p_f_tmp * X[3] + (((model[5] + -model[7])
     + l_f_tmp) + m_f_tmp) * X[6] * a_tmp) * b_a_tmp) * (1.0 / ((((q_f_tmp +
     e_f_tmp * b_f_tmp) + r_f_tmp * b_f_tmp * (a_tmp * a_tmp)) + s_f_tmp *
     b_f_tmp * (b_a_tmp * b_a_tmp)) + u_f_tmp * b_f_tmp * o_f_tmp));
   f[5] = X[6];
   t_f_tmp = 4.0 * model[4] * model[11];
   k_f_tmp = k_f_tmp * model[2] * model[0];
   m_f_tmp = m_f_tmp * model[1] * b_f_tmp;
   x_f_tmp = -(model[4] * model[4]) * d_f_tmp;
   f[6] = f_f_tmp * ((((((((((((((((((((8.0 * Fric * model[2] + 4.0 * Fric *
     model[0] * b_f_tmp) + 8.0 * Fric * model[1] * b_f_tmp) + 2.0 * model[0] *
     (2.0 * model[4] * Fric * model[9] + model[3] * model[11] * (2.0 * model[2] +
     (model[0] + 2.0 * model[1]) * b_f_tmp)) * a_tmp) + -2.0 * model[3] * model[4]
     * model[0] * (model[0] * h_f_tmp * b_f_tmp + -2.0 * (model[2] + model[1] *
     b_f_tmp) * g_f_tmp) * j_f_tmp) + t_f_tmp * model[2] * model[0] * b_a_tmp) +
     -4.0 * model[3] * Fric * model[0] * model[9] * b_a_tmp) + 2.0 * model[4] *
     model[11] * d_f_tmp * b_f_tmp * b_a_tmp) + t_f_tmp * model[0] * model[1] *
     b_f_tmp * b_a_tmp) + v_f_tmp * h_f_tmp * b_f_tmp * o_f_tmp) + x_f_tmp *
     h_f_tmp * b_f_tmp * o_f_tmp) + -2.0 * model[5] * model[2] * g_f_tmp *
     o_f_tmp) + 2.0 * model[7] * model[2] * g_f_tmp * o_f_tmp) + i_f_tmp * model
     [2] * model[0] * g_f_tmp * o_f_tmp) + k_f_tmp * g_f_tmp * o_f_tmp) + -model
     [5] * model[0] * b_f_tmp * g_f_tmp * o_f_tmp) + model[7] * model[0] *
                         b_f_tmp * g_f_tmp * o_f_tmp) + -2.0 * model[5] * model[1]
                        * b_f_tmp * g_f_tmp * o_f_tmp) + 2.0 * model[7] * model[1]
                       * b_f_tmp * g_f_tmp * o_f_tmp) + l_f_tmp * model[1] *
                      b_f_tmp * g_f_tmp * o_f_tmp) + m_f_tmp * g_f_tmp * o_f_tmp);
   t_f_tmp = x_f_tmp * b_f_tmp;
   l_f_tmp = (((((((2.0 * model[6] * model[2] + w_f_tmp * model[2] * model[0]) +
                   k_f_tmp) + model[6] * model[0] * b_f_tmp) + f_tmp_tmp) +
                b_f_tmp_tmp) + 2.0 * model[6] * model[1] * b_f_tmp) + w_f_tmp *
              model[0] * model[1] * b_f_tmp) + m_f_tmp;
   j_f_tmp = -f_tmp * d_f_tmp * b_f_tmp;
   i_f_tmp = model[3] * model[4] * d_f_tmp * b_f_tmp * o_f_tmp;
   g_f_tmp = p_f_tmp * model[9] * a_tmp;
   h_f_tmp = -model[3] * model[0] * model[9] * b_a_tmp;
   Fric = model[8] * model[9] * ((((model[6] + n_f_tmp) + c_f_tmp) + g_f_tmp) +
     h_f_tmp);
   g[3] = Fric * (1.0 / (((l_f_tmp + t_f_tmp * (a_tmp * a_tmp)) + j_f_tmp *
     (b_a_tmp * b_a_tmp)) + i_f_tmp));
   g[10] = Fric * (1.0 / (((l_f_tmp + t_f_tmp * (a_tmp * a_tmp)) + j_f_tmp *
     (b_a_tmp * b_a_tmp)) + i_f_tmp));
   t_f_tmp = r_f_tmp * model[9];
   l_f_tmp = q_f_tmp * (1.0 / model[9]) + e_f_tmp * model[9];
   j_f_tmp = s_f_tmp * model[9];
   i_f_tmp = u_f_tmp * model[9] * o_f_tmp;
   g[4] = -model[8] * model[10] * (1.0 / (((l_f_tmp + t_f_tmp * (a_tmp * a_tmp))
     + j_f_tmp * (b_a_tmp * b_a_tmp)) + i_f_tmp));
   g[11] = model[8] * model[10] * (1.0 / (((l_f_tmp + t_f_tmp * (a_tmp * a_tmp))
     + j_f_tmp * (b_a_tmp * b_a_tmp)) + i_f_tmp));
   g[0] = 0.0;
   g[1] = 0.0;
   g[2] = 0.0;
   g[5] = 0.0;
   g[7] = 0.0;
   g[8] = 0.0;
   g[9] = 0.0;
   g[12] = 0.0;
   t_f_tmp = -2.0 * model[8] * ((((2.0 * model[2] + model[0] * b_f_tmp) + 2.0 *
     model[1] * b_f_tmp) + g_f_tmp) + h_f_tmp) * f_f_tmp;
   g[6] = t_f_tmp;
   g[13] = t_f_tmp;

   for(int i=0; i<STATE_LENGTH; i++)
   {
      for(int j=0; j<INPUT_LENGTH; j++)
      {
         xDot[i]=f[i]+g[i+j*STATE_LENGTH]*U[j];
      }
   }
}

inline void JfFun(const double in1[STATE_LENGTH], double Jf[STATE_LENGTH*STATE_LENGTH])
{
  double t2;
  double t3;
  double t4;
  double t5;
  double t6;
  double t7;
  double t8;
  double t9;
  double t10;
  double t11;
  double t12;
  double t13;
  double t14;
  double t15;
  double t16;
  double t17;
  double t18;
  double t19;
  double t20;
  double t21;
  double t22;
  double t32;
  double t24;
  double t25;
  double t27;
  double t28;
  double t29;
  double t33;
  double t41;
  double t42_tmp;
  double t43_tmp;
  double t44;
  double t46_tmp;
  double t46;
  double t54_tmp;
  double t66_tmp;
  double t66;
  double t57;
  double t60;
  double t61;
  double t62;
  double t69;
  double t74_tmp;
  double t74;
  double t76_tmp_tmp;
  double t76_tmp;
  double t81;
  double t89;
  double t91_tmp;
  double t107_tmp;
  double t107;
  double t114_tmp_tmp;
  double t114_tmp;
  double b_t114_tmp;
  double t114;
  double t117_tmp;
  double b_t117_tmp;
  double c_t117_tmp;
  double d_t117_tmp;
  double t117;
  double t120_tmp;
  double b_t120_tmp;
  double t120;
  double t94;
  double t118_tmp;
  double b_t118_tmp;
  double c_t118_tmp;
  double t118;
  double t121;
  double t115_tmp;
  double t115;
  double t109;
  double t112;
  double Jf_tmp_tmp;
  double b_Jf_tmp_tmp;
  double Jf_tmp;
  double c_Jf_tmp_tmp;
  double d_Jf_tmp_tmp;
  double b_Jf_tmp;
  double c_Jf_tmp;
  double d_Jf_tmp;
  double e_Jf_tmp;
  double f_Jf_tmp;
  double g_Jf_tmp;
  double h_Jf_tmp;
  double i_Jf_tmp;
  double j_Jf_tmp;
  double k_Jf_tmp;
  double l_Jf_tmp;
  double m_Jf_tmp;
  double n_Jf_tmp;
  double o_Jf_tmp;
  double p_Jf_tmp;
  double q_Jf_tmp;
  double r_Jf_tmp;
  double s_Jf_tmp;
  double t_Jf_tmp;
  double u_Jf_tmp;

  /*     This function was generated by the Symbolic Math Toolbox version 8.3. */
  /*     23-Feb-2020 01:43:45 */
  t2 = cos(in1[2]);
  t3 = cos(in1[5]);
  t4 = sin(in1[2]);
  t5 = sin(in1[5]);
  t6 = model[9] * model[14];
  t7 = model[9] * in1[6];
  t8 = model[1] * 2.0;
  t9 = model[2] * 2.0;
  t10 = model[5] * 2.0;
  t11 = model[7] * 2.0;
  t12 = model[0] * model[0];
  t13 = model[3] * model[3];
  t14 = rt_powd_snf(model[3], 3.0);
  t15 = model[4] * model[4];
  t16 = rt_powd_snf(model[4], 3.0);
  t17 = model[9] * model[9];
  t18 = model[10] * model[10];
  t19 = in1[5] * 2.0;
  t20 = in1[5] * 3.0;
  t21 = in1[4] * in1[4];
  t22 = in1[6] * in1[6];
  t32 = 1.0 / model[13];
  t24 = cos(t19);
  t25 = cos(t20);
  t27 = sin(t19);
  t28 = sin(t20);
  t29 = t5 * t5;
  t33 = t21 * 3.0;
  t41 = model[5] + -model[7];
  t42_tmp = model[0] * t13;
  t19 = t42_tmp * 2.0;
  t43_tmp = model[0] * t15;
  t20 = t43_tmp * 2.0;
  t44 = t8 * t17;
  t46_tmp = model[0] * model[2];
  t46 = t46_tmp * t15 * 4.0;
  t54_tmp = t7 * t7;
  t66_tmp = model[0] * model[1];
  t66 = t66_tmp * t15 * t17 * 4.0;
  t57 = t13 + -t15;
  t60 = t10 + t20;
  t61 = t11 + t19;
  t62 = t9 + t44;
  t69 = t33 + t22 * 4.0;
  t74_tmp = t7 - in1[3];
  t74 = tanh(-t32 * t74_tmp);
  t76_tmp_tmp = model[3] * model[4];
  t76_tmp = t76_tmp_tmp * t12 * t17;
  t81 = model[12] * t74;
  t89 = (t41 + t20) + -t19;
  t91_tmp = t12 * t17;
  t32 = model[12] * t32 * (t74 * t74 - 1.0);
  t107_tmp = model[0] * model[4];
  t107 = t107_tmp * in1[3] * 2.0 + t3 * t89 * in1[6] * 2.0;
  t114_tmp_tmp = model[14] * t74_tmp;
  t114_tmp = t81 - t114_tmp_tmp;
  b_t114_tmp = model[4] * model[9];
  t114 = model[3] * model[11] * (t9 + t17 * (model[0] + t8)) + b_t114_tmp * t114_tmp *
    2.0;
  t117_tmp = model[0] * model[3];
  b_t117_tmp = t117_tmp * model[4];
  c_t117_tmp = model[1] * t17;
  d_t117_tmp = b_t117_tmp * t17;
  t117 = 1.0 / ((((model[2] * t18 + c_t117_tmp * t18) + d_t117_tmp * t27 * 2.0) +
                 t17 * (t3 * t3) * t61) + t17 * t29 * t60);
  t120_tmp = t46_tmp * t13;
  b_t120_tmp = t66_tmp * t13 * t17;
  t120 = 1.0 / ((((((((((model[2] * model[6] * 4.0 + t120_tmp * 4.0) + t46) + model[0]
                       * model[6] * t17 * 2.0) + model[1] * model[6] * t17 * 4.0) +
                     t12 * t13 * t17) + t12 * t15 * t17) + b_t120_tmp * 4.0) +
                  t66) + t76_tmp * t27 * 2.0) + t91_tmp * t24 * t57);
  t94 = model[9] * t32;
  t118_tmp = model[3] * model[6];
  b_t118_tmp = model[0] * t14;
  c_t118_tmp = t117_tmp * t15;
  t118 = (((t118_tmp * (t21 + t22) * 4.0 + -(model[3] * t21 * t41)) + b_t118_tmp *
           t69) + c_t118_tmp * t69) + -(model[4] * t114_tmp * 8.0);
  t121 = t120 * t120;
  t19 = model[14] + -t32;
  t20 = t117_tmp * t3;
  t115_tmp = b_t117_tmp * t24;
  t115 = (t20 * in1[3] * 2.0 + t115_tmp * in1[6] * 4.0) + t5 * t107;
  t109 = t6 + -t94;
  t74 = model[14] * 8.0 + -(t32 * 8.0);
  t112 = t6 * 8.0 + -(t94 * 8.0);
  memset(&Jf[0], 0, 14U * sizeof(double));
  Jf[14] = -t4 * in1[3];
  Jf[15] = t2 * in1[3];
  Jf[16] = 0.0;
  Jf[17] = 0.0;
  Jf[18] = 0.0;
  Jf[19] = 0.0;
  Jf[20] = 0.0;
  Jf[21] = t2;
  Jf[22] = t4;
  Jf[23] = 0.0;
  Jf_tmp_tmp = t107_tmp * model[9];
  b_Jf_tmp_tmp = Jf_tmp_tmp * t3;
  Jf_tmp = b_Jf_tmp_tmp * t19;
  c_Jf_tmp_tmp = t117_tmp * model[9];
  d_Jf_tmp_tmp = c_Jf_tmp_tmp * t5;
  b_Jf_tmp = d_Jf_tmp_tmp * t19;
  c_Jf_tmp = model[9] * t120;
  Jf[24] = c_Jf_tmp * ((((model[6] * t19 * 8.0 + t42_tmp * t19 * 8.0) + t43_tmp *
    t19 * 8.0) + Jf_tmp * 8.0) - b_Jf_tmp * 8.0) * -0.5;
  d_Jf_tmp = -t17 * t117 * in1[4];
  Jf[25] = d_Jf_tmp * (t20 * 2.0 + t107_tmp * t5 * 2.0);
  Jf[26] = 0.0;
  e_Jf_tmp = model[0] * t17;
  Jf[27] = t120 * ((((model[2] * t74 + e_Jf_tmp * (model[14] * 4.0 - t32 * 4.0)) +
                     c_t117_tmp * t74) + Jf_tmp * 4.0) - b_Jf_tmp * 4.0);
  Jf[28] = 0.0;
  Jf[29] = 0.0;
  Jf[30] = 1.0;
  Jf_tmp = model[9] * t12;
  b_Jf_tmp = t107_tmp * model[7] * model[9];
  f_Jf_tmp = Jf_tmp * t16;
  Jf_tmp *= t14;
  g_Jf_tmp = t107_tmp * model[6];
  h_Jf_tmp = b_t114_tmp * t12 * t13;
  i_Jf_tmp = model[3] * model[9] * t12 * t15;
  j_Jf_tmp = g_Jf_tmp * model[9];
  k_Jf_tmp = t117_tmp * model[7] * model[9];
  l_Jf_tmp = model[0] * model[9] * t3;
  m_Jf_tmp = model[9] * t5 * t12 * t16;
  n_Jf_tmp = Jf_tmp_tmp * t5;
  o_Jf_tmp = b_t114_tmp * t5 * t12 * t13;
  p_Jf_tmp = h_Jf_tmp * t28;
  q_Jf_tmp = j_Jf_tmp * t5;
  Jf[31] = c_Jf_tmp * (((((((((((((l_Jf_tmp * (((t118_tmp * in1[4] * 8.0 +
    b_t118_tmp * in1[4] * 6.0) - model[3] * t41 * in1[4] * 2.0) + c_t118_tmp *
    in1[4] * 6.0) + m_Jf_tmp * in1[4] * 6.0) + Jf_tmp * t25 * in1[4] * 2.0) -
    f_Jf_tmp * t28 * in1[4] * 2.0) + q_Jf_tmp * in1[4] * 8.0) - b_Jf_tmp * t5 *
    in1[4] * 2.0) - k_Jf_tmp * t25 * in1[4] * 2.0) - b_Jf_tmp * t28 * in1[4] *
    2.0) + n_Jf_tmp * t10 * in1[4]) + c_Jf_tmp_tmp * t10 * t25 * in1[4]) +
    Jf_tmp_tmp * t10 * t28 * in1[4]) + o_Jf_tmp * in1[4] * 6.0) - i_Jf_tmp * t25
                        * in1[4] * 6.0) + p_Jf_tmp * in1[4] * 6.0) / 2.0;
  Jf[32] = -t17 * t115 * t117;
  Jf[33] = 0.0;
  t10 = model[2] * model[5];
  r_Jf_tmp = model[2] * model[7];
  s_Jf_tmp = model[0] * model[5] * t17;
  t_Jf_tmp = model[1] * model[5] * t17;
  u_Jf_tmp = model[1] * model[7] * t17;
  t14 = model[0] * t11 * t17;
  Jf[34] = t120 * ((((((((((t27 * t46 * in1[4] + t27 * t66 * in1[4]) - t10 * t27
    * in1[4] * 4.0) + r_Jf_tmp * t27 * in1[4] * 4.0) - t120_tmp * t27 * in1[4] *
    4.0) - s_Jf_tmp * t27 * in1[4] * 2.0) - t_Jf_tmp * t27 * in1[4] * 4.0) +
                      u_Jf_tmp * t27 * in1[4] * 4.0) + t14 * t27 * in1[4]) +
                    t115_tmp * t62 * in1[4] * 4.0) - b_t120_tmp * t27 * in1[4] *
                   4.0);
  Jf[35] = 0.0;
  Jf[36] = 0.0;
  Jf[37] = 0.0;
  t41 = model[9] * model[11] * t12;
  t22 = t107_tmp * model[5] * model[9];
  t2 = t41 * t13;
  t41 *= t15;
  Jf_tmp *= t21;
  f_Jf_tmp *= t21;
  i_Jf_tmp *= t21;
  t4 = t117_tmp * model[5] * model[9] * t21;
  t8 = t76_tmp_tmp * model[9];
  t18 = t8 * model[11] * t12;
  t69 = c_Jf_tmp_tmp * t3 * t114_tmp;
  t19 = t76_tmp * t24 * 4.0 - t91_tmp * t27 * t57 * 2.0;
  t20 = d_Jf_tmp_tmp * t114_tmp;
  t74 = t5 * t7 * t12 * t16;
  t32 = g_Jf_tmp * t5 * t7;
  t74_tmp = model[4] * t5 * t7 * t12 * t13;
  Jf[38] = c_Jf_tmp * ((((((((((((((((((((-model[0] * model[9] * t5 * t118 + t69 *
    8.0) + t2 * t24 * 4.0) - t41 * t24 * 4.0) + model[9] * t3 * t12 * t16 * t33) -
    f_Jf_tmp * t25 * 3.0) - Jf_tmp * t28 * 3.0) + t3 * t7 * t12 * t16 * in1[6] *
    4.0) + b_Jf_tmp_tmp * t21 * -model[7]) + g_Jf_tmp * t3 * t7 * in1[6] * 4.0) +
    b_t114_tmp * t3 * t12 * t13 * t33) + h_Jf_tmp * t21 * t25 * 9.0) + i_Jf_tmp *
    t28 * 9.0) + model[4] * t3 * t7 * t12 * t13 * in1[6] * 4.0) + t22 * t3 * t21)
    + j_Jf_tmp * t3 * t21 * 4.0) - t4 * t28 * 3.0) - b_Jf_tmp * t21 * t25 * 3.0)
    + t18 * t27 * 8.0) + t22 * t25 * t33) + k_Jf_tmp * t28 * t33) / 2.0 - model[9]
    * t121 * t19 * (((((((((((((((((((((((model[6] * t114_tmp * -8.0 - t42_tmp *
    t114_tmp * 8.0) - t43_tmp * t114_tmp * 8.0) + l_Jf_tmp * t118) + t20 * 8.0)
    + t2 * t27 * 2.0) - t41 * t27 * 2.0) + m_Jf_tmp * t33) + Jf_tmp * t25) -
    f_Jf_tmp * t28) + t74 * in1[6] * 4.0) + n_Jf_tmp * t21 * -model[7]) +
    c_Jf_tmp_tmp * t21 * t25 * -model[7]) + Jf_tmp_tmp * t21 * t28 * -model[7]) +
    t32 * in1[6] * 4.0) + o_Jf_tmp * t33) - i_Jf_tmp * t25 * 3.0) + p_Jf_tmp *
    t33) + t74_tmp * in1[6] * 4.0) + t22 * t5 * t21) + q_Jf_tmp * t21 * 4.0) +
                      t4 * t25) - t18 * t24 * 4.0) + t22 * t21 * t28) / 2.0;
  Jf_tmp_tmp = t3 * t5;
  Jf_tmp = Jf_tmp_tmp * t17;
  b_Jf_tmp = b_t117_tmp * t27;
  Jf[39] = t17 * t117 * in1[4] * (((-t3 * t107 + t29 * t89 * in1[6] * 2.0) +
    t117_tmp * t5 * in1[3] * 2.0) + b_Jf_tmp * in1[6] * 8.0) + t17 * t115 *
    (t117 * t117) * in1[4] * ((Jf_tmp * t60 * 2.0 - Jf_tmp * t61 * 2.0) +
    d_t117_tmp * t24 * 4.0);
  Jf[40] = 0.0;
  Jf_tmp = t21 * t24;
  f_Jf_tmp = t54_tmp * t12;
  g_Jf_tmp = t81 * 8.0 - t114_tmp_tmp * 8.0;
  h_Jf_tmp = t10 * t21;
  i_Jf_tmp = f_Jf_tmp * t13;
  j_Jf_tmp = t46_tmp * model[4] * model[11];
  k_Jf_tmp = model[4] * model[11];
  m_Jf_tmp = t120_tmp * t21;
  n_Jf_tmp = s_Jf_tmp * t21;
  o_Jf_tmp = t_Jf_tmp * t21;
  p_Jf_tmp = model[0] * t54_tmp - t21 * t62;
  q_Jf_tmp = b_t120_tmp * t21;
  t10 = t66_tmp * model[4] * model[11];
  Jf[41] = t120 * (((((((((((((((((model[0] * t5 * t114 * -2.0 + Jf_tmp * t46) +
    Jf_tmp * t66) - h_Jf_tmp * t24 * 4.0) + r_Jf_tmp * t21 * t24 * 4.0) +
    i_Jf_tmp * t24 * 2.0) - f_Jf_tmp * t15 * t24 * 2.0) - t69 * 4.0) + j_Jf_tmp *
    t3 * 4.0) + k_Jf_tmp * t3 * t12 * t17 * 2.0) - m_Jf_tmp * t24 * 4.0) -
    n_Jf_tmp * t24 * 2.0) - o_Jf_tmp * t24 * 4.0) + u_Jf_tmp * t21 * t24 * 4.0)
                      + t14 * t21 * t24) + b_Jf_tmp * p_Jf_tmp * 4.0) - q_Jf_tmp
                    * t24 * 4.0) + t10 * t3 * t17 * 4.0) - t121 * t19 *
    ((((((((((((((((((((model[2] * g_Jf_tmp + c_t117_tmp * g_Jf_tmp) + model[0] * t3
                       * t114 * 2.0) + e_Jf_tmp * (t81 * 4.0 - t114_tmp_tmp *
    4.0)) - h_Jf_tmp * t27 * 2.0) + model[7] * t9 * t21 * t27) + model[7] * t21 *
                   t27 * t44) + i_Jf_tmp * t27) - t20 * 4.0) + j_Jf_tmp * t5 *
                4.0) + k_Jf_tmp * t5 * t12 * t17 * 2.0) - m_Jf_tmp * t27 * 2.0)
             - n_Jf_tmp * t27) - o_Jf_tmp * t27 * 2.0) + model[0] * model[7] * t17 *
           t21 * t27) + model[0] * t9 * t15 * t21 * t27) + t43_tmp * t21 * t27 *
         t44) + t7 * t12 * t15 * t27 * -t7) - t115_tmp * p_Jf_tmp * 2.0) -
      q_Jf_tmp * t27 * 2.0) + t10 * t5 * t17 * 4.0);
  Jf[42] = 0.0;
  Jf[43] = 0.0;
  Jf[44] = 0.0;
  Jf_tmp = d_Jf_tmp_tmp * t109;
  Jf[45] = c_Jf_tmp * (((((((model[6] * t109 * 8.0 + t42_tmp * t109 * 8.0) +
    t43_tmp * t109 * 8.0) + t74 * 8.0) + l_Jf_tmp * (((model[4] * t109 * 8.0 +
    t118_tmp * in1[6] * 8.0) + b_t118_tmp * in1[6] * 8.0) + c_t118_tmp * in1[6] *
    8.0)) + t32 * 8.0) - Jf_tmp * 8.0) + t74_tmp * 8.0) / 2.0;
  Jf[46] = d_Jf_tmp * (Jf_tmp_tmp * t89 * 2.0 + t115_tmp * 4.0);
  Jf[47] = 1.0;
  b_Jf_tmp = model[9] * t7 * t12;
  Jf[48] = -t120 * (((((((model[2] * t112 + e_Jf_tmp * (t6 * 4.0 - t94 * 4.0)) +
    c_t117_tmp * t112) + b_Jf_tmp_tmp * t109 * 4.0) - Jf_tmp * 4.0) - b_Jf_tmp *
                      t13 * t27 * 2.0) + b_Jf_tmp * t15 * t27 * 2.0) + t8 * t7 *
                    t12 * t24 * 4.0);
}


inline void dynamicsGradientsRaw(const double x[STATE_LENGTH], double Df[STATE_LENGTH*STATE_LENGTH], double Dg[STATE_LENGTH*INPUT_LENGTH*STATE_LENGTH])
{
  double t2;
  double t3;
  double t4;
  double t7;
  double t8;
  double t9;
  double t10;
  double t11;
  double t12;
  double t15;
  double t16;
  double t17;
  double t18;
  double t20;
  double t21;
  double t22;
  double t27;
  double t32_tmp;
  double t32;
  double t47;
  double t48;
  double t23;
  double t30;
  double t31;
  double t43;
  double t53_tmp;
  double t94;
  double t54_tmp;
  double t54;
  double t45;
  double t46;
  double t81;
  double t82_tmp;
  double t82;
  double dv0[98];
  JfFun(x, Df);

  /* JGFUN */
  /*     JG = JGFUN(IN1,IN2) */
  /*     This function was generated by the Symbolic Math Toolbox version 8.3. */
  /*     23-Feb-2020 01:43:45 */
  t2 = cos(x[5]);
  t3 = sin(x[5]);
  t4 = model[2] * 2.0;
  t7 = model[0] * model[0];
  t8 = model[3] * model[3];
  t9 = model[4] * model[4];
  t10 = model[9] * model[9];
  t11 = model[10] * model[10];
  t12 = x[5] * 2.0;
  t15 = cos(t12);
  t16 = t2 * t2;
  t17 = sin(t12);
  t18 = t3 * t3;
  t20 = model[0] * t8;
  t21 = model[0] * t9;
  t22 = model[0] * t10;
  t12 = model[0] * model[4] * model[9];
  t27 = t12 * t2;
  t32_tmp = model[1] * t10;
  t32 = t32_tmp * 2.0;
  t47 = t7 * t8 * t10;
  t48 = t7 * t9 * t10;
  t23 = model[6] * t22;
  t30 = t20 * 2.0;
  t31 = t21 * 2.0;
  t43 = t8 + -t9;
  t53_tmp = model[3] * model[4] * t7 * t10;
  t8 = t53_tmp * t17;
  t94 = model[0] * model[3];
  t54_tmp = t94 * model[9];
  t54 = t54_tmp * t2 + t12 * t3;
  t45 = model[5] * 2.0 + t31;
  t46 = model[7] * 2.0 + t30;
  t81 = t94 * model[4] * model[9];
  t94 = (((model[1] * model[9] * t11 + model[2] * t11 * (1.0 / model[9])) + t81 *
          t17 * 2.0) + model[9] * t16 * t46) + model[9] * t18 * t45;
  t82_tmp = t7 * t10;
  t82 = 1.0 / ((((((((((model[2] * model[6] * 4.0 + model[2] * t20 * 4.0) +
                       model[2] * t21 * 4.0) + t23 * 2.0) + model[1] * model[6] *
                     t10 * 4.0) + t47) + t48) + t32_tmp * t20 * 4.0) + t32_tmp *
                 t21 * 4.0) + t8 * 2.0) + t82_tmp * t15 * t43);
  t10 = 1.0 / (((((((((((model[6] * t4 + t23) + t4 * t20) + t4 * t21) + model[6]
                      * t32) + t47) + t48) + t32_tmp * t30) + t32_tmp * t31) +
                 t8) + t82_tmp * t16 * -t9) + -(t18 * t47));
  t12 = model[9] * t2 * t3;
  t81 = model[8] * model[10] * ((t81 * t15 * 4.0 + t12 * t45 * 2.0) + -(t12 *
    t46 * 2.0)) * (1.0 / (t94 * t94));
  t7 = -(t54_tmp * t3);
  t11 = t53_tmp * t15;
  t94 = model[8] * t54 * t82 * 2.0 + model[8] * ((((t4 + t22) + t27) + t32) + t7)
    * (t82 * t82) * (t11 * 4.0 - t82_tmp * t17 * t43 * 2.0) * 2.0;
  t8 = model[8] * model[9];
  t12 = t2 * t3;
  t12 = -(t8 * t54 * t10) + -(t8 * ((((model[6] + t20) + t21) + t27) + t7) *
    ((t11 * 2.0 + t12 * t48 * 2.0) + -(t12 * t47 * 2.0)) * (t10 * t10));
  memset(&dv0[0], 0, 73U * sizeof(double));
  dv0[73] = t12;
  dv0[74] = t81;
  dv0[75] = 0.0;
  dv0[76] = t94;
  dv0[77] = 0.0;
  dv0[78] = 0.0;
  dv0[79] = 0.0;
  dv0[80] = t12;
  dv0[81] = -t81;
  dv0[82] = 0.0;
  dv0[83] = t94;
  memset(&dv0[84], 0, 14U * sizeof(double));
  memcpy(&Dg[0], &dv0[0], 98U * sizeof(double));
}

inline void dynamicsGradient(const double t,
                      const double X[STATE_LENGTH],
                      const double U[INPUT_LENGTH],
                      double Df[STATE_LENGTH*STATE_LENGTH])
{

   double Dg[STATE_LENGTH*INPUT_LENGTH*STATE_LENGTH];
   dynamicsGradientsRaw(X, Df, Dg);
   for(uint32_t i=0; i<STATE_LENGTH; i++)
   {
      for(uint32_t j=0; j<STATE_LENGTH; j++)
      {
         double tmp = Df[i];
         for(uint32_t k=0; k<INPUT_LENGTH; k++)
         {
            tmp+=Dg[i+k*(STATE_LENGTH)+j*STATE_LENGTH*INPUT_LENGTH]*U[k];
         }
         Df[i+j*STATE_LENGTH] = tmp;
      }
   }
}

inline void measurementH(const double X[STATE_LENGTH],
                  double h[MEASUREMENT_LENGTH])
{
   h[0] = X[3]; //velocity
   h[1] = X[4]; //thetaDot
   h[2] = X[5]; //psi
   h[3] = X[6]; //psiDot
}

inline void measurementHGradient(const double X[STATE_LENGTH],
                          double Dh[MEASUREMENT_LENGTH*STATE_LENGTH])
{
   //with respect to x[0]
   Dh[0] = 0.;
   Dh[1] = 0.;
   Dh[2] = 0.;
   Dh[3] = 0.;

   //with respect to x[1]
   Dh[0+4] = 0.;
   Dh[1+4] = 0.;
   Dh[2+4] = 0.;
   Dh[3+4] = 0.;

   //with respect to x[2]
   Dh[0+4*2] = 0.;
   Dh[1+4*2] = 0.;
   Dh[2+4*2] = 0.;
   Dh[3+4*2] = 0.;

   //with respect to x[3]
   Dh[0+4*3] = 1.;
   Dh[1+4*3] = 0.;
   Dh[2+4*3] = 0.;
   Dh[3+4*3] = 0.;

   //with respect to x[4]
   Dh[0+4*4] = 0.;
   Dh[1+4*4] = 1.;
   Dh[2+4*4] = 0.;
   Dh[3+4*4] = 0.;

   //with respect to x[5]
   Dh[0+4*5] = 0.;
   Dh[1+4*5] = 0.;
   Dh[2+4*5] = 1.;
   Dh[3+4*5] = 0.;

   //with respect to x[6]
   Dh[0+4*6] = 0.;
   Dh[1+4*6] = 0.;
   Dh[2+4*6] = 0.;
   Dh[3+4*6] = 1.;
}
#endif