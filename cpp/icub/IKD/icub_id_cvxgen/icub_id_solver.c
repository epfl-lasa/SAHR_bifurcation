/* Produced by CVXGEN, 2018-11-27 12:50:17 -0500.  */
/* CVXGEN is Copyright (C) 2006-2017 Jacob Mattingley, jem@cvxgen.com. */
/* The code in this file is Copyright (C) 2006-2017 Jacob Mattingley. */
/* CVXGEN, or solvers produced by CVXGEN, cannot be used for commercial */
/* applications without prior written permission from Jacob Mattingley. */

/* Filename: solver.c. */
/* Description: Main solver file. */
#include "icub_id_solver.h"
double icub_id_eval_gap(void) {
  int i;
  double gap;
  gap = 0;
  for (i = 0; i < 172; i++)
    gap += icub_id_work.z[i]*icub_id_work.s[i];
  return gap;
}
void icub_id_set_defaults(void) {
  icub_id_settings.resid_tol = 1e-6;
  icub_id_settings.eps = 1e-4;
  icub_id_settings.max_iters = 25;
  icub_id_settings.refine_steps = 1;
  icub_id_settings.s_init = 1;
  icub_id_settings.z_init = 1;
  icub_id_settings.debug = 0;
  icub_id_settings.verbose = 1;
  icub_id_settings.verbose_refinement = 0;
  icub_id_settings.better_start = 1;
  icub_id_settings.kkt_reg = 1e-7;
}
void icub_id_setup_pointers(void) {
  icub_id_work.y = icub_id_work.x + 124;
  icub_id_work.s = icub_id_work.x + 192;
  icub_id_work.z = icub_id_work.x + 364;
  icub_id_vars.F_1 = icub_id_work.x + 0;
  icub_id_vars.F_2 = icub_id_work.x + 3;
  icub_id_vars.F_3 = icub_id_work.x + 6;
  icub_id_vars.F_4 = icub_id_work.x + 9;
  icub_id_vars.T_1 = icub_id_work.x + 12;
  icub_id_vars.T_2 = icub_id_work.x + 15;
  icub_id_vars.T_3 = icub_id_work.x + 18;
  icub_id_vars.T_4 = icub_id_work.x + 21;
  icub_id_vars.ddq = icub_id_work.x + 24;
  icub_id_vars.slack = icub_id_work.x + 62;
  icub_id_vars.tau = icub_id_work.x + 92;
}
void setup_indexed_icub_id_params(void) {
  /* In CVXGEN, you can say */
  /*   parameters */
  /*     A[i] (5,3), i=1..4 */
  /*   end */
  /* This function sets up A[2] to be a pointer to A_2, which is a length-15 */
  /* vector of doubles. */
  /* If you access parameters that you haven't defined in CVXGEN, the result */
  /* is undefined. */
  icub_id_params.QlT[1] = icub_id_params.QlT_1;
  icub_id_params.QlT[2] = icub_id_params.QlT_2;
  icub_id_params.QlT[3] = icub_id_params.QlT_3;
  icub_id_params.QlT[4] = icub_id_params.QlT_4;
  icub_id_params.QlR[1] = icub_id_params.QlR_1;
  icub_id_params.QlR[2] = icub_id_params.QlR_2;
  icub_id_params.QlR[3] = icub_id_params.QlR_3;
  icub_id_params.QlR[4] = icub_id_params.QlR_4;
  icub_id_params.I[1] = icub_id_params.I_1;
  icub_id_params.J[7] = icub_id_params.J_7;
  icub_id_params.ifp[1] = icub_id_params.ifp_1;
  icub_id_params.J[13] = icub_id_params.J_13;
  icub_id_params.ifp[2] = icub_id_params.ifp_2;
  icub_id_params.J[19] = icub_id_params.J_19;
  icub_id_params.ifp[3] = icub_id_params.ifp_3;
  icub_id_params.J[25] = icub_id_params.J_25;
  icub_id_params.ifp[4] = icub_id_params.ifp_4;
  icub_id_params.J[8] = icub_id_params.J_8;
  icub_id_params.J[14] = icub_id_params.J_14;
  icub_id_params.J[20] = icub_id_params.J_20;
  icub_id_params.J[26] = icub_id_params.J_26;
  icub_id_params.J[9] = icub_id_params.J_9;
  icub_id_params.J[15] = icub_id_params.J_15;
  icub_id_params.J[21] = icub_id_params.J_21;
  icub_id_params.J[27] = icub_id_params.J_27;
  icub_id_params.I[2] = icub_id_params.I_2;
  icub_id_params.I[3] = icub_id_params.I_3;
  icub_id_params.I[4] = icub_id_params.I_4;
  icub_id_params.J[10] = icub_id_params.J_10;
  icub_id_params.ifo[1] = icub_id_params.ifo_1;
  icub_id_params.J[16] = icub_id_params.J_16;
  icub_id_params.ifo[2] = icub_id_params.ifo_2;
  icub_id_params.J[22] = icub_id_params.J_22;
  icub_id_params.ifo[3] = icub_id_params.ifo_3;
  icub_id_params.J[28] = icub_id_params.J_28;
  icub_id_params.ifo[4] = icub_id_params.ifo_4;
  icub_id_params.J[11] = icub_id_params.J_11;
  icub_id_params.J[17] = icub_id_params.J_17;
  icub_id_params.J[23] = icub_id_params.J_23;
  icub_id_params.J[29] = icub_id_params.J_29;
  icub_id_params.J[12] = icub_id_params.J_12;
  icub_id_params.J[18] = icub_id_params.J_18;
  icub_id_params.J[24] = icub_id_params.J_24;
  icub_id_params.J[30] = icub_id_params.J_30;
  icub_id_params.I[5] = icub_id_params.I_5;
  icub_id_params.I[6] = icub_id_params.I_6;
  icub_id_params.I[7] = icub_id_params.I_7;
  icub_id_params.I[8] = icub_id_params.I_8;
  icub_id_params.I[9] = icub_id_params.I_9;
  icub_id_params.I[10] = icub_id_params.I_10;
  icub_id_params.I[11] = icub_id_params.I_11;
  icub_id_params.I[12] = icub_id_params.I_12;
  icub_id_params.I[13] = icub_id_params.I_13;
  icub_id_params.I[14] = icub_id_params.I_14;
  icub_id_params.I[15] = icub_id_params.I_15;
  icub_id_params.I[16] = icub_id_params.I_16;
  icub_id_params.I[17] = icub_id_params.I_17;
  icub_id_params.I[18] = icub_id_params.I_18;
  icub_id_params.I[19] = icub_id_params.I_19;
  icub_id_params.I[20] = icub_id_params.I_20;
  icub_id_params.I[21] = icub_id_params.I_21;
  icub_id_params.I[22] = icub_id_params.I_22;
  icub_id_params.I[23] = icub_id_params.I_23;
  icub_id_params.I[24] = icub_id_params.I_24;
  icub_id_params.I[25] = icub_id_params.I_25;
  icub_id_params.I[26] = icub_id_params.I_26;
  icub_id_params.I[27] = icub_id_params.I_27;
  icub_id_params.I[28] = icub_id_params.I_28;
  icub_id_params.I[29] = icub_id_params.I_29;
  icub_id_params.I[30] = icub_id_params.I_30;
  icub_id_params.I[31] = icub_id_params.I_31;
  icub_id_params.I[32] = icub_id_params.I_32;
  icub_id_params.I[33] = icub_id_params.I_33;
  icub_id_params.I[34] = icub_id_params.I_34;
  icub_id_params.I[35] = icub_id_params.I_35;
  icub_id_params.I[36] = icub_id_params.I_36;
  icub_id_params.I[37] = icub_id_params.I_37;
  icub_id_params.I[38] = icub_id_params.I_38;
  icub_id_params.Jtask[1] = icub_id_params.Jtask_1;
  icub_id_params.ifTask[1] = icub_id_params.ifTask_1;
  icub_id_params.Jtask[2] = icub_id_params.Jtask_2;
  icub_id_params.Jtask[3] = icub_id_params.Jtask_3;
  icub_id_params.Jtask[4] = icub_id_params.Jtask_4;
  icub_id_params.ifTask[2] = icub_id_params.ifTask_2;
  icub_id_params.Jtask[5] = icub_id_params.Jtask_5;
  icub_id_params.Jtask[6] = icub_id_params.Jtask_6;
  icub_id_params.Jtask[7] = icub_id_params.Jtask_7;
  icub_id_params.ifTask[3] = icub_id_params.ifTask_3;
  icub_id_params.Jtask[8] = icub_id_params.Jtask_8;
  icub_id_params.Jtask[9] = icub_id_params.Jtask_9;
  icub_id_params.Jtask[10] = icub_id_params.Jtask_10;
  icub_id_params.ifTask[4] = icub_id_params.ifTask_4;
  icub_id_params.Jtask[11] = icub_id_params.Jtask_11;
  icub_id_params.Jtask[12] = icub_id_params.Jtask_12;
  icub_id_params.Jtask[13] = icub_id_params.Jtask_13;
  icub_id_params.ifTask[5] = icub_id_params.ifTask_5;
  icub_id_params.Jtask[14] = icub_id_params.Jtask_14;
  icub_id_params.Jtask[15] = icub_id_params.Jtask_15;
  icub_id_params.Jtask[16] = icub_id_params.Jtask_16;
  icub_id_params.ifTask[6] = icub_id_params.ifTask_6;
  icub_id_params.Jtask[17] = icub_id_params.Jtask_17;
  icub_id_params.Jtask[18] = icub_id_params.Jtask_18;
  icub_id_params.Jtask[19] = icub_id_params.Jtask_19;
  icub_id_params.ifTask[7] = icub_id_params.ifTask_7;
  icub_id_params.Jtask[20] = icub_id_params.Jtask_20;
  icub_id_params.Jtask[21] = icub_id_params.Jtask_21;
  icub_id_params.Jtask[22] = icub_id_params.Jtask_22;
  icub_id_params.ifTask[8] = icub_id_params.ifTask_8;
  icub_id_params.Jtask[23] = icub_id_params.Jtask_23;
  icub_id_params.Jtask[24] = icub_id_params.Jtask_24;
  icub_id_params.Jtask[25] = icub_id_params.Jtask_25;
  icub_id_params.ifTask[9] = icub_id_params.ifTask_9;
  icub_id_params.Jtask[26] = icub_id_params.Jtask_26;
  icub_id_params.Jtask[27] = icub_id_params.Jtask_27;
  icub_id_params.Jtask[28] = icub_id_params.Jtask_28;
  icub_id_params.ifTask[10] = icub_id_params.ifTask_10;
  icub_id_params.Jtask[29] = icub_id_params.Jtask_29;
  icub_id_params.Jtask[30] = icub_id_params.Jtask_30;
  icub_id_params.uT_min[1] = icub_id_params.uT_min_1;
  icub_id_params.nz[1] = icub_id_params.nz_1;
  icub_id_params.nx[1] = icub_id_params.nx_1;
  icub_id_params.uT_min[2] = icub_id_params.uT_min_2;
  icub_id_params.nz[2] = icub_id_params.nz_2;
  icub_id_params.nx[2] = icub_id_params.nx_2;
  icub_id_params.uT_min[3] = icub_id_params.uT_min_3;
  icub_id_params.nz[3] = icub_id_params.nz_3;
  icub_id_params.nx[3] = icub_id_params.nx_3;
  icub_id_params.uT_min[4] = icub_id_params.uT_min_4;
  icub_id_params.nz[4] = icub_id_params.nz_4;
  icub_id_params.nx[4] = icub_id_params.nx_4;
  icub_id_params.uT_max[1] = icub_id_params.uT_max_1;
  icub_id_params.uT_max[2] = icub_id_params.uT_max_2;
  icub_id_params.uT_max[3] = icub_id_params.uT_max_3;
  icub_id_params.uT_max[4] = icub_id_params.uT_max_4;
  icub_id_params.ny[1] = icub_id_params.ny_1;
  icub_id_params.ny[2] = icub_id_params.ny_2;
  icub_id_params.ny[3] = icub_id_params.ny_3;
  icub_id_params.ny[4] = icub_id_params.ny_4;
  icub_id_params.uR_min[1] = icub_id_params.uR_min_1;
  icub_id_params.uR_min[2] = icub_id_params.uR_min_2;
  icub_id_params.uR_min[3] = icub_id_params.uR_min_3;
  icub_id_params.uR_min[4] = icub_id_params.uR_min_4;
  icub_id_params.uR_max[1] = icub_id_params.uR_max_1;
  icub_id_params.uR_max[2] = icub_id_params.uR_max_2;
  icub_id_params.uR_max[3] = icub_id_params.uR_max_3;
  icub_id_params.uR_max[4] = icub_id_params.uR_max_4;
}
void setup_indexed_opticub_id_vars(void) {
  /* In CVXGEN, you can say */
  /*   variables */
  /*     x[i] (5), i=2..4 */
  /*   end */
  /* This function sets up x[3] to be a pointer to x_3, which is a length-5 */
  /* vector of doubles. */
  /* If you access variables that you haven't defined in CVXGEN, the result */
  /* is undefined. */
  icub_id_vars.F[1] = icub_id_vars.F_1;
  icub_id_vars.F[2] = icub_id_vars.F_2;
  icub_id_vars.F[3] = icub_id_vars.F_3;
  icub_id_vars.F[4] = icub_id_vars.F_4;
  icub_id_vars.T[1] = icub_id_vars.T_1;
  icub_id_vars.T[2] = icub_id_vars.T_2;
  icub_id_vars.T[3] = icub_id_vars.T_3;
  icub_id_vars.T[4] = icub_id_vars.T_4;
}
void icub_id_setup_indexing(void) {
  icub_id_setup_pointers();
  setup_indexed_icub_id_params();
  setup_indexed_opticub_id_vars();
}
void icub_id_set_start(void) {
  int i;
  for (i = 0; i < 124; i++)
    icub_id_work.x[i] = 0;
  for (i = 0; i < 68; i++)
    icub_id_work.y[i] = 0;
  for (i = 0; i < 172; i++)
    icub_id_work.s[i] = (icub_id_work.h[i] > 0) ? icub_id_work.h[i] : icub_id_settings.s_init;
  for (i = 0; i < 172; i++)
    icub_id_work.z[i] = icub_id_settings.z_init;
}
double icub_id_eval_objv(void) {
  int i;
  double objv;
  /* Borrow space in icub_id_work.rhs. */
  icub_id_multbyP(icub_id_work.rhs, icub_id_work.x);
  objv = 0;
  for (i = 0; i < 124; i++)
    objv += icub_id_work.x[i]*icub_id_work.rhs[i];
  objv *= 0.5;
  for (i = 0; i < 124; i++)
    objv += icub_id_work.q[i]*icub_id_work.x[i];
  objv += icub_id_work.quad_714092859392[0];
  return objv;
}
void icub_id_fillrhs_aff(void) {
  int i;
  double *r1, *r2, *r3, *r4;
  r1 = icub_id_work.rhs;
  r2 = icub_id_work.rhs + 124;
  r3 = icub_id_work.rhs + 296;
  r4 = icub_id_work.rhs + 468;
  /* r1 = -A^Ty - G^Tz - Px - q. */
  icub_id_multbymAT(r1, icub_id_work.y);
  icub_id_multbymGT(icub_id_work.buffer, icub_id_work.z);
  for (i = 0; i < 124; i++)
    r1[i] += icub_id_work.buffer[i];
  icub_id_multbyP(icub_id_work.buffer, icub_id_work.x);
  for (i = 0; i < 124; i++)
    r1[i] -= icub_id_work.buffer[i] + icub_id_work.q[i];
  /* r2 = -z. */
  for (i = 0; i < 172; i++)
    r2[i] = -icub_id_work.z[i];
  /* r3 = -Gx - s + h. */
  icub_id_multbymG(r3, icub_id_work.x);
  for (i = 0; i < 172; i++)
    r3[i] += -icub_id_work.s[i] + icub_id_work.h[i];
  /* r4 = -Ax + b. */
  icub_id_multbymA(r4, icub_id_work.x);
  for (i = 0; i < 68; i++)
    r4[i] += icub_id_work.b[i];
}
void icub_id_fillrhs_cc(void) {
  int i;
  double *r2;
  double *ds_aff, *dz_aff;
  double mu;
  double alpha;
  double sigma;
  double smu;
  double minval;
  r2 = icub_id_work.rhs + 124;
  ds_aff = icub_id_work.lhs_aff + 124;
  dz_aff = icub_id_work.lhs_aff + 296;
  mu = 0;
  for (i = 0; i < 172; i++)
    mu += icub_id_work.s[i]*icub_id_work.z[i];
  /* Don't finish calculating mu quite yet. */
  /* Find min(min(ds./s), min(dz./z)). */
  minval = 0;
  for (i = 0; i < 172; i++)
    if (ds_aff[i] < minval*icub_id_work.s[i])
      minval = ds_aff[i]/icub_id_work.s[i];
  for (i = 0; i < 172; i++)
    if (dz_aff[i] < minval*icub_id_work.z[i])
      minval = dz_aff[i]/icub_id_work.z[i];
  /* Find alpha. */
  if (-1 < minval)
      alpha = 1;
  else
      alpha = -1/minval;
  sigma = 0;
  for (i = 0; i < 172; i++)
    sigma += (icub_id_work.s[i] + alpha*ds_aff[i])*
      (icub_id_work.z[i] + alpha*dz_aff[i]);
  sigma /= mu;
  sigma = sigma*sigma*sigma;
  /* Finish calculating mu now. */
  mu *= 0.005813953488372093;
  smu = sigma*mu;
  /* Fill-in the rhs. */
  for (i = 0; i < 124; i++)
    icub_id_work.rhs[i] = 0;
  for (i = 296; i < 536; i++)
    icub_id_work.rhs[i] = 0;
  for (i = 0; i < 172; i++)
    r2[i] = icub_id_work.s_inv[i]*(smu - ds_aff[i]*dz_aff[i]);
}
void icub_id_refine(double *target, double *var) {
  int i, j;
  double *residual = icub_id_work.buffer;
  double norm2;
  double *new_var = icub_id_work.buffer2;
  for (j = 0; j < icub_id_settings.refine_steps; j++) {
    norm2 = 0;
    icub_id_matrix_multiply(residual, var);
    for (i = 0; i < 536; i++) {
      residual[i] = residual[i] - target[i];
      norm2 += residual[i]*residual[i];
    }
#ifndef ZERO_LIBRARY_MODE
    if (icub_id_settings.verbose_refinement) {
      if (j == 0)
        printf("Initial residual before refinement has norm squared %.6g.\n", norm2);
      else
        printf("After refinement we get squared norm %.6g.\n", norm2);
    }
#endif
    /* Solve to find new_var = KKT \ (target - A*var). */
    icub_id_ldl_icub_id_solve(residual, new_var);
    /* Update var += new_var, or var += KKT \ (target - A*var). */
    for (i = 0; i < 536; i++) {
      var[i] -= new_var[i];
    }
  }
#ifndef ZERO_LIBRARY_MODE
  if (icub_id_settings.verbose_refinement) {
    /* Check the residual once more, but only if we're reporting it, since */
    /* it's expensive. */
    norm2 = 0;
    icub_id_matrix_multiply(residual, var);
    for (i = 0; i < 536; i++) {
      residual[i] = residual[i] - target[i];
      norm2 += residual[i]*residual[i];
    }
    if (j == 0)
      printf("Initial residual before refinement has norm squared %.6g.\n", norm2);
    else
      printf("After refinement we get squared norm %.6g.\n", norm2);
  }
#endif
}
double icub_id_calc_ineq_resid_squared(void) {
  /* Calculates the norm ||-Gx - s + h||. */
  double norm2_squared;
  int i;
  /* Find -Gx. */
  icub_id_multbymG(icub_id_work.buffer, icub_id_work.x);
  /* Add -s + h. */
  for (i = 0; i < 172; i++)
    icub_id_work.buffer[i] += -icub_id_work.s[i] + icub_id_work.h[i];
  /* Now find the squared norm. */
  norm2_squared = 0;
  for (i = 0; i < 172; i++)
    norm2_squared += icub_id_work.buffer[i]*icub_id_work.buffer[i];
  return norm2_squared;
}
double icub_id_calc_eq_resid_squared(void) {
  /* Calculates the norm ||-Ax + b||. */
  double norm2_squared;
  int i;
  /* Find -Ax. */
  icub_id_multbymA(icub_id_work.buffer, icub_id_work.x);
  /* Add +b. */
  for (i = 0; i < 68; i++)
    icub_id_work.buffer[i] += icub_id_work.b[i];
  /* Now find the squared norm. */
  norm2_squared = 0;
  for (i = 0; i < 68; i++)
    norm2_squared += icub_id_work.buffer[i]*icub_id_work.buffer[i];
  return norm2_squared;
}
void icub_id_better_start(void) {
  /* Calculates a better starting point, using a similar approach to CVXOPT. */
  /* Not yet speed optimized. */
  int i;
  double *x, *s, *z, *y;
  double alpha;
  icub_id_work.block_33[0] = -1;
  /* Make sure sinvz is 1 to make hijacked KKT system ok. */
  for (i = 0; i < 172; i++)
    icub_id_work.s_inv_z[i] = 1;
  icub_id_fill_KKT();
  icub_id_ldl_factor();
  icub_id_fillrhs_start();
  /* Borrow icub_id_work.lhs_aff for the solution. */
  icub_id_ldl_icub_id_solve(icub_id_work.rhs, icub_id_work.lhs_aff);
  /* Don't do any refinement for now. Precision doesn't matter too much. */
  x = icub_id_work.lhs_aff;
  s = icub_id_work.lhs_aff + 124;
  z = icub_id_work.lhs_aff + 296;
  y = icub_id_work.lhs_aff + 468;
  /* Just set x and y as is. */
  for (i = 0; i < 124; i++)
    icub_id_work.x[i] = x[i];
  for (i = 0; i < 68; i++)
    icub_id_work.y[i] = y[i];
  /* Now complete the initialization. Start with s. */
  /* Must have alpha > max(z). */
  alpha = -1e99;
  for (i = 0; i < 172; i++)
    if (alpha < z[i])
      alpha = z[i];
  if (alpha < 0) {
    for (i = 0; i < 172; i++)
      icub_id_work.s[i] = -z[i];
  } else {
    alpha += 1;
    for (i = 0; i < 172; i++)
      icub_id_work.s[i] = -z[i] + alpha;
  }
  /* Now initialize z. */
  /* Now must have alpha > max(-z). */
  alpha = -1e99;
  for (i = 0; i < 172; i++)
    if (alpha < -z[i])
      alpha = -z[i];
  if (alpha < 0) {
    for (i = 0; i < 172; i++)
      icub_id_work.z[i] = z[i];
  } else {
    alpha += 1;
    for (i = 0; i < 172; i++)
      icub_id_work.z[i] = z[i] + alpha;
  }
}
void icub_id_fillrhs_start(void) {
  /* Fill rhs with (-q, 0, h, b). */
  int i;
  double *r1, *r2, *r3, *r4;
  r1 = icub_id_work.rhs;
  r2 = icub_id_work.rhs + 124;
  r3 = icub_id_work.rhs + 296;
  r4 = icub_id_work.rhs + 468;
  for (i = 0; i < 124; i++)
    r1[i] = -icub_id_work.q[i];
  for (i = 0; i < 172; i++)
    r2[i] = 0;
  for (i = 0; i < 172; i++)
    r3[i] = icub_id_work.h[i];
  for (i = 0; i < 68; i++)
    r4[i] = icub_id_work.b[i];
}
long icub_id_solve(void) {
  int i;
  int iter;
  double *dx, *ds, *dy, *dz;
  double minval;
  double alpha;
  icub_id_work.converged = 0;
  icub_id_setup_pointers();
  icub_id_pre_ops();
#ifndef ZERO_LIBRARY_MODE
  if (icub_id_settings.verbose)
    printf("iter     objv        gap       |Ax-b|    |Gx+s-h|    step\n");
#endif
  icub_id_fillq();
  icub_id_fillh();
  icub_id_fillb();
  if (icub_id_settings.better_start)
    icub_id_better_start();
  else
    icub_id_set_start();
  for (iter = 0; iter < icub_id_settings.max_iters; iter++) {
    for (i = 0; i < 172; i++) {
      icub_id_work.s_inv[i] = 1.0 / icub_id_work.s[i];
      icub_id_work.s_inv_z[i] = icub_id_work.s_inv[i]*icub_id_work.z[i];
    }
    icub_id_work.block_33[0] = 0;
    icub_id_fill_KKT();
    icub_id_ldl_factor();
    /* Affine scaling directions. */
    icub_id_fillrhs_aff();
    icub_id_ldl_icub_id_solve(icub_id_work.rhs, icub_id_work.lhs_aff);
    icub_id_refine(icub_id_work.rhs, icub_id_work.lhs_aff);
    /* Centering plus corrector directions. */
    icub_id_fillrhs_cc();
    icub_id_ldl_icub_id_solve(icub_id_work.rhs, icub_id_work.lhs_cc);
    icub_id_refine(icub_id_work.rhs, icub_id_work.lhs_cc);
    /* Add the two together and store in aff. */
    for (i = 0; i < 536; i++)
      icub_id_work.lhs_aff[i] += icub_id_work.lhs_cc[i];
    /* Rename aff to reflect its new meaning. */
    dx = icub_id_work.lhs_aff;
    ds = icub_id_work.lhs_aff + 124;
    dz = icub_id_work.lhs_aff + 296;
    dy = icub_id_work.lhs_aff + 468;
    /* Find min(min(ds./s), min(dz./z)). */
    minval = 0;
    for (i = 0; i < 172; i++)
      if (ds[i] < minval*icub_id_work.s[i])
        minval = ds[i]/icub_id_work.s[i];
    for (i = 0; i < 172; i++)
      if (dz[i] < minval*icub_id_work.z[i])
        minval = dz[i]/icub_id_work.z[i];
    /* Find alpha. */
    if (-0.99 < minval)
      alpha = 1;
    else
      alpha = -0.99/minval;
    /* Update the primal and dual variables. */
    for (i = 0; i < 124; i++)
      icub_id_work.x[i] += alpha*dx[i];
    for (i = 0; i < 172; i++)
      icub_id_work.s[i] += alpha*ds[i];
    for (i = 0; i < 172; i++)
      icub_id_work.z[i] += alpha*dz[i];
    for (i = 0; i < 68; i++)
      icub_id_work.y[i] += alpha*dy[i];
    icub_id_work.gap = icub_id_eval_gap();
    icub_id_work.eq_resid_squared = icub_id_calc_eq_resid_squared();
    icub_id_work.ineq_resid_squared = icub_id_calc_ineq_resid_squared();
#ifndef ZERO_LIBRARY_MODE
    if (icub_id_settings.verbose) {
      icub_id_work.optval = icub_id_eval_objv();
      printf("%3d   %10.3e  %9.2e  %9.2e  %9.2e  % 6.4f\n",
          iter+1, icub_id_work.optval, icub_id_work.gap, sqrt(icub_id_work.eq_resid_squared),
          sqrt(icub_id_work.ineq_resid_squared), alpha);
    }
#endif
    /* Test termination conditions. Requires optimality, and satisfied */
    /* constraints. */
    if (   (icub_id_work.gap < icub_id_settings.eps)
        && (icub_id_work.eq_resid_squared <= icub_id_settings.resid_tol*icub_id_settings.resid_tol)
        && (icub_id_work.ineq_resid_squared <= icub_id_settings.resid_tol*icub_id_settings.resid_tol)
       ) {
      icub_id_work.converged = 1;
      icub_id_work.optval = icub_id_eval_objv();
      return iter+1;
    }
  }
  return iter;
}
