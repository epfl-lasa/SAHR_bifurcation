/* Produced by CVXGEN, 2018-11-27 12:50:18 -0500.  */
/* CVXGEN is Copyright (C) 2006-2017 Jacob Mattingley, jem@cvxgen.com. */
/* The code in this file is Copyright (C) 2006-2017 Jacob Mattingley. */
/* CVXGEN, or solvers produced by CVXGEN, cannot be used for commercial */
/* applications without prior written permission from Jacob Mattingley. */

/* Filename: icub_id_solver.h. */
/* Description: Header file with relevant definitions. */
#ifndef icub_id_SOLVER_H
#define icub_id_SOLVER_H
/* Uncomment the next line to remove all library dependencies. */
/*#define ZERO_LIBRARY_MODE */
#ifdef MATLAB_MEX_FILE
/* Matlab functions. MATLAB_MEX_FILE will be defined by the mex compiler. */
/* If you are not using the mex compiler, this functionality will not intrude, */
/* as it will be completely disabled at compile-time. */
#include "mex.h"
#else
#ifndef ZERO_LIBRARY_MODE
#include <stdio.h>
#endif
#endif
/* Space must be allocated somewhere (icub_id_testsolver.c, csolve.c or your own */
/* program) for the global variables icub_id_vars, icub_id_params, icub_id_work and icub_id_settings. */
/* At the bottom of this file, they are externed. */
#ifndef ZERO_LIBRARY_MODE
#include <math.h>
#define icub_id_pm(A, m, n) icub_id_printmatrix(#A, A, m, n, 1)
#endif
typedef struct icub_id_Params_t {
  double Qq[38];
  double Qt[32];
  double tau_ref[32];
  double QqTask[30];
  double QlT_1[3];
  double QlT_2[3];
  double QlT_3[3];
  double QlT_4[3];
  double QlR_1[3];
  double QlR_2[3];
  double QlR_3[3];
  double QlR_4[3];
  double I_1[38];
  double H[38];
  double J_7[38];
  double ifp_1[1];
  double J_13[38];
  double ifp_2[1];
  double J_19[38];
  double ifp_3[1];
  double J_25[38];
  double ifp_4[1];
  double J_8[38];
  double J_14[38];
  double J_20[38];
  double J_26[38];
  double J_9[38];
  double J_15[38];
  double J_21[38];
  double J_27[38];
  double I_2[38];
  double I_3[38];
  double I_4[38];
  double J_10[38];
  double ifo_1[1];
  double J_16[38];
  double ifo_2[1];
  double J_22[38];
  double ifo_3[1];
  double J_28[38];
  double ifo_4[1];
  double J_11[38];
  double J_17[38];
  double J_23[38];
  double J_29[38];
  double J_12[38];
  double J_18[38];
  double J_24[38];
  double J_30[38];
  double I_5[38];
  double I_6[38];
  double I_7[38];
  double I_8[38];
  double I_9[38];
  double I_10[38];
  double I_11[38];
  double I_12[38];
  double I_13[38];
  double I_14[38];
  double I_15[38];
  double I_16[38];
  double I_17[38];
  double I_18[38];
  double I_19[38];
  double I_20[38];
  double I_21[38];
  double I_22[38];
  double I_23[38];
  double I_24[38];
  double I_25[38];
  double I_26[38];
  double I_27[38];
  double I_28[38];
  double I_29[38];
  double I_30[38];
  double I_31[38];
  double I_32[38];
  double I_33[38];
  double I_34[38];
  double I_35[38];
  double I_36[38];
  double I_37[38];
  double I_38[38];
  double min_tau[32];
  double max_tau[32];
  double min_ddq[32];
  double max_ddq[32];
  double Jtask_1[38];
  double dJdqTask[30];
  double xddTask[30];
  double ifTask_1[1];
  double Jtask_2[38];
  double Jtask_3[38];
  double Jtask_4[38];
  double ifTask_2[1];
  double Jtask_5[38];
  double Jtask_6[38];
  double Jtask_7[38];
  double ifTask_3[1];
  double Jtask_8[38];
  double Jtask_9[38];
  double Jtask_10[38];
  double ifTask_4[1];
  double Jtask_11[38];
  double Jtask_12[38];
  double Jtask_13[38];
  double ifTask_5[1];
  double Jtask_14[38];
  double Jtask_15[38];
  double Jtask_16[38];
  double ifTask_6[1];
  double Jtask_17[38];
  double Jtask_18[38];
  double Jtask_19[38];
  double ifTask_7[1];
  double Jtask_20[38];
  double Jtask_21[38];
  double Jtask_22[38];
  double ifTask_8[1];
  double Jtask_23[38];
  double Jtask_24[38];
  double Jtask_25[38];
  double ifTask_9[1];
  double Jtask_26[38];
  double Jtask_27[38];
  double Jtask_28[38];
  double ifTask_10[1];
  double Jtask_29[38];
  double Jtask_30[38];
  double uT_min_1[3];
  double nz_1[3];
  double nx_1[3];
  double uT_min_2[3];
  double nz_2[3];
  double nx_2[3];
  double uT_min_3[3];
  double nz_3[3];
  double nx_3[3];
  double uT_min_4[3];
  double nz_4[3];
  double nx_4[3];
  double uT_max_1[3];
  double uT_max_2[3];
  double uT_max_3[3];
  double uT_max_4[3];
  double ny_1[3];
  double ny_2[3];
  double ny_3[3];
  double ny_4[3];
  double uR_min_1[3];
  double uR_min_2[3];
  double uR_min_3[3];
  double uR_min_4[3];
  double uR_max_1[3];
  double uR_max_2[3];
  double uR_max_3[3];
  double uR_max_4[3];
  double *QlT[5];
  double *QlR[5];
  double *I[39];
  double *J[31];
  double *ifp[5];
  double *ifo[5];
  double *Jtask[31];
  double *ifTask[11];
  double *uT_min[5];
  double *nz[5];
  double *nx[5];
  double *uT_max[5];
  double *ny[5];
  double *uR_min[5];
  double *uR_max[5];
} icub_id_Params;
typedef struct icub_id_Vars_t {
  double *ddq; /* 38 rows. */
  double *tau; /* 32 rows. */
  double *slack; /* 30 rows. */
  double *F_1; /* 3 rows. */
  double *F_2; /* 3 rows. */
  double *F_3; /* 3 rows. */
  double *F_4; /* 3 rows. */
  double *T_1; /* 3 rows. */
  double *T_2; /* 3 rows. */
  double *T_3; /* 3 rows. */
  double *T_4; /* 3 rows. */
  double *F[5];
  double *T[5];
} icub_id_Vars;
typedef struct icub_id_Workspace_t {
  double h[172];
  double s_inv[172];
  double s_inv_z[172];
  double b[68];
  double q[124];
  double rhs[536];
  double x[536];
  double *s;
  double *z;
  double *y;
  double lhs_aff[536];
  double lhs_cc[536];
  double buffer[536];
  double buffer2[536];
  double KKT[2668];
  double L[3055];
  double d[536];
  double v[536];
  double d_inv[536];
  double gap;
  double optval;
  double ineq_resid_squared;
  double eq_resid_squared;
  double block_33[1];
  /* Pre-op symbols. */
  double quad_714092859392[1];
  int converged;
} icub_id_Workspace;
typedef struct icub_id_Settings_t {
  double resid_tol;
  double eps;
  int max_iters;
  int refine_steps;
  int better_start;
  /* Better start obviates the need for s_init and z_init. */
  double s_init;
  double z_init;
  int verbose;
  /* Show extra details of the iterative refinement steps. */
  int verbose_refinement;
  int debug;
  /* For regularization. Minimum value of abs(D_ii) in the kkt D factor. */
  double kkt_reg;
} icub_id_Settings;
extern icub_id_Vars icub_id_vars;
extern icub_id_Params icub_id_params;
extern icub_id_Workspace icub_id_work;
extern icub_id_Settings icub_id_settings;
/* Function definitions in ldl.c: */
void icub_id_ldl_icub_id_solve(double *target, double *var);
void icub_id_ldl_factor(void);
double icub_id_check_factorization(void);
void icub_id_matrix_multiply(double *result, double *source);
double icub_id_check_residual(double *target, double *multiplicand);
void icub_id_fill_KKT(void);

/* Function definitions in matrix_support.c: */
void icub_id_multbymA(double *lhs, double *rhs);
void icub_id_multbymAT(double *lhs, double *rhs);
void icub_id_multbymG(double *lhs, double *rhs);
void icub_id_multbymGT(double *lhs, double *rhs);
void icub_id_multbyP(double *lhs, double *rhs);
void icub_id_fillq(void);
void icub_id_fillh(void);
void icub_id_fillb(void);
void icub_id_pre_ops(void);

/* Function definitions in solver.c: */
double icub_id_eval_gap(void);
void icub_id_set_defaults(void);
void icub_id_setup_pointers(void);
void setup_indexed_icub_id_params(void);
void setup_indexed_opticub_id_vars(void);
void icub_id_setup_indexing(void);
void icub_id_set_start(void);
double icub_id_eval_objv(void);
void icub_id_fillrhs_aff(void);
void icub_id_fillrhs_cc(void);
void icub_id_refine(double *target, double *var);
double icub_id_calc_ineq_resid_squared(void);
double icub_id_calc_eq_resid_squared(void);
void icub_id_better_start(void);
void icub_id_fillrhs_start(void);
long icub_id_solve(void);

/* Function definitions in icub_id_testsolver.c: */
int icub_id_main(int argc, char **argv);
void icub_id_load_default_data(void);

/* Function definitions in util.c: */
void icub_id_tic(void);
float icub_id_toc(void);
float icub_id_tocq(void);
void icub_id_printmatrix(char *name, double *A, int m, int n, int sparse);
double icub_id_unif(double lower, double upper);
float icub_id_ran1(long*idum, int reset);
float icub_id_randn_internal(long *idum, int reset);
double icub_id_randn(void);
void icub_id_reset_rand(void);

#endif
