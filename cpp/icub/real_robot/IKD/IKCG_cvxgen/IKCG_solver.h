/* Produced by CVXGEN, 2018-11-08 10:47:22 -0500.  */
/* CVXGEN is Copyright (C) 2006-2017 Jacob Mattingley, jem@cvxgen.com. */
/* The code in this file is Copyright (C) 2006-2017 Jacob Mattingley. */
/* CVXGEN, or solvers produced by CVXGEN, cannot be used for commercial */
/* applications without prior written permission from Jacob Mattingley. */

/* Filename: IKCG_solver.h. */
/* Description: Header file with relevant definitions. */
#ifndef IKCG_SOLVER_H
#define IKCG_SOLVER_H
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
/* Space must be allocated somewhere (IKCG_testsolver.c, csolve.c or your own */
/* program) for the global variables IKCG_vars, IKCG_params, IKCG_work and IKCG_settings. */
/* At the bottom of this file, they are externed. */
#ifndef ZERO_LIBRARY_MODE
#include <math.h>
#define IKCG_pm(A, m, n) IKCG_printmatrix(#A, A, m, n, 1)
#endif
typedef struct IKCG_Params_t {
  double damping[38];
  double qref[38];
  double slack[30];
  double J_1[38];
  double dx[30];
  double J_2[38];
  double J_3[38];
  double J_4[38];
  double J_5[38];
  double J_6[38];
  double J_7[38];
  double J_8[38];
  double J_9[38];
  double J_10[38];
  double J_11[38];
  double J_12[38];
  double J_13[38];
  double J_14[38];
  double J_15[38];
  double J_16[38];
  double J_17[38];
  double J_18[38];
  double J_19[38];
  double J_20[38];
  double J_21[38];
  double J_22[38];
  double J_23[38];
  double J_24[38];
  double J_25[38];
  double J_26[38];
  double J_27[38];
  double J_28[38];
  double J_29[38];
  double J_30[38];
  double qlow[32];
  double qup[32];
  double *J[31];
} IKCG_Params;
typedef struct IKCG_Vars_t {
  double *dq; /* 38 rows. */
  double *delta; /* 30 rows. */
} IKCG_Vars;
typedef struct IKCG_Workspace_t {
  double h[64];
  double s_inv[64];
  double s_inv_z[64];
  double b[30];
  double q[68];
  double rhs[226];
  double x[226];
  double *s;
  double *z;
  double *y;
  double lhs_aff[226];
  double lhs_cc[226];
  double buffer[226];
  double buffer2[226];
  double KKT[777];
  double L[752];
  double d[226];
  double v[226];
  double d_inv[226];
  double gap;
  double optval;
  double ineq_resid_squared;
  double eq_resid_squared;
  double block_33[1];
  /* Pre-op symbols. */
  double quad_568441778176[1];
  int converged;
} IKCG_Workspace;
typedef struct IKCG_Settings_t {
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
} IKCG_Settings;
extern IKCG_Vars IKCG_vars;
extern IKCG_Params IKCG_params;
extern IKCG_Workspace IKCG_work;
extern IKCG_Settings IKCG_settings;
/* Function definitions in ldl.c: */
void IKCG_ldl_IKCG_solve(double *target, double *var);
void IKCG_ldl_factor(void);
double IKCG_check_factorization(void);
void IKCG_matrix_multiply(double *result, double *source);
double IKCG_check_residual(double *target, double *multiplicand);
void IKCG_fill_KKT(void);

/* Function definitions in matrix_support.c: */
void IKCG_multbymA(double *lhs, double *rhs);
void IKCG_multbymAT(double *lhs, double *rhs);
void IKCG_multbymG(double *lhs, double *rhs);
void IKCG_multbymGT(double *lhs, double *rhs);
void IKCG_multbyP(double *lhs, double *rhs);
void IKCG_fillq(void);
void IKCG_fillh(void);
void IKCG_fillb(void);
void IKCG_pre_ops(void);

/* Function definitions in solver.c: */
double IKCG_eval_gap(void);
void IKCG_set_defaults(void);
void IKCG_setup_pointers(void);
void setup_indexed_IKCG_params(void);
void IKCG_setup_indexing(void);
void IKCG_set_start(void);
double IKCG_eval_objv(void);
void IKCG_fillrhs_aff(void);
void IKCG_fillrhs_cc(void);
void IKCG_refine(double *target, double *var);
double IKCG_calc_ineq_resid_squared(void);
double IKCG_calc_eq_resid_squared(void);
void IKCG_better_start(void);
void IKCG_fillrhs_start(void);
long IKCG_solve(void);

/* Function definitions in IKCG_testsolver.c: */
int IKCG_main(int argc, char **argv);
void IKCG_load_default_data(void);

/* Function definitions in util.c: */
void IKCG_tic(void);
float IKCG_toc(void);
float IKCG_tocq(void);
void IKCG_printmatrix(char *name, double *A, int m, int n, int sparse);
double IKCG_unif(double lower, double upper);
float IKCG_ran1(long*idum, int reset);
float IKCG_randn_internal(long *idum, int reset);
double IKCG_randn(void);
void IKCG_reset_rand(void);

#endif
