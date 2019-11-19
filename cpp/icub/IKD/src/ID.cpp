#include "ID.h"

using namespace std;
using namespace Eigen;

#ifdef __cplusplus
	extern "C" {
	#include "icub_id_solver.h"
	}
#endif

icub_id_Vars icub_id_vars;
icub_id_Params icub_id_params;
icub_id_Workspace icub_id_work;
icub_id_Settings icub_id_settings;

double ID(  Contact_Manager &CM,
			IDPARAM &param,
			Cvector Qacc, Cvector Qtau,
			Cvector &o_tau, Cvector &o_acc, Cvector &freeze, 
			Cvector tau_des_joint)
{
	timeval start, end;
	gettimeofday(&start, NULL);

	if(param.verbose)
		printf("----- icub_id_Optimization: ------------------\n");

	Cmatrix I = CM.M->get_massmat();
	Cvector h = CM.M->get_frcmat() * (-1) - CM.get_virtual_mult();
	for(int i=0;i<AIR_N_U;i++)
		if(freeze[i]==1)
			I.block(0,i,AIR_N_U,1) *= 0;

	// initialize solver
	icub_id_set_defaults();
	icub_id_setup_indexing();

	// equality constraint, EoM
	for(int i=0;i<AIR_N_U;i++)
		memcpy(icub_id_params.I[i+1], &I(i,0), sizeof(double)*(AIR_N_U));
	memcpy(icub_id_params.H, &h[0], sizeof(double)*(AIR_N_U));

	memcpy(icub_id_params.min_tau, &CM.M->taumin[0], sizeof(double)*(AIR_N_U-6));
	memcpy(icub_id_params.max_tau, &CM.M->taumax[0], sizeof(double)*(AIR_N_U-6));

	memcpy(icub_id_params.min_ddq, &CM.M->ddqmin[0], sizeof(double)*(AIR_N_U-6));
	memcpy(icub_id_params.max_ddq, &CM.M->ddqmax[0], sizeof(double)*(AIR_N_U-6));

	memcpy(icub_id_params.tau_ref, &tau_des_joint[6], sizeof(double)*(AIR_N_U-6));

	memcpy(icub_id_params.Qt, &Qtau[0], sizeof(double)*(AIR_N_U-6));
	memcpy(icub_id_params.Qq, &Qacc[0], sizeof(double)*(AIR_N_U));

	for(unsigned int i=1;i<CM.size();i++)
	{
		load_cvector3(icub_id_params.nx[i],  CM.C[i].n1);
		load_cvector3(icub_id_params.ny[i],  CM.C[i].n2);
		load_cvector3(icub_id_params.nz[i],  CM.C[i].n3);
		load_cvector3(icub_id_params.uT_min[i],  Cvector3(CM.C[i].mu, CM.C[i].mu, 1)*-1);
		load_cvector3(icub_id_params.uT_max[i],  Cvector3(CM.C[i].mu, CM.C[i].mu, 1));
		load_cvector3(icub_id_params.QlT[i], CM.C[i].T.F_cost);

		load_cvector3(icub_id_params.uR_min[i], Cvector3(CM.C[i].w_y[0], CM.C[i].w_x[0], -CM.C[i].muR));
		load_cvector3(icub_id_params.uR_max[i], Cvector3(CM.C[i].w_y[1], CM.C[i].w_x[1], CM.C[i].muR));
		load_cvector3(icub_id_params.QlR[i],    CM.C[i].R.F_cost); 
	}




	Cmatrix JTask;
	Cvector dJdqTask, xddTask, QqTask;
	int M = 30;
	CM.tasks(JTask, dJdqTask, xddTask, QqTask);
	memcpy(icub_id_params.xddTask, &xddTask[0], sizeof(double)*(M));
	memcpy(icub_id_params.dJdqTask, &dJdqTask[0], sizeof(double)*(M));
	memcpy(icub_id_params.QqTask, &QqTask[0], sizeof(double)*(M));
	for(int i=6;i<M;i++)
		memcpy(icub_id_params.J[i+1], &JTask(i,0), sizeof(double)*(AIR_N_U));
	for(int i=0;i<AIR_N_U;i++)
		if(freeze[i]==1)
			JTask.block(0,i,M,1) *= 0;
	for(int i=0;i<M;i++)
		memcpy(icub_id_params.Jtask[i+1], &JTask(i,0), sizeof(double)*(AIR_N_U));

	for(int i=0;i<CM.size();i++)
	{
		bool rot  = CM.C[i].status!=PS_NO_CONTROL && (CM.C[i].contact_type==CT_ROTATION || CM.C[i].contact_type==CT_FULL);
		bool trans = CM.C[i].status!=PS_NO_CONTROL && (CM.C[i].contact_type==CT_TRANSLATION || CM.C[i].contact_type==CT_FULL);
		icub_id_params.ifTask[2*i+1][0] = trans;
		icub_id_params.ifTask[2*i+2][0] = rot;
		if(i>0)
		{
			icub_id_params.ifp[i][0] = trans && CM.C[i].status!=PS_FLOATING;
			icub_id_params.ifo[i][0] = rot && CM.C[i].status!=PS_FLOATING;
		}
	}

	// solving the problem
	icub_id_settings.verbose =              param.CVXGEN_VERBOSE;
	icub_id_settings.eps =                  param.CVXGEN_EPS;
	icub_id_settings.resid_tol =            param.CVXGEN_RESID_TOL;
	icub_id_settings.kkt_reg =              param.CVXGEN_KKT_REG;
	icub_id_settings.refine_steps =         param.CVXGEN_REFINE_STEPS;
	icub_id_settings.verbose_refinement =   param.CVXGEN_VERBOSE_REFINEMENT;
	icub_id_settings.s_init =               param.CVXGEN_S_INIT;
	icub_id_settings.z_init =               param.CVXGEN_Z_INIT;
	icub_id_settings.better_start =         param.CVXGEN_BETTER_START;
	icub_id_settings.max_iters =            param.CVXGEN_NUM_ITERATION;
	int iter = icub_id_solve();

	if(param.verbose)
	{
		printf("iter     objv        gap       |Ax-b|    |Gx+s-h|    step\n");
		printf("%3d   %10.3e  %9.2e  %9.2e  %9.2e  % 6.4f\n",
		iter, icub_id_eval_objv(), icub_id_work.gap, sqrt(icub_id_work.eq_resid_squared),
		sqrt(icub_id_work.ineq_resid_squared), 0.0);
	}

	//return values
	for(unsigned int i=0;i<CM.size();i++)
	{
		memcpy(&CM.C[i].T.slack[0], &icub_id_vars.slack[i*6], sizeof(double)*(3));
		memcpy(&CM.C[i].R.slack[0], &icub_id_vars.slack[i*6+3], sizeof(double)*(3));
		if(i>=1)
		{
			if(icub_id_params.ifp[i][0])
				save_Cvector3(CM.C[i].T.F_est, icub_id_vars.F[i]);
			else
				CM.C[i].T.F_est = zero_v3;
			if(icub_id_params.ifo[i][0])
				save_Cvector3(CM.C[i].R.F_est, icub_id_vars.T[i]);
			else
				CM.C[i].R.F_est = zero_v3;
		}
	}

	o_tau = Cvector::Zero(AIR_N_U);
	memcpy(&o_tau[6], icub_id_vars.tau, sizeof(double)*(AIR_N_U-6));
	o_acc = Cvector::Zero(AIR_N_U);
	memcpy(&o_acc[0], icub_id_vars.ddq, sizeof(double)*(AIR_N_U));
	o_acc = o_acc.cwiseProduct(Cvector::Ones(AIR_N_U)-freeze);

	gettimeofday(&end, NULL);
	return (end.tv_sec-start.tv_sec) + (end.tv_usec-start.tv_usec)/1e6;
}


