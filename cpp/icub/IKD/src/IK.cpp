#include "IK.h"
#include "ID.h"
using namespace std;
using namespace Eigen;

#ifdef __cplusplus
	extern "C" {
	#include "IKCG_solver.h"
	}
#endif

IKCG_Vars IKCG_vars;
IKCG_Params IKCG_params;
IKCG_Workspace IKCG_work;
IKCG_Settings IKCG_settings;

Cvector4 getQ(Cvector Q)
{
	Cvector4 a;
	a[3] = Q[AIR_N_U];
	a.segment(0,3) = Q.segment(3,3);
	return a;
}

Cvector setQ(Cvector Q, Cvector4 a)
{
	Q[AIR_N_U] = a[3];
	Q.segment(3,3) = a.segment(0,3);
	return Q;
}

void get_info(Contact_Manager &CM, Model &model, Cvector q0, Cmatrix &J, Cvector &X0, Cvector &Xr, Cvector &slack)
{
	Cvector dq0 = Cvector::Zero(AIR_N_U);
	model.set_state(0, q0, dq0);
	int beg_index = 0;

	for(int index=0;index<CM.size();index++)
	{
		int body = CM.C[index].body;
		Cvector3 offset = CM.C[index].offset;
		Cmatrix JJ = model.get_jacob(body, offset, CT_FULL);

		if(CM.C[index].contact_type==CT_FULL || CM.C[index].contact_type==CT_TRANSLATION)
		{
			X0.segment(beg_index,3)        = model.get_pos(body, offset);
			Xr.segment(beg_index,3)        = CM.C[index].ref_p.pos;
			slack.segment(beg_index,3)     = CM.C[index].T.slack_cost;
			J.block(beg_index,0,3,AIR_N_U) = JJ.block(0,0,3,AIR_N_U);
			if(index==0 && CM.ifCoM)
			{
				X0.segment(beg_index,3)    = model.get_cm();
				J.block(beg_index,0,3,AIR_N_U) = model.get_cm_J();
			}
			beg_index += 3;
		}
		if(CM.C[index].contact_type==CT_FULL || CM.C[index].contact_type==CT_ROTATION)
		{
			Cvector qact = quat_sub(model.get_orient_quat(body), CM.C[index].ref_o.pos);
			Cvector qref = ang2quat(Cvector3(0,0,0));
			X0.segment(beg_index,3)        = qact.segment(0,3);
			Xr.segment(beg_index,3)        = qref.segment(0,3);
			slack.segment(beg_index,3)     = CM.C[index].R.slack_cost;
			J.block(beg_index,0,3,AIR_N_U) = JJ.block(3,0,3,AIR_N_U);
			beg_index += 3;
		}
	}
}

Cvector solve_flex(Cmatrix &J, Cvector dX, Cvector &slack, Cvector &damping, Cvector &qref, Cvector &qlow, Cvector &qup)
{
	IKCG_set_defaults();
	IKCG_setup_indexing();

	for(int i=0;i<N_TASK;i++)
		memcpy(IKCG_params.J[i+1], &J(i,0),     sizeof(double)*(AIR_N_U));
	memcpy(IKCG_params.qlow,       &qlow[0],    sizeof(double)*(AIR_N_U-6));
	memcpy(IKCG_params.qup,        &qup[0],     sizeof(double)*(AIR_N_U-6));
	memcpy(IKCG_params.dx,         &dX[0],      sizeof(double)*(N_TASK));
	memcpy(IKCG_params.damping,    &damping[0], sizeof(double)*(AIR_N_U));
	memcpy(IKCG_params.qref,       &qref[0],    sizeof(double)*(AIR_N_U));
	memcpy(IKCG_params.slack,      &slack[0],   sizeof(double)*(N_TASK));

	Cvector hard = slack;
	for(int i=0; i<hard.size(); i++)
		hard[i] = hard[i]==-1 ? 1 : 0;
	memcpy(IKCG_params.hard, &hard[0], sizeof(double)*(N_TASK));

	IKCG_settings.verbose = false;
	int iter = IKCG_solve();
	Cvector dq = Cvector::Zero(AIR_N_U);
	memcpy(&dq[0], IKCG_vars.dq, sizeof(double)*(AIR_N_U));
	return dq;
}

double return_error(Contact_Manager &CM, Cvector& ref_pos)
{
	// call exactly after solving IK
	Cvector Xr = Cvector::Zero(N_TASK);
	Cvector X0 = Cvector::Zero(N_TASK);
	Cvector slack = Cvector::Zero(N_TASK);
	Cmatrix J = Cmatrix::Zero(N_TASK, AIR_N_U);
	get_info(CM, *CM.M, ref_pos, J, X0, Xr, slack);
	return (Xr-X0).segment(18,12).norm();
}

double IK(Contact_Manager &CM, IKPARAM &param, Cvector& ref_pos, Cvector freeze)
{
	timeval start, end;
	gettimeofday(&start, NULL);

	// initialize variables
	Model model = *CM.M;
	Cvector Xr = Cvector::Zero(N_TASK);
	Cvector X0 = Cvector::Zero(N_TASK);
	Cvector slack = Cvector::Zero(N_TASK);
	Cmatrix J = Cmatrix::Zero(N_TASK, AIR_N_U);
	Cvector damping = param.damping * Cvector::Ones(AIR_N_U);

	for(int j=0;j<param.num_iter;j++)
	{
		// recalculate gradients
		get_info(CM, model, ref_pos, J, X0, Xr, slack);

		for(int i=0;i<AIR_N_U;i++)
			if(freeze[i]==1)
				J.block(0,i,N_TASK,1) *= 0;

		Cvector qlow = model.qmin/180.0*M_PI - ref_pos.segment(6,AIR_N_U-6);
		Cvector qup  = model.qmax/180.0*M_PI - ref_pos.segment(6,AIR_N_U-6);
		Cvector dX 	 = Xr-X0;

		Cvector qref = Cvector::Zero(AIR_N_U);
		qref.segment(6,AIR_N_U-6) = - (qup+qlow) * 0.5 * 0;
		qref = qref.cwiseProduct(Cvector::Ones(AIR_N_U)-freeze);

		// solve linearized CVXGEN problem
		Cvector dq = solve_flex(J, dX, slack, damping, qref, qlow, qup);
		dq = dq.cwiseProduct(Cvector::Ones(AIR_N_U)-freeze);

		// apply dq
		Cvector q0 = getQ(ref_pos);
		ref_pos.segment(0,AIR_N_U) += dq;
		Cvector4 w(0,0,0,0);
		w.segment(0,3) = quat2dc(q0) * dq.segment(3,3);
		Cvector rotBase = quat_mul(quat_exp(0.5 * w), q0);
		ref_pos = setQ(ref_pos,rotBase);
	}


	gettimeofday(&end, NULL);
	return (end.tv_sec-start.tv_sec) + (end.tv_usec-start.tv_usec)/1e6;
}



