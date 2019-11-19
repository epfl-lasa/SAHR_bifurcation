#pragma once
#include "Contacts.h"

struct IDPARAM
{
    // Core options of CVXGEN inverse dynamics formulations
    //! If inverse dynamics prints a record or not
    bool verbose;
    //! Maximum number of iterations for inverse dynamics, refer to cvxgen documentation
    int CVXGEN_NUM_ITERATION;
    //! If cvxgen shows details of each iteration or not, refer to cvxgen documentation
    int CVXGEN_VERBOSE;
    //! Bound for duality gap in cvxgen, refer to cvxgen documentation
    double CVXGEN_EPS;
    //! Bound for equality and inequality gaps, refer to cvxgen documentation
    double CVXGEN_RESID_TOL;
    //! Stability parameter, refer to cvxgen documentation
    double CVXGEN_KKT_REG;
    //! If cvxgen refines the solution or not, refer to cvxgen documentation
    int CVXGEN_REFINE_STEPS;
    //! If refinement is verbose or not, refer to cvxgen documentation
    int CVXGEN_VERBOSE_REFINEMENT;
    //! Refer to cvxgen documentation
    double CVXGEN_S_INIT;
    //! Refer to cvxgen documentation
    double CVXGEN_Z_INIT;
    //! Refer to cvxgen documentation
    int CVXGEN_BETTER_START;
    IDPARAM() :     verbose(false),
                    CVXGEN_NUM_ITERATION(15),
                    CVXGEN_VERBOSE(false),
                    CVXGEN_EPS(1e-4),
                    CVXGEN_RESID_TOL(1e-6),
                    CVXGEN_KKT_REG(1e-7),
                    CVXGEN_REFINE_STEPS(1),
                    CVXGEN_VERBOSE_REFINEMENT(0),
                    CVXGEN_S_INIT(1),
                    CVXGEN_Z_INIT(1),
                    CVXGEN_BETTER_START(1) {}
};

double ID(  Contact_Manager &CM, 
			IDPARAM &param,
            Cvector Qacc, 
			Cvector Qtau,
			Cvector &o_tau, 
			Cvector &o_acc, 
			Cvector &freeze, 
			Cvector tau_des_joint);
