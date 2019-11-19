#pragma once
#include "Contacts.h"

struct IKPARAM
{
    double xinit;
    double damping;
	bool verbose;
	int num_iter;
    IKPARAM() :     xinit(10.0/180.0*M_PI),
                    damping(1e-3),
					verbose(false),
					num_iter(15){}
};

double IK(	Contact_Manager &CM, 
			IKPARAM &param, 
			Cvector& ref_pos, 
			Cvector freeze);
