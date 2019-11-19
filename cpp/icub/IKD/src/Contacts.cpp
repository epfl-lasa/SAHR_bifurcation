#include "Contacts.h"
using namespace std;

char names[5][6+1] = {"pelvis", "l_foot", "r_foot", "l_hand", "r_hand"};

void Geom_Point_print(Geom_Point in, __const char *str)
{
    Cvector all(9);
    all[0] = in.pos[0];
    all[1] = in.pos[1];
    all[2] = in.pos[2];
    all[3] = in.vel[0];
    all[4] = in.vel[1];
    all[5] = in.vel[2];
    all[6] = in.acc[0];
    all[7] = in.acc[1];
    all[8] = in.acc[2];
    cout << str << all << endl;
}

void print_comparitive(__const char *str, Geom_Point ref, Geom_Point actual)
{
    printf("compare %s :\n",str);
    for(int i=0;i<3;i++)
        printf("| %3.2f <- %3.2f  \t| %3.2f <- %3.2f  \t| %3.2f <- %3.2f \n",
               ref.pos[i], actual.pos[i], ref.vel[i], actual.vel[i], ref.acc[i], actual.acc[i]);
}

////////////////// trajectory generation /////////////////////////

void apply_exp(double A, double t, double tau, double * pos, double * vel, double * acc)
{
    double e = exp(-(t/tau)*(t/tau));
    *pos += A * (1.0 - e);
    *vel += -A * (-2.0*t/tau/tau*e);
    *acc += -A * (-2.0/tau/tau*e + 4.0*t*t/tau/tau/tau/tau*e);
}

void apply_exp(double A, double t, double tau, Cvector& pos, Cvector& vel, Cvector& acc, int index)
{
    apply_exp(A, t, tau, &pos[index], &vel[index], &acc[index]);
}

void apply_exp(double A, double t, double tau, Geom_Point &in, Dir_Axis index)
{
    apply_exp(A, t, tau, &in.pos[index], &in.vel[index], &in.acc[index]);
}

////////////////////////////////////////////////////////////////////////////////////////////////////////////

void apply_sin(double A, double t, double w, double phi, double * pos, double * vel, double * acc)
{
    *pos += A * sin(w*t+phi);
    *vel += A * w * cos(w*t+phi);
    *acc += -A * w * w * sin(w*t+phi);
}

void apply_sin(double A, double t, double w, double phi, Cvector& pos, Cvector& vel, Cvector& acc, int index)
{
    apply_sin(A, t, w, phi, &pos[index], &vel[index], &acc[index]);
}

void apply_sin(double A, double t, double w, double phi, Geom_Point &in, Dir_Axis index)
{
    apply_sin(A, t, w, phi, &in.pos[index], &in.vel[index], &in.acc[index]);
}

////////////////////////////////////////////////////////////////////////////////////////////////////////////

void apply_sin2(double A, double t, double w, double * pos, double * vel, double * acc)
{
    *pos += A * pow(sin(w*t),2.0);
    *vel += A * w * sin(2.0*w*t);
    *acc += 2.0* A * w * w * cos(2.0*w*t);
}

void apply_sin2(double A, double t, double w, Geom_Point &in, Dir_Axis index)
{
    apply_sin2(A, t, w, &in.pos[index], &in.vel[index], &in.acc[index]);
}

////////////////////////////////////////////////////////////////////////////////////////////////////////////

void apply_sin3(double A, double t, double w, double * pos, double * vel, double * acc)
{
    A = 1.0/(1.0/4.0 + 1.0/12.0)/2.0*A;
    *pos += A * (sin(w*t)/2.0 - sin(3.0*w*t)/2.0/3.0);
    *vel += A * ((w*cos(t*w))/2.0 - (w*cos(3.0*t*w))/2.0);
    *acc += A * ((3.0*w*w*sin(3.0*t*w))/2.0 - (w*w*sin(t*w))/2.0);
}

void apply_sin3(double A, double t, double w, Geom_Point &in, Dir_Axis index)
{
    apply_sin3(A, t, w, &in.pos[index], &in.vel[index], &in.acc[index]);
}

////////////////////////////////////////////////////////////////////////////////////////////////////////////

void apply_expsin(double t, double A, double tau, double w, double phi, double* pos, double* vel, double* acc)
{
    double p=0, v=0, a=0;
    double p1=0, v1=0, a1=0;
    apply_exp(A, t, tau, &p, &v, &a);
    apply_sin(1.0, t, w, phi, &p1, &v1, &a1);
    *pos += p * p1;
    *vel += p * v1 + v * p1;
    *acc += p * a1 + 2.0 * v * v1 + a * p1;
}

void apply_expsin(double t, double A, double tau, double w, double phi, Geom_Point &in, Dir_Axis index)
{
    apply_expsin(t, A, tau, w, phi, &in.pos[index], &in.vel[index], &in.acc[index]);
}

void apply_expsin(double t, double A, double tau, double w, double phi, Cvector3& pos, Cvector3& vel, Cvector3& acc, int index)
{
    apply_expsin(t, A, tau, w, phi, &pos[index], &vel[index], &acc[index]);
}

////////////////////////////////////////////////////////////////////////////////////////////////////////////

void apply_fexp(double t, double A, double tau, double f, double df, double ddf, Cvector& pos, Cvector& vel, Cvector& acc, int index)
{
    double p=0, v=0, a=0;
    apply_exp(A, t, tau, &p, &v, &a);
    pos[index] += p * f;
    vel[index] += p * df + v * f;
    acc[index] += p * ddf + 2.0 * v * df + a * f;
}

////////////////////////////////////////////////////////////////////////////////////////////////////////////

void exp_transition(double t, double tau,
                    double p, double v, double a,
                    double p_des, double v_des, double a_des,
                    double &p_out, double &v_out, double &a_out)
{
    double e = exp(-t*t/tau/tau);
    double de = (-2.0*t/tau/tau) * e;
    double dde = (-2.0/tau/tau) * e + (-2.0*t/tau/tau) * de;
    p_out = (p-p_des) * e + p_des;
    v_out = (v-v_des) * e + (p-p_des) * de + v_des;
    a_out = (a-a_des) * e + (v-v_des) * de * 2.0 + (p-p_des) * dde + a_des;
}

void exp_transition(double t, double tau,
                    Cvector3 p, Cvector3 v, Cvector3 a,
                    Cvector3 p_des, Cvector3 v_des, Cvector3 a_des,
                    Cvector3 &p_out, Cvector3 &v_out, Cvector3 &a_out)
{
    for(int i=0;i<3;i++)
        exp_transition( t, tau,
                        p[i], v[i], a[i],
                        p_des[i], v_des[i], a_des[i],
                        p_out[i], v_out[i], a_out[i]);
}

void exp_transition(double t, double tau, Geom_Point & init, Geom_Point & end, Geom_Point & out)
{
    exp_transition(t, tau,
                   init.pos, init.vel, init.acc,
                    end.pos, end.vel, end.acc,
                    out.pos, out.vel, out.acc);
}

////////////////////////////////////////////////////////////////////////////////////////////////////////////

void slerp(double t, double T, double tau, Geom_Point & init, Geom_Point & end, Geom_Point & out)
{
    for(int i=0;i<3;i++)
        slerp(t, T, tau, init, end, out, (Dir_Axis)i);
}

void slerp(double t, double T, double tau, Geom_Point & init, Geom_Point & end, Geom_Point & out, Dir_Axis index)
{
    t -= T/2.0;
    double e = exp(-t/tau);
    double de = (-1.0/tau) * e;
    double dde = (-1.0/tau) * de;
    double s = pow(1.0+e,-1);
    double ds = -de*pow(1.0+e,-2);
    double dds = -dde*pow(1.0+e,-2) + 2.0*pow(de,2)*pow(1.0+e,-3);
    out.pos[index] = init.pos[index] + (end.pos[index] - init.pos[index]) * s;
    out.vel[index] = init.vel[index] + (end.vel[index] - init.vel[index]) * s + (end.pos[index] - init.pos[index]) * ds;
    out.acc[index] = init.acc[index] + (end.acc[index] - init.acc[index]) * s +
                    (end.vel[index] - init.vel[index]) * ds * 2.0 +
                    (end.pos[index] - init.pos[index]) * dds;
}

void slerp_sin(double t, double T, Geom_Point & init, Geom_Point & end, Geom_Point & out)
{
    for(int i=0;i<3;i++)
        slerp_sin(t, T, init, end, out, (Dir_Axis)i);
}

void slerp_sin(double t, double T, Geom_Point & init, Geom_Point & end, Geom_Point & out, Dir_Axis index)
{
    double theta = M_PI/T;
    double s = 2.0*t-1.0/theta*sin(2.0*theta*t);
    double ds = 2.0 - 2.0*cos(2.0*theta*t);
    double dds = 4.0 * theta * sin(2.0*theta*t);
    out.pos[index] = init.pos[index] + (end.pos[index] - init.pos[index]) * s;
    out.vel[index] = init.vel[index] + (end.vel[index] - init.vel[index]) * s + (end.pos[index] - init.pos[index]) * ds;
    out.acc[index] = init.acc[index] + (end.acc[index] - init.acc[index]) * s +
                    (end.vel[index] - init.vel[index]) * ds * 2.0 +
                    (end.pos[index] - init.pos[index]) * dds;
}

void slerp_poly(double t, double T, Geom_Point & init, Geom_Point & end, Geom_Point & out)
{
    for(int i=0;i<3;i++)
        slerp_poly(t, T, init, end, out, (Dir_Axis)i);
}

void slerp_poly(double t, double T, Geom_Point & init, Geom_Point & end, Geom_Point & out, Dir_Axis index)
{
    double p = t/T;
    double a = 6, b = -15, c = 10;
    double s = p*p*p*(a*p*p + b*p + c);
    double ds = p*p*(5*a*p*p + 4*b*p + 3*c)/T;
    double dds = p*(20*a*p*p + 12*b*p + 6*c)/T/T;
    out.pos[index] = init.pos[index] + (end.pos[index] - init.pos[index]) * s;
    out.vel[index] = init.vel[index] + (end.vel[index] - init.vel[index]) * s + (end.pos[index] - init.pos[index]) * ds;
    out.acc[index] = init.acc[index] + (end.acc[index] - init.acc[index]) * s +
                    (end.vel[index] - init.vel[index]) * ds * 2.0 +
                    (end.pos[index] - init.pos[index]) * dds;
}

void exp_transition_quat(double t, double T, double tau, Geom_Rot & init, Geom_Rot & end, Geom_Rot & out)
{
    t -= T/2.0;
    double e = exp(-t/tau);
    double de = (-1.0/tau) * e;
    double dde = (-1.0/tau) * de;

    // sigmoid
    double s = pow(1.0+e,-1);
    double ds = -de*pow(1.0+e,-2);
    double dds = -dde*pow(1.0+e,-2) + 2.0*pow(de,2)*pow(1.0+e,-3);

    //between 0 and 1
    e = s;
    de = ds;
    dde = dds;

    t += T/2.0;

    Cvector o0t = 0.5*(0.5 * pow(t-0.0,2) * quat_deriv(init.acc) + (t-0.0) * quat_deriv(init.vel)) + quat_log(init.pos);
    Cvector w0t = (t-0.0) * quat_deriv(init.acc) + quat_deriv(init.vel);
    Cvector al0t = quat_deriv(init.acc);
    Cvector q0 = quat_exp(o0t);
    Cvector dq0 = 0.5 * quat_cross(q0, w0t);
    Cvector ddq0 = 0.5 * quat_cross(q0,al0t) + 0.5 * quat_cross(dq0,w0t);

    Cvector o1t = 0.5*(0.5 * pow(t-T,2) * quat_deriv(end.acc) + (t-T) * quat_deriv(end.vel)) + quat_log(end.pos);
    Cvector w1t = (t-T) * quat_deriv(end.acc) + quat_deriv(end.vel);
    Cvector al1t = quat_deriv(end.acc);
    Cvector q1 = quat_exp(o1t);
    Cvector dq1 = 0.5 * quat_cross(q1, w1t);
    Cvector ddq1 = 0.5 * quat_cross(q1,al1t) + 0.5 * quat_cross(dq1,w1t);

    Cvector q0i = quat_inverse(q0);
    Cvector dq0i = -quat_cross(q0i, dq0, q0i);
    Cvector ddq0i = -quat_cross(q0i, quat_cross(ddq0,q0i) + 2.0*quat_cross(dq0,dq0i));

    Cvector A = quat_cross(q1, q0i);
    Cvector dA = quat_cross(dq1, q0i) + quat_cross(q1, dq0i);
    Cvector ddA =   quat_cross(ddq1, q0i) + 2 * quat_cross(dq1, dq0i) + quat_cross(q1, ddq0i);

    if(A[3]<0)
    {
        A *= -1;
        dA *= -1;
        ddA *= -1;
    }

    Cvector At = quat_pow(A,e);
    Cvector dAt = e * quat_cross(dA, quat_pow(A,e-1));
    Cvector ddAt = e * quat_cross(ddA, quat_pow(A,e-1)) +
                    e * (e-1) * quat_cross(dA, dA, quat_pow(A,e-2));

    Cvector pm = quat_cross(At,q0);
    Cvector vm = quat_cross(dAt,q0) + quat_cross(At,dq0);
    Cvector am = quat_cross(ddAt,q0) + 2.0 * quat_cross(dAt,dq0) + quat_cross(At,ddq0);

    vm = vm + de * quat_cross(quat_log(A) , pm);
    am = am + dde * quat_cross(quat_log(A) , pm) +
              de * de * quat_cross(quat_log(A) , quat_log(A) , pm);

    Cvector wm = 2.0 * quat_cross(quat_inverse(pm) , vm);
    Cvector alm = 2.0 * quat_cross(quat_inverse(pm) , am - 0.5 * quat_cross(vm,wm));

    out.pos = pm;
    out.vel = Cvector3(wm[0], wm[1], wm[2]);
    out.acc = Cvector3(alm[0], alm[1], alm[2]);
}

Contact::Contact()
{
    initialize_contact();
}

Contact::~Contact()
{
    body = 0;
}

void Contact::init(Contact_Name Name,
            int Body,
            Cvector3 Offset,
            constraint_type Contact_type,
            Point_Status Status,
			double length,
			double width)
{
    initialize_contact();
    name = Name;
    body = Body;
    offset = Offset;
    contact_type = Contact_type;
    status = Status;
	w_x = Cvector3(-1,1,0) * length / 2.0;
	w_y = Cvector3(-1,1,0) * width / 2.0;
}

void Contact::initialize_contact()
{
    name = (Contact_Name)-1;
    body = 0;
    offset = zero_v3;
    contact_type = CT_FULL;
    status = PS_NO_CONTROL;
    used_for_odometry = false;

    Geom_Point * gp[3] = {&p, &init_p, &ref_p};
    for(int i=0;i<3;i++)
    {
        gp[i]->pos = zero_v3;
        gp[i]->vel = zero_v3;
        gp[i]->acc = zero_v3;
    }

	Geom_Rot * go[3] = {&o, &init_o, &ref_o};
    for(int i=0;i<3;i++)
    {
        go[i]->pos = zero_quat;
        go[i]->vel = zero_v3;
        go[i]->acc = zero_v3;
    }

    Dyn_Point * dp[2] = {&T, &R};
    for(int i=0;i<2;i++)
    {
        dp[i]->acc_ref = zero_v3;
        dp[i]->J = Cmatrix(1,1).setZero();
        dp[i]->dJdq = zero_v3;
        dp[i]->F_est = zero_v3;
        dp[i]->F_ref = zero_v3;
        dp[i]->F_sens = zero_v3;

        dp[i]->F_cost = zero_v3;
        dp[i]->K = Cvector3(20, 3, 1);
        dp[i]->slack_cost = zero_v3;
    }

    cop = zero_v3;
    n1 = Cvector3(1,0,0);
    n2 = Cvector3(0,1,0);
    n3 = Cvector3(0,0,1);
    w_x = Cvector3(-100,100,0);
    w_y = Cvector3(-100,100,0);
    mu = 1;
    muR = 1;
}

Contact_Manager::Contact_Manager()
{
    M = NULL;
    C.resize(0);
	ifCoM = false;
	base = zero_v3;
	dbase = zero_v3;
	heading = 0;
}

Contact_Manager::~Contact_Manager()
{
    C.clear();
}

void Contact_Manager::initialize(Model * model)
{
    M = model;
    C.resize(NUM_Contact);
	ifCoM = false;

	// this combination of contact points is fixed since the jacobian in the IK solver has hard-coded sparsity pattern
	C[CN_CM].init(CN_CM, body_base,   offset_base,   CT_FULL, 		PS_FLOATING,  0              , 0             );
	C[CN_LF].init(CN_LF, body_l_foot, offset_l_foot, CT_FULL, 		PS_CONTACTED, foot_length*0.8, foot_width*0.8);
	C[CN_RF].init(CN_RF, body_r_foot, offset_r_foot, CT_FULL, 		PS_CONTACTED, foot_length*0.8, foot_width*0.8);
	C[CN_LH].init(CN_LH, body_l_hand, offset_l_hand, CT_FULL, 		PS_FLOATING,  hand_length*0.8, hand_width*0.8);
	C[CN_RH].init(CN_RH, body_r_hand, offset_r_hand, CT_FULL, 		PS_FLOATING,  hand_length*0.8, hand_width*0.8);
	C[CN_TO].init(CN_TO, body_torso , offset_torso , CT_ROTATION, 	PS_FLOATING,  0              , 0             );
	C[CN_HD].init(CN_HD, body_head  , offset_head  , CT_ROTATION, 	PS_FLOATING,  0              , 0             );
}

void Contact_Manager::update_kinematics()
{
	// assumes model.set_state is called before
    for(unsigned int i=0;i<C.size();i++)
    {
        C[i].p.pos  = M->get_pos(C[i].body,C[i].offset);
        C[i].p.vel  = M->get_vel(C[i].body,C[i].offset);
        C[i].T.J    = M->get_jacob(C[i].body, C[i].offset, CT_TRANSLATION);
        C[i].o.pos  = M->get_orient_quat(C[i].body);
        C[i].o.vel  = M->get_angvel(C[i].body);
        C[i].R.J    = M->get_jacob(C[i].body, C[i].offset, CT_ROTATION);
        if(i == 0 && ifCoM)
        {
            C[i].p.pos  = M->get_cm();
            C[i].p.vel  = M->get_cm_v();
            C[i].T.J    = M->get_cm_J();
        }
        // a rough estimation of the contact normal.
        C[i].n1 = M->get_pos(C[i].body,Cvector3(1,0,0)) - M->get_pos(C[i].body,zero_v3);
        C[i].n2 = M->get_pos(C[i].body,Cvector3(0,1,0)) - M->get_pos(C[i].body,zero_v3);
        C[i].n3 = M->get_pos(C[i].body,Cvector3(0,0,1)) - M->get_pos(C[i].body,zero_v3);
    }
	Cvector zero_acc = Cvector::Zero(AIR_N_U);
    M->set_acc(zero_acc);
    for(unsigned int i=0;i<C.size();i++)
    {
        C[i].T.dJdq = M->get_acc(C[i].body,C[i].offset);
        C[i].R.dJdq = M->get_angacc(C[i].body);
        if(i==0 && ifCoM)
            C[i].T.dJdq = M->get_cm_a();
    }
}

void Contact_Manager::update_dynamics()
{
	// assumes model.set_state and model.set_acc are called before
    for(unsigned int i=0;i<C.size();i++)
    {
        C[i].p.acc  = M->get_acc(C[i].body,C[i].offset);
        C[i].o.acc  = M->get_angacc(C[i].body);
        if(i == 0 && ifCoM)
            C[i].p.acc  = M->get_cm_a();
    }
}

void Contact_Manager::save_initial_contacts()
{
    for(unsigned int i=0;i<C.size();i++)
    {
        C[i].init_p.pos = M->get_pos(C[i].body,C[i].offset);
        C[i].init_o.pos = M->get_orient_quat(C[i].body);
        C[i].init_p.vel = zero_v3;
        C[i].init_p.acc = zero_v3;
        C[i].init_o.vel = zero_v3;
        C[i].init_o.acc = zero_v3;
        if(i == 0 && ifCoM)
            C[i].init_p.pos = M->get_cm();
    }
}

unsigned int Contact_Manager::size()
{
    return C.size();
}

void Contact_Manager::tasks(Cmatrix &J, Cvector &dJdq, Cvector &acc, Cvector &slack)
{
    J = Cmatrix::Zero(N_TASK,AIR_N_U);
    dJdq = Cvector::Zero(N_TASK);
    acc = Cvector::Zero(N_TASK);
    slack = Cvector::Zero(N_TASK);
    int beg_index = 0;
    for(unsigned int i=0;i<C.size();i++)
    {
		if(C[i].contact_type==CT_FULL || C[i].contact_type==CT_TRANSLATION)
		{
		    J.block(beg_index,0,3,AIR_N_U) 	= C[i].T.J;
		    dJdq.segment(beg_index,3) 		= C[i].T.dJdq;
		    acc.segment(beg_index,3) 		= C[i].T.acc_ref;
		    slack.segment(beg_index,3) 		= C[i].T.slack_cost;
		    beg_index += 3;
		}

		if(C[i].contact_type==CT_FULL || C[i].contact_type==CT_ROTATION)
		{
		    J.block(beg_index,0,3,AIR_N_U) 	= C[i].R.J;
		    dJdq.segment(beg_index,3) 		= C[i].R.dJdq;
		    acc.segment(beg_index,3) 		= C[i].R.acc_ref;
		    slack.segment(beg_index,3) 		= C[i].R.slack_cost;
		    beg_index += 3;
		}
    }
}

Cvector Contact_Manager::get_mult()
{
    Cvector mult = Cvector::Zero(AIR_N_U);
    for(unsigned int i=1;i<C.size();i++)
    {
		if(C[i].contact_type==CT_FULL || C[i].contact_type==CT_TRANSLATION)
        	mult += C[i].T.J.transpose() * C[i].T.F_est;
		if(C[i].contact_type==CT_FULL || C[i].contact_type==CT_ROTATION)
        	mult += C[i].R.J.transpose() * C[i].R.F_est;
    }
    return mult;
}

Cvector Contact_Manager::get_virtual_mult()
{
    Cvector mult = Cvector::Zero(AIR_N_U);
    for(unsigned int i=0;i<C.size();i++)
    {
		if(C[i].contact_type==CT_FULL || C[i].contact_type==CT_TRANSLATION)
	        mult += C[i].T.J.transpose() * C[i].T.F_ref;
		if(C[i].contact_type==CT_FULL || C[i].contact_type==CT_ROTATION)
	        mult += C[i].R.J.transpose() * C[i].R.F_ref;
    }
    return mult;
}

void Contact_Manager::control_position(Contact_Name index, Geom_Point ref)
{
    C[index].T.acc_ref =  (ref.pos - C[index].p.pos) * C[index].T.K[0] +
                          (ref.vel - C[index].p.vel) * C[index].T.K[1] +
                           ref.acc * C[index].T.K[2];
}

void Contact_Manager::control_orientation(Contact_Name index, Geom_Rot ref)
{
    // r : ref frame
    // e : end effector frame
    // i : inertial frame
    // t : theta, rotation matrix
    // w : omega, angular velocity
    // a : angular velocity

    Geom_Rot act = C[index].o;

    Cmatrix  itr = quat2dc(ref.pos);
    Cvector3 iwr = ref.vel;
    Cvector3 iar = ref.acc;

    Cmatrix  ite = quat2dc(act.pos);
    Cvector3 iwe = act.vel;

    Cmatrix rte = itr.inverse() * ite;
    Cvector3 rwe = itr.inverse() * (iwe - iwr);
    Cvector3 rae = dc2ang(rte) * (-C[index].R.K[0]) + rwe * (-C[index].R.K[1]);
    Cvector3 rw = itr.inverse() * iwr;

    Cvector3 iae =  itr * skew_symmetric(rw) * rwe +
                    itr * rae +
                    iar;

    C[index].R.acc_ref = iae;
}

void Contact_Manager::control(Contact_Name index, Geom_Point ref_pos, Geom_Rot ref_ori, bool if_force)
{
	if(if_force)
	{
		if(C[index].contact_type==CT_FULL || C[index].contact_type==CT_TRANSLATION)
			C[index].T.F_ref = C[index].T.K[0] * (C[index].ref_p.pos - C[index].p.pos)
							 + C[index].T.K[1] * (C[index].ref_p.vel - C[index].p.vel);
		if(C[index].contact_type==CT_FULL || C[index].contact_type==CT_ROTATION)
			C[index].R.F_ref = C[index].R.K[0] * quat_log(quat_sub(C[index].ref_o.pos, C[index].o.pos)).segment(0,3)
							 + C[index].R.K[1] * (C[index].ref_o.vel - C[index].o.vel);
	}
	else
	{
		if(C[index].contact_type==CT_FULL || C[index].contact_type==CT_TRANSLATION)
		    control_position(index, ref_pos);
		if(C[index].contact_type==CT_FULL || C[index].contact_type==CT_ROTATION)
    		control_orientation(index, ref_ori);
	}
}

void print_cvector(Cvector x, double precision)
{
	for(int i=0; i<x.size();i++)
		cout << fixed << setprecision( precision ) << std::setw( 11 ) << x[i];
	cout << endl;
}

void Contact_Manager::print_acc_ref()
{
    printf("----- Contact Ref. Accelerations-------------------------- \n");
    for(unsigned int i=0;i<C.size();i++)
	{
		cout << names[i];
		print_cvector(vectorbig(C[i].T.acc_ref, C[i].R.acc_ref), 2);
	}
}

void Contact_Manager::print_forces()
{
    printf("----- Contact forces -------------------------------------- \n");
    for(unsigned int i=0;i<C.size();i++)
	{
		cout << names[i];
		print_cvector(vectorbig(C[i].T.F_sens, C[i].R.F_sens), 2);
	}
}

void Contact_Manager::print_IK_errors()
{
    printf("----- Contact IK position errors -------------------------- \n");
    for(unsigned int i=0;i<C.size();i++)
	{
		cout << names[i];
		print_cvector(vectorbig(C[i].ref_p.pos- C[i].p.pos, quat_sub(C[i].ref_o.pos, C[i].o.pos).segment(0,3)), 3);
	}
}

void Contact_Manager::print_ID_errors()
{
    printf("----- Contact ID acceleration errors ---------------------- \n");
    for(unsigned int i=0;i<C.size();i++)
	{
		cout << names[i];
		print_cvector(vectorbig(C[i].T.acc_ref-C[i].p.acc, C[i].R.acc_ref-C[i].o.acc),2);
	}
}
