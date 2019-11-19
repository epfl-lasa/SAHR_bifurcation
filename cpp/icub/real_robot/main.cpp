#include <string>
#include <cstdio>
#include <iostream>
#include <fstream>
#include <cmath>
#include <signal.h>
#include <unistd.h>
#include <termios.h>

#include "wrapper.h"
#include "IK.h"
#include "ID.h"
using namespace std;

enum state {BALANCE=0,  PICK_APPROACH, PICK, PICK_STAND, DROP_APPROACH, DROP, DROP_STAND, WALK_START, WALK, WALK_STOP};
enum trigger {NONE=0, GO, HALT, TAKE, RELEASE};

int khbit()
{
    struct timeval tv;
    fd_set fds;
    tv.tv_sec = 0;
    tv.tv_usec = 0;
    FD_ZERO(&fds);
    FD_SET(STDIN_FILENO, &fds);
    select(STDIN_FILENO+1, &fds, NULL, NULL, &tv);
    return FD_ISSET(STDIN_FILENO, &fds);
}

void nonblock(int state)
{
    struct termios ttystate;
    tcgetattr(STDIN_FILENO, &ttystate);

    if ( state == 1)
    {
        ttystate.c_lflag &= (~ICANON & ~ECHO); //Not display character
        ttystate.c_cc[VMIN] = 1;
    }
    else if (state == 0)
    {
        ttystate.c_lflag |= ICANON;
        ttystate.c_lflag |= ECHO;
    }
    tcsetattr(STDIN_FILENO, TCSANOW, &ttystate);
}

bool keyState(char key)
{
    bool pressed = false;
    int i = khbit(); //Alow to read from terminal
    if (i != 0)
    {
        char c = fgetc(stdin);
        if (c == key)
        {
            pressed = true;
        }
        else
        {
            pressed = false;
        }
    }

    return pressed;
}

bool exit_sim = false;
void my_handler(int s)
{
   cout << "Exit Program Now ... " << endl;
   exit_sim = true;
}

void loadData(wrapper &W, Contact_Manager &points, Cvector &pos,  Cvector &vel,  Cvector &acc,  Cvector &tau)
{
	pos.segment(6,AIR_N_U-6)	= W.sens_pos / 180.0 * M_PI;
	vel.segment(6,AIR_N_U-6)	= W.sens_vel / 180.0 * M_PI;
	acc.segment(6,AIR_N_U-6)	= W.sens_acc / 180.0 * M_PI;
	tau.segment(6,AIR_N_U-6)	= W.sens_tau;

	Cvector q				= dc2quat(W.R);
	pos.segment(3,3)			= q.segment(0,3);
	pos[AIR_N_U]				= q[3];
	vel.segment(3,3)			= W.angvel / 180.0 * M_PI;
	acc.segment(3,3)			= W.linacc;
	for(int i=1; i<5; i++)
	{
		points[i].T.F_sens = W.FTsensor[i-1][0];
		points[i].R.F_sens = W.FTsensor[i-1][1];
	}

	// WARNING: make sure to cancel IMU yaw on the robot
	points.M->set_state(0, pos, vel);
	Cvector3 base  = points.M->get_pos(points[CN_LF].body, points[CN_LF].offset) / 2.0
			+ points.M->get_pos(points[CN_RF].body, points[CN_RF].offset) / 2.0;
	Cvector3 dbase = points.M->get_vel(points[CN_LF].body, points[CN_LF].offset) / 2.0
			+ points.M->get_vel(points[CN_RF].body, points[CN_RF].offset) / 2.0;
	pos.segment(0,3) -= base;
	vel.segment(0,3) -= dbase;

	points.M->set_state(0, pos, vel);
	points.update_kinematics();
}

/* Index reference and a natural posture of icub           */
/* global positions                                        */
/* pelvis pos:     pos[0] pos[1] pos[2]                    */
/* pelvis rot:     pos[3] pos[4] pos[5] pos[38]            */
/*                 // right      // left                   */
/* head                   pos[11]                          */
/* neck roll              pos[10]                          */
/* neck pitch             pos[9]                           */
/* shoulder pitch  pos[31]       pos[24]                   */
/* shoulder roll   pos[32]       pos[25]                   */
/* shoulder yaw    pos[33]       pos[26]                   */
/* elbow           pos[34]       pos[27]                   */
/* forearm yaw     pos[35]       pos[28]                   */
/* forearm roll    pos[36]       pos[29]                   */
/* forearm pitch   pos[37]       pos[30]                   */
/* torso pitch            pos[6]                           */
/* torso roll             pos[7]                           */
/* torso yaw              pos[8]                           */
/* hip pitch       pos[18]      pos[12]                    */
/* hip roll        pos[19]       pos[13]                   */
/* hip yaw         pos[20]       pos[14]                   */
/* knee            pos[21]       pos[15]                   */
/* ankle pitch     pos[22]       pos[16]                   */
/* ankle roll      pos[23]       pos[17]                   */

Cvector4 projection_gains(double t, double T)
{
	// mass = 27.6, height = 1.05, iCub Robot
	double time = t/T;
	if (std::abs(T-0.4)<0.1)
	{
		// 0 <= time <= 1 : phase time
		// freq = 2.5 & log10(gain) = 1.1
		double g1[4] = {
			exp(-std::pow(time/0.595639,1.0)),
			exp(-std::pow(time/0.434266,1.0)),
			exp(-std::pow(time/0.458892,1.0)),
			exp(-std::pow(time/0.636159,1.0)),
		};
		 double g2[4] = {
			exp(-std::pow((1.0-time)/0.1671792 ,2.0)),
			exp(-std::pow((1.0-time)/0.2202462 ,2.0)),
			exp(-std::pow((1.0-time)/0.5009862 ,2.0)),
			exp(-std::pow((1.0-time)/0.1700142 ,2.0)),
		};
		 double K[4] = {
			0.228948 * g1[0] + 0.955843 * g2[0],
			2.538919 * g1[1] + -1.246919 * g2[1],
			0.033176 * g1[2] + 0.010854 * g2[2],
			0.640404 * g1[3] + -0.160717 * g2[3],
		};
		Cvector4 KK(K);
		return KK;
	}
	return Cvector::Zero(4);
}

double deadzone(double e,double dz)
{
    return e-atan(e*M_PI/2.0/dz)/M_PI*2.0*dz;
}

#define FILT_WIN 5
class median_filter
{
public:
    int n;
    Cvector x[FILT_WIN];
    int index;
    bool first_input;
    void init(int N)
    {
        index = FILT_WIN-1;
        first_input = true;
        n = N;
        for(int i=0;i<FILT_WIN;i++)
            x[i] = Cvector::Zero(n);
    }
    median_filter()
    {
        init(0);
    }
    median_filter(int N)
    {
        init(N);
    }
    Cvector update(Cvector X)
    {
        assert(X.size()==n);
        if(first_input)
        {
            for(int i=0;i<FILT_WIN;i++)
                x[i] = X;
            first_input = false;
        }
        else
        {
            index = (index+1)%FILT_WIN;
            x[index] = X;
        }
        Cvector Y = X;
        for(int i=0;i<n;i++)
        {
            Cvector window(FILT_WIN+1);
            for(int j=0;j<FILT_WIN;j++)
                window[j] = x[j][i];
            std::sort(&window[0], &window[FILT_WIN]);
            Y[i] = window[std::ceil(FILT_WIN/2)];
        }

        return Y;
    }
};

void rePID(wrapper &W, bool walk)
{
	// set gains relative to factory settigns
	if(walk)
	{
		// hips
		W.setPidJoint(13-6,1,1,0);
		W.setPidJoint(19-6,1,1,0);
		W.setPidJoint(12-6,1,1,0);
		W.setPidJoint(18-6,1,1,0);
		W.setPidJoint(14-6,1,1,0);
		W.setPidJoint(20-6,1,1,0);

		// knees
		W.setPidJoint(15-6,1,1,0);
		W.setPidJoint(21-6,1,1,0);

		// ankles
		W.setPidJoint(16-6,1,1,0);
		W.setPidJoint(22-6,1,1,0);
		W.setPidJoint(17-6,1,1,0);
		W.setPidJoint(23-6,1,1,0);
	}
	else
	{
		// hips
		W.setPidJoint(13-6,1,1,1);
		W.setPidJoint(19-6,1,1,1);
		W.setPidJoint(12-6,1,1,0);
		W.setPidJoint(18-6,1,1,0);
		W.setPidJoint(14-6,1,1,1);
		W.setPidJoint(20-6,1,1,1);

		// knees
		W.setPidJoint(15-6,1,1,0);
		W.setPidJoint(21-6,1,1,0);

		// ankles
		W.setPidJoint(16-6,1,1,0);
		W.setPidJoint(22-6,1,1,0);
		W.setPidJoint(17-6,1,1,1);
		W.setPidJoint(23-6,1,1,1);
	}
}

int main(int argc, char *argv[])
{
	// setup existing condition
	struct sigaction sigIntHandler;
	sigIntHandler.sa_handler = my_handler;
	sigemptyset(&sigIntHandler.sa_mask);
	sigIntHandler.sa_flags = 0;
	sigaction(SIGINT, &sigIntHandler, NULL);

	// logger
	std::ofstream OutRecord;
	OutRecord.open("/tmp/log.txt");

	// YARP wrapper
	wrapper W;
	if(W.checkRobot(argc, argv))
		return 1;
	W.initialize();

	// definitions
	Model model;
	model.init();
	IKPARAM ikparam;
	ikparam.num_iter = 10;


	// joint variables
	Cvector ref_pos  = Cvector::Zero(AIR_N_U+1); ref_pos[AIR_N_U] = 1;
	Cvector ref_vel  = Cvector::Zero(AIR_N_U);
	Cvector ref_acc  = Cvector::Zero(AIR_N_U);
	Cvector ref_tau  = Cvector::Zero(AIR_N_U);
	Cvector sens_pos = Cvector::Zero(AIR_N_U+1); sens_pos[AIR_N_U] = 1;
	Cvector sens_vel = Cvector::Zero(AIR_N_U);
	Cvector sens_acc = Cvector::Zero(AIR_N_U);
	Cvector sens_tau = Cvector::Zero(AIR_N_U);
	Cvector init_pos = Cvector::Zero(AIR_N_U+1); init_pos[AIR_N_U] = 1;
	Cvector next_pos = Cvector::Zero(AIR_N_U+1); next_pos[AIR_N_U] = 1;
	Cvector freezeIK = Cvector::Zero(AIR_N_U);
	Cvector mode 	 = Cvector::Zero(AIR_N_U);
	Cvector kp 	 = Cvector::Zero(AIR_N_U);
	Cvector kd 	 = Cvector::Zero(AIR_N_U);
	Cvector ki 	 = Cvector::Zero(AIR_N_U);

	// frozen joints
	for(int i=9;i<=11;i++)
		freezeIK[i] = 1;
	for(int i=24;i<38;i++)
		mode[i] = 1;
	for(int i=12;i<24;i++)
		mode[i] = 1;
	for(int i=6;i<9;i++)
		mode[i] = 1;

	// contact definition
	Contact_Manager points;
	points.initialize(&model, 5);
	points[CN_CM].init(CN_CM, body_base,   offset_base,   CT_FULL, PS_FLOATING,  0              , 0             );
	points[CN_LF].init(CN_LF, body_l_foot, offset_l_foot, CT_FULL, PS_CONTACTED, foot_length*0.8, foot_width*0.8);
	points[CN_RF].init(CN_RF, body_r_foot, offset_r_foot, CT_FULL, PS_CONTACTED, foot_length*0.8, foot_width*0.8);
	points[CN_LH].init(CN_LH, body_l_hand, offset_l_hand, CT_FULL, PS_FLOATING,  hand_length*0.8, hand_width*0.8);
	points[CN_RH].init(CN_RH, body_r_hand, offset_r_hand, CT_FULL, PS_FLOATING,  hand_length*0.8, hand_width*0.8);

	// control CoM instead of pelvis
	points.ifCoM = true; //WARNING

	// default hand orientations
	Cvector4 pre_rot = ang2quat(Cvector3(0,-M_PI/2,0));

	// task flexibility IK
	points[CN_CM].T.slack_cost = Cvector3(100,100,0.01);
	points[CN_CM].R.slack_cost = Cvector3(0.01,0.01,0.01);
	points[CN_LF].T.slack_cost = Cvector3(1,1,1) * 100;
	points[CN_LF].R.slack_cost = Cvector3(1,1,1) * 100;
	points[CN_RF].T.slack_cost = Cvector3(1,1,1) * 100;
	points[CN_RF].R.slack_cost = Cvector3(1,1,1) * 100;
	points[CN_LH].R.slack_cost = Cvector3(1,1,1) * 1;
	points[CN_LH].T.slack_cost = Cvector3(1,1,1) * 1;
	points[CN_RH].R.slack_cost = Cvector3(1,1,1) * 1;
	points[CN_RH].T.slack_cost = Cvector3(1,1,1) * 1;

	// read data
	yarp::os::Time::delay(1);
	W.readSensors();
	loadData(W, points, init_pos,  sens_vel,  sens_acc,  sens_tau);
	init_pos.segment(6,3) *= 0;  //WARNING, this removes the redundancy (warm start disabled)

	// set position/torque control modes
	W.initializeJoint(mode.segment(6,AIR_N_U-6));
	rePID(W, false);


	// object properties
	double obj_width = 0.3;
	double obj_length = 0.8 - 0.15;
	Cvector3 des_obj_pos(0.2, 0.0, 0.65);
	Cvector3 des_obj_rot(0.0, 0.0, 0.0);
	Cvector3 next_obj_pos = des_obj_pos;
	Cvector3 next_obj_rot = des_obj_rot;
	bool grasp = false;

	// robot properties
	Cvector3 des_com_pos(-0.02,0.0,0.53);
	Cvector3 next_com_pos = des_com_pos;
	points[CN_CM].ref_p.pos = des_com_pos;

	// walking stuff
	double Tstep = 0.4;
	double walk_start = 0;
	median_filter state_filt;
	state_filt.init(10);

	nonblock(1);

	// start the loop
	state S = BALANCE;
	trigger G = NONE;
	double ref_vx = 0;
	double ref_vy = 0;
	double ref_w = 0;
	while(!exit_sim)
	{
		////////////////////////////////////////////////////////////////////////////////////////////
		double time = W.time;
		static double prev_time = time;
		double dt = max(time - prev_time,0.001);
		prev_time = time;
		double timeStep = 0.002;
		// read sensors
		W.readSensors();
		loadData(W, points, sens_pos,  sens_vel,  sens_acc,  sens_tau);

		// arm force estimation
		static Cvector3 wrench_lh = zero_v3;
		static Cvector3 wrench_rh = zero_v3;
		wrench_lh = wrench_lh * 0.99 + points[CN_LH].R.F_sens * 0.01;
		wrench_rh = wrench_rh * 0.99 + points[CN_RH].R.F_sens * 0.01;

		////////////////////////////////////////////////////////////////////////////////////////////

		if(khbit() != 0 && false)
		{
			char c = fgetc(stdin);
			fflush(stdin);
			switch(c)
			{
				case 'g': G = GO; break;
				case 'h': G = HALT; break;
				case 't': G = TAKE; break;
				case 'r': G = RELEASE; break;
				case 'w': ref_vx += 0.1; break;
				case 's': ref_vx -= 0.1; break;
				case 'a': ref_vy += 0.1; break;
				case 'd': ref_vy -= 0.1; break;
				case 'q': ref_w += 0.05; break;
				case 'e': ref_w -= 0.05; break;
			}
		}

		ref_vx = truncate_command(ref_vx * 0.98, 0.5, -0.5);
		ref_vy = truncate_command(ref_vy * 0.98, 0.5, -0.5);
		ref_w = truncate_command(ref_w * 0.98, 2, -2);

		double eps_CoM = 0.04;
		double eps_obj = 0.001;

		double phase = fmod(time,2.0*Tstep) > Tstep; // 1: left support, right swing
		bool lean = (points[CN_CM].ref_p.pos-next_com_pos).norm()<eps_CoM && phase && fmod(time,Tstep)<=dt*2.0;
		bool stand = (points[CN_CM].ref_p.pos-next_com_pos).norm()<eps_CoM;
		bool reached = (points[CN_LH].p.pos*0.5+points[CN_RH].p.pos*0.5-next_obj_pos).norm()<eps_obj;
		bool fbig = wrench_lh.norm()>3.0 && wrench_rh.norm()>3.0;
		bool fsmall = wrench_lh.norm()<3.0 && wrench_rh.norm()<3.0;

		bool rest = fmod(time,Tstep)<=dt*2.0 && std::abs(points[CN_LF].p.pos[0]-points[CN_RF].p.pos[0]) < 0.05;

		//        cout << time << "   " << G << "  " << S << "  |  " << ref_vx << "   " << ref_vy << "   " << ref_w
		//             << "           " << wrench_lh.norm() << "  " <<  wrench_rh.norm() << endl;

		// calculate robot to object relative position
		Cvector RootRot = quat_mul(W.Root.segment(3,4), ang2quat(Cvector3(0,0,M_PI)));
		Cvector3 RootPos = W.Root.segment(0,3);

		Cvector ObjectRot = W.Object.segment(3,4);
		Cvector3 ObjectPos = W.Object.segment(0,3);
		Cvector3 ObjectPos1 = ObjectPos + quat2dc(ObjectRot) * Cvector3(obj_length/2,0,0.0);
		Cvector3 ObjectPos2 = ObjectPos + quat2dc(ObjectRot) * Cvector3(-obj_length/2,0,0.0);
		if((RootPos-ObjectPos1).norm() < (RootPos-ObjectPos2).norm())
		{
			ObjectPos = ObjectPos1;
			ObjectRot = quat_mul(ObjectRot, ang2quat(Cvector3(0,0,M_PI)));;
		}
		else
		{
			ObjectPos = ObjectPos2;
		}
		Cvector relROT = quat_mul(ObjectRot, quat_inverse(RootRot));
		Cvector3 relPOS = quat2dc(RootRot).inverse() * Cvector3(ObjectPos[0]-RootPos[0], ObjectPos[1]-RootPos[1], ObjectPos[2]);

		switch(S)
		{
			case BALANCE:
				if(G==GO)
				{
					G = NONE;
					S = WALK_START;
					next_com_pos = des_com_pos + Cvector3(0,0.02,0);
					ref_pos.segment(6,3) *= 0;
				}
				if(G==TAKE)
				{
					G = NONE;
					S = PICK_APPROACH;
					next_obj_pos = relPOS;
					next_obj_rot = quat2ang(relROT);
				}
				if(G==RELEASE)
				{
					G = NONE;
					S = DROP_APPROACH;
					next_obj_pos = des_obj_pos;
					next_obj_rot = des_obj_rot;
				}
			break;
			case PICK_APPROACH:
				if(reached || G==HALT)
				{
					G = NONE;
					S = PICK;
					grasp = true;
				}
				if(G==RELEASE)
				{
					G = NONE;
					S = DROP_APPROACH;
					next_obj_pos = des_obj_pos;
					next_obj_rot = des_obj_rot;
				}
			break;
			case PICK:
				if(fbig)
				{
					G = NONE;
					S = PICK_STAND;
					next_obj_pos = des_obj_pos;
					next_obj_rot = des_obj_rot;
				}
			break;
			case PICK_STAND:
				if(stand)
				{
					G = NONE;
					S = BALANCE;
				}
			break;
			case DROP_APPROACH:
				if(reached)
				{
					G = NONE;
					S = DROP;
					grasp = false;
				}
			break;
			case DROP:
				if(fsmall)
				{
					G = NONE;
					S = DROP_STAND;
					next_obj_pos = des_obj_pos;
					next_obj_rot = des_obj_rot;
				}
			break;
			case DROP_STAND:
				if(stand)
				{
					G = NONE;
					S = BALANCE;
				}
			break;
			case WALK_START:
				if(lean)
				{
					G = NONE;
					S = WALK;
					walk_start = time;
					rePID(W, true);
				}
			break;
			case WALK:
				if(G==HALT && rest)
				{
					G = NONE;
					S = WALK_STOP;
				}
			break;
			case WALK_STOP:
				if(stand)
				{
					G = NONE;
					S = BALANCE;
					rePID(W, false);
				}
			break;
		}


		double ramp = max(0.0, time - walk_start);
		//        ref_vx = min(1.0, ramp/10.0) * 0;
		//        ref_w = - ramp/1.0 * 0;

		////////////////////////////////////////////////////////////////////////////////////////////
		double r = min(1.0, ramp/10.0);
		double v = std::abs(ref_vx);
		// feet tasks
		double liftL = S==WALK ? max(0.0, sin(M_PI*time/Tstep)) : 0;
		double liftR = S==WALK ? max(0.0, sin(M_PI*time/Tstep+M_PI)) : 0;

		//WARNING
		liftL = max(0.0, sin(M_PI*time/Tstep)) * 0;
		liftR = max(0.0, sin(M_PI*time/Tstep+M_PI)) * 0;
		
		points[CN_LF].ref_p.pos = Cvector3(0, 0.045+0.05*(1.0-v), liftL * 0.02*(1+v));
		points[CN_RF].ref_p.pos = Cvector3(0,-0.045-0.05*(1.0-v), liftR * 0.02*(1+v));

		// base tasks
		if(S==WALK)
			next_com_pos = des_com_pos + Cvector3(-0.0125 + 0.03*ref_vx, 0.03*ref_vy, 0);
		points[CN_CM].ref_p.pos = points[CN_CM].ref_p.pos * 0.99 + next_com_pos * 0.01;
		points[CN_CM].ref_o.pos = zero_quat;


		// arm tasks
		static Cvector3 pos = next_obj_pos;
		static Cvector3 rot = next_obj_rot;
		pos = 0.995 * pos + 0.005 * next_obj_pos;
		rot = 0.995 * rot + 0.005 * next_obj_rot;
		points[CN_LH].ref_p.pos = pos + ang2dc(rot) * Cvector3(0,obj_width/2.0,0);
		points[CN_RH].ref_p.pos = pos + ang2dc(rot) * Cvector3(0,-obj_width/2.0,0);
		points[CN_LH].ref_o.pos = quat_mul(ang2quat(rot), pre_rot);
		points[CN_RH].ref_o.pos = quat_mul(ang2quat(rot), pre_rot);
		// arm compliant control
		points[CN_LH].T.K = -Cvector3(30, 2, 1)*2;
		points[CN_LH].R.K = -Cvector3(3, 0.5, 1);
		points[CN_RH].T.K = -Cvector3(30, 2, 1)*2;
		points[CN_RH].R.K = -Cvector3(3, 0.5, 1);
		points.control(CN_LH, points[CN_LH].ref_p, points[CN_LH].ref_o, true);
		points.control(CN_RH, points[CN_RH].ref_p, points[CN_RH].ref_o, true);
		// grasping forces
		if(grasp)
		{
			points[CN_LH].T.F_ref -= (pos - points[CN_LH].p.pos) * 200;
			points[CN_RH].T.F_ref -= (pos - points[CN_RH].p.pos) * 200;
		}
		Cvector h = model.get_frcmat() * (-1) - points.get_virtual_mult();
		ref_tau.segment(24,14) = h.segment(24,14);
		////////////////////////////////////////////////////////////////////////////////////////////


		// feet force estimation
		static double fl = 0;
		static double fr = 0;
		fl = fl * 0.8 + max(0.0, -points[CN_LF].T.F_sens[2]) * 0.2;
		fr = fr * 0.8 + max(0.0, -points[CN_RF].T.F_sens[2]) * 0.2;
		double al = fl/(fl+fr);
		double ar = fr/(fl+fr);

		//WARNING
		/*OutRecord << 	time << " " << 
				liftL << "    " << liftR << "  " << 
				al << "   " << ar << "   " << 
				points[CN_LF].T.F_sens[2]/270.0 << "  " << points[CN_RF].T.F_sens[2]/270.0 << 
				endl;*/


		if(S==WALK)
		{
			// foot orientation correction
			points[CN_LF].ref_o.pos = ang2quat(-quat2ang(points[CN_LF].o.pos)*1);
			points[CN_RF].ref_o.pos = ang2quat(-quat2ang(points[CN_RF].o.pos)*1);

			double heading = -ref_w - quat2ang(relROT)[2];
			points[CN_LF].ref_o.pos = quat_mul(ang2quat(Cvector3(0,0,-heading/3.0 * ar)), points[CN_LF].ref_o.pos);
			points[CN_CM].ref_o.pos = quat_mul(ang2quat(Cvector3(0,0,-heading/3.0 * 0 )), points[CN_CM].ref_o.pos);
			points[CN_RF].ref_o.pos = quat_mul(ang2quat(Cvector3(0,0,-heading/3.0 * al)), points[CN_RF].ref_o.pos);
		}
		
		//WARNING
		if(time>20)
		{
			//points[CN_LF].ref_o.pos = ang2quat(-quat2ang(points[CN_LF].o.pos)*1);
			//points[CN_RF].ref_o.pos = ang2quat(-quat2ang(points[CN_RF].o.pos)*1);
		}

		//        // this works in static condition
		//        Cmatrix Forces(AIR_N_BODY,3);
		//        Cmatrix Torques(AIR_N_BODY,3);
		//        model.get_reac(Forces, Torques);
		//        cout << time << "        " << points[CN_LH].T.F_sens.transpose()
		//             << "   " << points[CN_LH].R.F_sens.transpose() << "    -------   "
		//             << Forces.block(26,0,1,3) << "   " << Torques.block(26,0,1,3)
		//             << endl;



		/////////////////////////////////////////////////////////////////////////////////////////////////
		// whole-body IK

		if(S==WALK || S==WALK_START || S==WALK_STOP)
		{
			points[CN_CM].T.slack_cost = Cvector3(100,100,100);
			points[CN_CM].R.slack_cost = Cvector3(100,100,100);
			for(int i=6;i<=8;i++)
				freezeIK[i] = 1;
		}
		else
		{
			points[CN_CM].T.slack_cost = Cvector3(100,100,0.01);
			points[CN_CM].R.slack_cost = Cvector3(0.01,0.01,0.01);
			for(int i=6;i<=8;i++)
				freezeIK[i] = 0;
		}

		// WARNING
		points[CN_CM].T.slack_cost = Cvector3(100,100,100);
		points[CN_CM].R.slack_cost = Cvector3(100,100,100);
		for(int i=6;i<=8;i++)
				freezeIK[i] = 1;


		ref_pos = init_pos;
		double IK_time = IK(points, ikparam, ref_pos, freezeIK);
		model.set_state(0, sens_pos, sens_vel);
		points.update_kinematics();

		if(S==WALK)
		{
			// extra leg lift by hip roll
			double pelvis_roll = sin(M_PI*time/Tstep) * min(1.0, ramp/5.0) * 0.1;
			ref_pos[7] = pelvis_roll;
			ref_pos[13] += -pelvis_roll;
			ref_pos[19] += pelvis_roll;

			// Pelvis to CoM shift filter
			static double shift = 0;
			shift = 0.99 * shift + 0.01 * (points[CN_CM].ref_p.pos[0]-ref_pos[0]);

			// Cartesian velocity estimation by velocity sensors
			Cvector lp3_state = Cvector::Zero(10);
			lp3_state[0] = time;
			lp3_state.segment(1,3) = points[CN_LF].p.pos;
			lp3_state.segment(4,3) = sens_pos.segment(0,3) + Cvector3(shift,0,0);
			lp3_state.segment(7,3) = points[CN_RF].p.pos;
			Cvector lp3_pstate = state_filt.x[(state_filt.index+1)%FILT_WIN];
			state_filt.update(lp3_state);
			Cvector lp3_dstate = (lp3_state - lp3_pstate)/std::max(timeStep,lp3_state[0]-lp3_pstate[0]);

			// projection gains
			double t3lp = fmod(time,Tstep);
			Cvector4 K = projection_gains(t3lp, Tstep);
			double sw = points[CN_LF].ref_p.pos[1] - points[CN_RF].ref_p.pos[1];

			Cvector4 elx(lp3_state[1]-lp3_state[7], lp3_state[4]-lp3_state[7], lp3_dstate[1]-lp3_dstate[7], lp3_dstate[4]-lp3_dstate[7]); //right sup.
			Cvector4 erx(lp3_state[7]-lp3_state[1], lp3_state[4]-lp3_state[1], lp3_dstate[7]-lp3_dstate[1], lp3_dstate[4]-lp3_dstate[1]); //left  sup.
			Cvector4 ex = phase==1 ? erx : elx;

			Cvector4 ely(lp3_state[2]-lp3_state[8]-sw, lp3_state[5]-lp3_state[8]-sw/2.0, lp3_dstate[2]-lp3_dstate[8], lp3_dstate[5]-lp3_dstate[8]);
			Cvector4 ery(lp3_state[8]-lp3_state[2]+sw, lp3_state[5]-lp3_state[2]+sw/2.0, lp3_dstate[8]-lp3_dstate[2], lp3_dstate[5]-lp3_dstate[2]);
			Cvector4 ey = phase==1 ? ery : ely;

			double sig2 = exp(-pow(t3lp/(Tstep/3.0),2.0));
			ex *= 1.0 - sig2;
			ey *= 1.0 - sig2;

			// foot-step position feedbacks wide range
			static double fblx = 0;
			static double fbrx = 0;
			static double fbly = 0;
			static double fbry = 0;
			fblx = phase==1 ? fblx : truncate_command(ex.transpose()*K, 0.2, -0.2);
			fbrx = phase==0 ? fbrx : truncate_command(ex.transpose()*K, 0.2, -0.2);
			fbly = phase==1 ? fbly : truncate_command(ey.transpose()*K, 0.3, -0.3);
			fbry = phase==0 ? fbry : truncate_command(ey.transpose()*K, 0.3, -0.3);

			// collision avoidance
			//            fbly = max(fbly, lp3_state[8]-lp3_state[5] - 0.1*0);
			//            fbry = min(fbry, lp3_state[2]-lp3_state[5] + 0.1*0);

			double z = des_com_pos[2];
			double hip_gain_K = 0.5;
			double hip_gain_D = 0.1;
			int index_l = 6+6;
			int index_r = 6+12;
			double Pitch = W.pitch;
			ref_pos[index_l] = ar * (ref_pos[index_l]+Pitch+fblx/z) +
			al * (-Pitch * hip_gain_K - W.angvel[1] * hip_gain_D + sens_pos[index_l]);
			ref_pos[index_r] = al * (ref_pos[index_r]+Pitch+fbrx/z) +
			ar * (-Pitch * hip_gain_K - W.angvel[1] * hip_gain_D + sens_pos[index_r]);

			/*double Roll = W.roll - pelvis_roll;
			index_l += 1;
			index_r += 1;
			ref_pos[index_l] = ar * (ref_pos[index_l]-Roll+fbly/z*0.7) +
			al * (Roll * hip_gain_K + sens_pos[index_l]);
			ref_pos[index_r] = al * (ref_pos[index_r]+Roll-fbry/z*0.7) +
			ar * (-Roll * hip_gain_K + sens_pos[index_r]);*/

			//		   OutRecord << time << " " << fblx << "    " << ex.transpose() << "  " << al << "   " << ar << endl;
			//		   OutRecord << time << " " << fbly << "    " << ey.transpose() << "  " << al << "   " << ar << endl;
		}


		cout << time << endl;
		if(time>10)
		{
			cout << W.roll << "     " << W.pitch << "     " << W.yaw << endl;

			// extra leg lift by hip roll
			double pelvis_roll = sin(M_PI*time/Tstep) * 0.2;
			ref_pos[7] = pelvis_roll;
			ref_pos[13] += -pelvis_roll;
			ref_pos[19] += pelvis_roll;

			static double shift = 0;
			shift = 0.99 * shift + 0.01 * (points[CN_CM].ref_p.pos[0]-ref_pos[0]);

			// Cartesian velocity estimation by velocity sensors
			double t3lp = fmod(time,Tstep);
			Cvector lp3_state = Cvector::Zero(10);
			lp3_state[0] = time;
			lp3_state.segment(1,3) = points[CN_LF].p.pos;
			lp3_state.segment(4,3) = sens_pos.segment(0,3) + Cvector3(shift,0,0);
			lp3_state.segment(7,3) = points[CN_RF].p.pos;
			Cvector lp3_pstate = state_filt.x[(state_filt.index+1)%FILT_WIN];
			state_filt.update(lp3_state);
			Cvector lp3_dstate = (lp3_state - lp3_pstate)/std::max(timeStep,lp3_state[0]-lp3_pstate[0]);

			// projection errors
			double sw = points[CN_LF].ref_p.pos[1] - points[CN_RF].ref_p.pos[1];

			Cvector4 elx(lp3_state[1]-lp3_state[7], lp3_state[4]-lp3_state[7], lp3_dstate[1]-lp3_dstate[7], lp3_dstate[4]-lp3_dstate[7]); //right sup.
			Cvector4 erx(lp3_state[7]-lp3_state[1], lp3_state[4]-lp3_state[1], lp3_dstate[7]-lp3_dstate[1], lp3_dstate[4]-lp3_dstate[1]); //left  sup.

			Cvector4 ely(lp3_state[2]-lp3_state[8]-sw, lp3_state[5]-lp3_state[8]-sw/2.0, lp3_dstate[2]-lp3_dstate[8], lp3_dstate[5]-lp3_dstate[8]);
			Cvector4 ery(lp3_state[8]-lp3_state[2]+sw, lp3_state[5]-lp3_state[2]+sw/2.0, lp3_dstate[8]-lp3_dstate[2], lp3_dstate[5]-lp3_dstate[2]);

			OutRecord << time << " " << phase << " " << 
						 elx.transpose() << " " << erx.transpose() << "  " << 
						 ely.transpose() << " " << ery.transpose() << endl;

			elx[0] = deadzone(elx[0]-(0.000), 0.001);
			elx[1] = deadzone(elx[1]-(-0.016), 0.002);
			elx[2] = deadzone(elx[2]-(0.003), 0.022);
			elx[3] = deadzone(elx[3]-(0.003), 0.018);
			erx[0] = deadzone(erx[0]-(-0.001), 0.001);
			erx[1] = deadzone(erx[1]-(-0.018), 0.001);
			erx[2] = deadzone(erx[2]-(0.003), 0.021);
			erx[3] = deadzone(erx[3]-(-0.000), 0.019);
			ely[0] = deadzone(ely[0]-(-0.034), 0.017);
			ely[1] = deadzone(ely[1]-(-0.020), 0.032);
			ely[2] = deadzone(ely[2]-(0.031), 0.216);
			ely[3] = deadzone(ely[3]-(-0.232), 0.147);
			ery[0] = deadzone(ery[0]-(0.042), 0.010);
			ery[1] = deadzone(ery[1]-(0.023), 0.033);
			ery[2] = deadzone(ery[2]-(0.032), 0.178);
			ery[3] = deadzone(ery[3]-(0.266), 0.145);

			Cvector4 ex = phase==1 ? erx : elx;
			Cvector4 ey = phase==1 ? ery : ely;
			double sig2 = exp(-pow(t3lp/(Tstep/3.0),2.0));
			ex *= 1.0 - sig2;
			ey *= 1.0 - sig2;

			// projection gains			
			Cvector4 K = projection_gains(t3lp, Tstep);

			// foot-step position feedbacks wide range
			static double fblx = 0;
			static double fbrx = 0;
			static double fbly = 0;
			static double fbry = 0;
			fblx = phase==1 ? fblx : truncate_command(ex.transpose()*K, 0.2, -0.2);
			fbrx = phase==0 ? fbrx : truncate_command(ex.transpose()*K, 0.2, -0.2);
			fbly = phase==1 ? fbly : truncate_command(ey.transpose()*K, 0.05, -0.05);
			fbry = phase==0 ? fbry : truncate_command(ey.transpose()*K, 0.05, -0.05);

			// definitions
			double z = des_com_pos[2];
			double hip_gain_K = 1;
			double hip_gain_D = 0.1;
			int index_l = 6+6;
			int index_r = 6+12;

			double enable = time>10.0 ? 1.0 : 0.0;
			//enable *= 0;

			double Pitch = W.pitch;
			double Pitch_feedback = Pitch * hip_gain_K + W.angvel[1] * hip_gain_D;
			ref_pos[index_l] = ar * (ref_pos[index_l] + Pitch + fblx/z * enable) + al * (-Pitch_feedback + sens_pos[index_l]);
			ref_pos[index_r] = al * (ref_pos[index_r] + Pitch + fblx/z * enable) + ar * (-Pitch_feedback + sens_pos[index_r]);

			hip_gain_K = 0.5;
			double Roll = W.roll - pelvis_roll;
			double Roll_feedback = Roll * hip_gain_K + W.angvel[0] * hip_gain_D;
			ref_pos[index_l+1] = ar * (ref_pos[index_l+1] - Roll + fbly/z * enable) + al * (Roll_feedback + sens_pos[index_l+1]);
			ref_pos[index_r+1] = al * (ref_pos[index_r+1] + Roll - fbry/z * enable) + ar * (-Roll_feedback + sens_pos[index_r+1]);
		}


		// plot all joint variables
		//points.print_IK_errors();
		//points.print_forces();
		/*cout << setprecision( 3 ) << "IK time:   " << IK_time << endl;
		cout << fixed << endl;
		cout << "  joint     min_pos   ref_pos    sens_pos    max_pos     ref_tau    sens_tau" << endl;
		for(int i=0;i<AIR_N_U;i++)
		{
			cout << std::setw( 5 ) << i << ":"  <<
			setprecision( 0 ) <<std::setw( 11 ) << (i<6 ? -1000 : model.qmin[i-6]) <<
			setprecision( 2 ) <<std::setw( 11 ) << ref_pos[i] * (i<6 ? 1 : 180/M_PI) <<
			setprecision( 2 ) <<std::setw( 11 ) << sens_pos[i] * (i<6 ? 1 : 180/M_PI) <<
			setprecision( 0 ) <<std::setw( 11 ) << (i<6 ? 1000 : model.qmax[i-6]) <<
			setprecision( 2 ) <<std::setw( 11 ) << ref_tau[i] <<
			setprecision( 2 ) <<std::setw( 11 ) << (i<6 ? 0 : sens_tau[i]) <<
			setprecision( 2 ) <<std::setw( 11 ) << (i<6 ? 1000 : model.taumax[i-6]) <<
			endl;
			if(i==6-1 || i==12-1 || i==18-1 || i==24-1 || i==31-1)
				cout << endl;
		}*/

		// send final commands
		Cvector pos_command = init_pos * exp(-time/3.0) + ref_pos * (1.0-exp(-time/3.0));
		Cvector tau_command = (init_pos-sens_pos).segment(0,AIR_N_U) * 10.0 * exp(-time/1.0) + ref_tau * (1.0-exp(-time/1.0));
		W.controlJoint( mode.segment(6,AIR_N_U-6),
				pos_command.segment(6,AIR_N_U-6) / M_PI * 180,
				ref_tau.segment(6,AIR_N_U-6));
	}

	nonblock(0);
	mode = Cvector::Zero(AIR_N_U);
	W.initializeJoint(mode.segment(6,AIR_N_U-6));
	rePID(W, false);
	W.close();
	return 0;
}
