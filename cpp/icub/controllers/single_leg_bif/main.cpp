#include <string>
#include <cstdio>
#include <iostream>
#include <fstream>
#include <cmath>
#include <signal.h>
#include <unistd.h>

#include "wrapper.h"
#include "IK.h"
#include "ID.h"
#include "bifurcation.h"

using namespace std;

bool exit_sim = false;
void my_handler(int s)
{
	cout << "Exit Program Now ... " << endl;
	exit_sim = true;
}

// extra processing of data to estimate floating states
void loadData(wrapper &W, Contact_Manager &points, Cvector &pos,  Cvector &vel,  Cvector &acc,  Cvector &tau)
{
	W.readSensors();

	// load pure data
	pos.segment(6,AIR_N_U-6)	= W.sens_pos / 180.0 * M_PI;
	vel.segment(6,AIR_N_U-6)	= W.sens_vel / 180.0 * M_PI;
	acc.segment(6,AIR_N_U-6)	= W.sens_acc / 180.0 * M_PI;
	tau.segment(6,AIR_N_U-6)	= W.sens_tau;
	Cvector q					= dc2quat(W.R);
	pos.segment(0,3)			= W.Root.segment(0,3);
	pos.segment(3,3)			= q.segment(0,3);
	pos[AIR_N_U]				= q[3];
	vel.segment(3,3)			= W.R.inverse() * W.angvel / 180.0 * M_PI;
	acc.segment(3,3)			= W.linacc;
	for(int i=1; i<5; i++)
	{
		points[i].T.F_sens 		= W.FTsensor[i-1][0];
		points[i].R.F_sens 		= W.FTsensor[i-1][1];
	}

	// update the model
	points.M->set_state(0, pos, vel);

	// bring the mid-foot points to zero and make feet point to +x direction
	points.base 	= points.M->get_pos(points[CN_LF].body, points[CN_LF].offset) * 0.5
					+ points.M->get_pos(points[CN_RF].body, points[CN_RF].offset) * 0.5;
	points.dbase 	= points.M->get_vel(points[CN_LF].body, points[CN_LF].offset) * 0.5
					+ points.M->get_vel(points[CN_RF].body, points[CN_RF].offset) * 0.5;
	points.heading 	= points.M->get_orient_ang(points[CN_LF].body)[2] * 0.5 
					+ points.M->get_orient_ang(points[CN_RF].body)[2] * 0.5;

	// apply transformations
	q 							= quat_mul(ang2quat(Cvector3(0,0,-points.heading)), q);
	pos.segment(0,3) 			-= points.base;
	vel.segment(0,3) 			-= points.dbase;
	pos.segment(3,3)			= q.segment(0,3);
	pos[AIR_N_U]				= q[3];
	acc.segment(3,3)			= ang2dc(Cvector3(0,0,-points.heading)) * W.linacc;

	// update the model
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

int main(int argc, char *argv[])
{
	// setup exiting condition
	struct sigaction sigIntHandler;
	sigIntHandler.sa_handler = my_handler;
	sigemptyset(&sigIntHandler.sa_mask);
	sigIntHandler.sa_flags = 0;
	sigaction(SIGINT, &sigIntHandler, NULL);

	// logger
	std::ofstream OutRecord;
	OutRecord.open("/home/lasapc28/catkin_ws/logs_icub/log.csv");

	// definitions
	Model model;
	model.init();
	IKPARAM ikparam;
	ikparam.num_iter = 10;
	ikparam.damping = 0.1;

	// YARP wrapper
	wrapper W;
	if(W.checkRobot(argc, argv))
		return 1;
	W.initialize();
	W.getJointLimits(&model.qmin[0], &model.qmax[0]);
	// avoid knee over-extension by changing joint limits conservatively
	model.qmax[15-6] = -10; 
	model.qmax[21-6] = -10;

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
	Cvector freezeIK = Cvector::Zero(AIR_N_U);
	Cvector mode 	 = Cvector::Zero(AIR_N_U);

	// contact definition
	Contact_Manager points;
	points.initialize(&model);

	// control CoM instead of pelvis
	points.ifCoM = true;

	// task flexibility in IK, -1 means hard constraint
	// positions
	points[CN_CM].T.slack_cost = Cvector3(-1,-1,1);
	points[CN_LF].T.slack_cost = Cvector3(-1,-1,-1);
	points[CN_RF].T.slack_cost = Cvector3(10,10,10);
	points[CN_LH].T.slack_cost = Cvector3(1,1,1);
	points[CN_RH].T.slack_cost = Cvector3(1,1,1);
	// orientations
	points[CN_CM].R.slack_cost = Cvector3(0.1,0.1,0.1);
	points[CN_TO].R.slack_cost = Cvector3(0.1,0.1,0.1);
	points[CN_HD].R.slack_cost = Cvector3(0.1,0.1,0.1);
	points[CN_LF].R.slack_cost = Cvector3(-1,-1,-1);
	points[CN_RF].R.slack_cost = Cvector3(1,1,1);
	points[CN_LH].R.slack_cost = Cvector3(1,1,1);
	points[CN_RH].R.slack_cost = Cvector3(1,1,1);

	// base tasks
	points[CN_CM].ref_p.pos = Cvector3(0.0,0.0,0.63);
	points[CN_CM].ref_o.pos = zero_quat;
	points[CN_TO].ref_o.pos = zero_quat;
	points[CN_HD].ref_o.pos = zero_quat;

	// feet tasks
	points[CN_LF].ref_p.pos = Cvector3(0, 0.065, 0);
	points[CN_RF].ref_p.pos = Cvector3(0,-0.065, 0);
	points[CN_LF].ref_o.pos = zero_quat;
	points[CN_RF].ref_o.pos = zero_quat;

	// arm tasks
	points[CN_LH].ref_o.pos = ang2quat(Cvector3(0,-M_PI/2,0));
	points[CN_RH].ref_o.pos = ang2quat(Cvector3(0,-M_PI/2,0));
	points[CN_LH].ref_p.pos = Cvector3(0.2,0.2,0.6);
	points[CN_RH].ref_p.pos = Cvector3(0.2,-0.2,0.6);

	// read sensors
	loadData(W, points, init_pos,  sens_vel,  sens_acc,  sens_tau);

	// set position/torque control modes
	W.initializeJoint(mode.segment(6,AIR_N_U-6));

	// initialize bifurcation and parameters (from txt file with parameters)
	Parameters *p = new Parameters(3, "/home/lasapc28/catkin_ws/logs_icub/params.txt");
	cout << p->getRho0() << ", " << p->getM() << ", " << p->getR() << endl;
	float orig_rho0 = p->getRho0();
	float *a = p->getA();
	float *x0 = p->getX0();
	cout << a[0] << ", " << a[1] << ", " << a[2] << endl;
	cout << x0[0] << ", " << x0[1] << ", " << x0[2] << endl;
	cout << p->getRotMat() << endl;
	Bifurcation bif(p,114.0f);
	float t = 0;
	float dt = 1/114.0;
	
	// start the loop
	nonblock(1);
	while(!exit_sim)
	{
		// read sensors
		loadData(W, points, sens_pos,  sens_vel,  sens_acc,  sens_tau);
		double time = W.time;

		// motions
		if(time < 5)
			points[CN_CM].ref_p.pos = Cvector3(0,0.08*(time/5.0),0.53);
		else{
			// Pass 0 when calling the function if you want to record data trajectories for learning
			if (!strcmp(argv[argc-1],"0")){
				// if(time < 10)
				if(time < 10)
					// points[CN_RF].ref_p.pos = Cvector3(0,-0.065, 0) + Cvector3(0.4,-0.035, 0.45)*(time-5.0)/5.0;
					points[CN_RF].ref_p.pos = Cvector3(0, -0.065, 0) + Cvector3(0.35, -0.035, 0.45)*(time-5.0)/5.0;
				else {
					// swing motions
					double delta = time - 10.0;
					points[CN_RF].ref_p.pos = Cvector3(0.4, -0.1, 0.45) + Cvector3(cos(2.0*(delta+M_PI/2)), sin(2.0*(delta+M_PI/2)), 0) * 0.05;
				}
			}
			// 
			else {
				if(time < 8)
					points[CN_RF].ref_p.pos = Cvector3(0,-0.065, 0) + Cvector3(0.35, -0.035, 0.45)*(time-5.0)/5.0;
				else {
					// change the rotation matrix:
					if(time > 20 && time < 20.1){
						Eigen::Matrix3f rotMat;
						rotMat << 0, 0, 1,
								  0, 1, 0,
								  -1, 0, 0;
						bif.p->changeRotMat(rotMat);
					}
					// switch to attractor behaviour
					if(time > 30 && time < 30.1){
						bif.p->changeRho0(0.0);
					}
					// switch back to original plane and radius, but invert the rotation direction
					if(time > 35 && time < 35.1){
						bif.p->changeRotMat(Eigen::Matrix3f::Identity());
						bif.p->changeR(-bif.p->getR());
						bif.p->changeRho0(orig_rho0);
					}
					dt = time - t;
					// Calculate next position from DS
					points[CN_RF].ref_p.pos = bif.nextPos(points[CN_RF].ref_p.pos.cast<float>(),dt).cast<double>();
				}
			}
		}
		t = time;

		// Save trajectory to file
		OutRecord << time << ", " << points[CN_RF].ref_p.pos(0) << ", " << points[CN_RF].ref_p.pos(1) << ", " << points[CN_RF].ref_p.pos(2) << endl;

		// whole-body IK ////////////////////////////////////////////////////////////////////////////////
		double IK_time = IK(points, ikparam, ref_pos, freezeIK);

		// send final commands
		Cvector pos_command = init_pos * exp(-time/1.0) + ref_pos * (1.0-exp(-time/1.0));
		W.controlJoint( mode.segment(6,AIR_N_U-6),
						pos_command.segment(6,AIR_N_U-6) / M_PI * 180,
						ref_tau.segment(6,AIR_N_U-6));
	}

	W.initializeJoint(Cvector::Zero(AIR_N_U-6));
	nonblock(0);
	OutRecord.close();
	delete p;
	W.close();
	return 0;
}
