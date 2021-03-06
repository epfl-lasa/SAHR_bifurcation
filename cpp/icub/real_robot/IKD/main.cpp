#include "IK.h"
#include "ID.h"
using namespace std;

/* Index reference and a natural posture of icub                      */
/* global positions                                                   */
/*pelvis pos: pos[0]= 0.0; pos[1]= 0.0; pos[2]= 0.5;                  */
/*pelvis rot: pos[3]= 0.0; pos[4]= 0.0; pos[5]= 0.0; pos[38]= 1.0;    */
/*                // right            // left                         */
/*head                     pos[11]= 0.0;                              */
/*neck roll                pos[10]= 0.0;                              */
/*neck pitch               pos[9]= 0.0;                               */
/*shoulder pitch  pos[31]= 0.0;       pos[24]= 0.0;                   */
/*shoulder roll   pos[32]= 0.0;       pos[25]= 0.0;                   */
/*shoulder yaw    pos[33]= 0.0;       pos[26]= 0.0;                   */
/*elbow           pos[34]=-0.8;       pos[27]=-0.8;                   */
/*forearm yaw     pos[35]=-0.8;       pos[28]=-0.8;                   */
/*forearm roll    pos[36]=-0.8;       pos[29]=-0.8;                   */
/*forearm pitch   pos[37]=-0.8;       pos[30]=-0.8;                   */
/*torso roll                pos[6]= 0.0;                              */
/*torso pitch               pos[7]= 0.0;                              */
/*torso yaw                 pos[8]= 0.0;                              */
/*hip pitch       pos[18]=-0.4;       pos[12]=-0.4;                   */
/*hip roll        pos[19]=-0.0;       pos[13]= 0.0;                   */
/*hip yaw         pos[20]= 0.0;       pos[14]= 0.0;                   */
/*knee            pos[21]= 0.8;       pos[15]= 0.8;                   */
/*ankle roll      pos[22]= 0.0;       pos[16]=-0.0;                   */
/*ankle pitch     pos[23]=-0.4;       pos[17]=-0.4;                   */

int main()
{
    // ****** Note that stages are linked together  ****

    //definitions
    Model model;
    model.init();

    // contact definition
    Contact_Manager points;
    points.initialize(&model, 5);
    points[CN_CM].init(CN_CM, body_base,   offset_base,   CT_FULL, PS_FLOATING,  0              , 0             );
    points[CN_LF].init(CN_LF, body_l_foot, offset_l_foot, CT_FULL, PS_CONTACTED, foot_length*0.8, foot_width*0.8);
    points[CN_RF].init(CN_RF, body_r_foot, offset_r_foot, CT_FULL, PS_CONTACTED, foot_length*0.8, foot_width*0.8);
    points[CN_LH].init(CN_LH, body_l_hand, offset_l_hand, CT_FULL, PS_FLOATING,  hand_length*0.8, hand_width*0.8);
    points[CN_RH].init(CN_RH, body_r_hand, offset_r_hand, CT_FULL, PS_FLOATING,  hand_length*0.8, hand_width*0.8);

	// control CoM instead of pelvis
    points.ifCoM = true;

	// default orientations
    points[CN_LH].pre_rot = ang2dc(Cvector3(0,M_PI/2,0));
    points[CN_RH].pre_rot = ang2dc(Cvector3(0,M_PI/2,0));

	// task flexibility
	points[CN_CM].T.slack_cost = Cvector3(1,1,1) * 100;
    points[CN_CM].R.slack_cost = Cvector3(1,1,1);
	points[CN_LF].T.slack_cost = Cvector3(1,1,1) * 100;
    points[CN_LF].R.slack_cost = Cvector3(1,1,1) * 100;
	points[CN_RF].T.slack_cost = Cvector3(1,1,1) * 100;
    points[CN_RF].R.slack_cost = Cvector3(1,1,1) * 100;
	points[CN_LH].T.slack_cost = Cvector3(1,1,1);
    points[CN_LH].R.slack_cost = Cvector3(1,1,1);
	points[CN_RH].T.slack_cost = Cvector3(1,1,1);
    points[CN_RH].R.slack_cost = Cvector3(1,1,1);

	// joints variables    
    Cvector ref_pos = Cvector::Zero(AIR_N_U+1); ref_pos[AIR_N_U] = 1;
    Cvector freeze  = Cvector::Zero(AIR_N_U);
    Cvector ref_vel = Cvector::Zero(AIR_N_U);
	Cvector ref_acc = Cvector::Zero(AIR_N_U);
    Cvector ref_tau = Cvector::Zero(AIR_N_U);

    // fixed joints
	for(int i=6;i<=11;i++)
    	freeze[i] = 1;

    // position tasks
    points[CN_CM].ref_p.pos = Cvector3(0, 0,  0.6);
    points[CN_LF].ref_p.pos = Cvector3(0, 0.075,0);
    points[CN_RF].ref_p.pos = Cvector3(0,-0.075,0);
    points[CN_LH].ref_p.pos = Cvector3(0.2, 0.2,0.5);
    points[CN_RH].ref_p.pos = Cvector3(0.2,-0.2,0.5);
    // orientation tasks
    points[CN_CM].ref_o.pos = zero_quat;
    points[CN_LF].ref_o.pos = zero_quat;
    points[CN_RF].ref_o.pos = zero_quat;
    points[CN_LH].ref_o.pos = ang2quat(Cvector3(0,0,0.5));
    points[CN_RH].ref_o.pos = ang2quat(Cvector3(0,0,0.5));

	// solving IK problem
    IKPARAM ikparam;
    double IK_time = IK(points, ikparam, ref_pos, freeze);

    // model update
	model.set_state(0, ref_pos, ref_vel);
	points.update_kinematics();
	// model.check_consistency(ref_pos, ref_vel, ref_acc);

	
    // dynamic limits
    Cvector Qacc = Cvector::Ones(AIR_N_U) * 1e-3;
    Cvector Qtau = Cvector::Ones(AIR_N_U)/30.0/30.0 * 1e-8;

    // solving ID problem
	IDPARAM idparam;
    double ID_time = ID(points, idparam,
						Qacc, Qtau,
						ref_tau, ref_acc, freeze);
	model.set_acc(ref_acc);
    points.update_dynamics();

	// plot all joint variables
	points.print_IK_errors();
	points.print_ID_errors();
	points.print_forces();
    
	cout << setprecision( 3 ) << "IK time:   " << IK_time << "  ---   ID time:   " << ID_time << endl;
	cout << fixed << endl;
    cout << "  joint     min_pos      pos      max_pos      vel        acc     torque  " << endl;
    for(int i=0;i<AIR_N_U;i++)
        cout << std::setw( 5 ) << i << ":"  <<
	            setprecision( 0 ) <<std::setw( 11 ) << (i<6 ? -1000 : model.qmin[i-6]) <<
                setprecision( 2 ) <<std::setw( 11 ) << ref_pos[i] * (i>=6?180/M_PI:1) <<
	            setprecision( 0 ) <<std::setw( 11 ) << (i<6 ? -1000 : model.qmax[i-6]) <<
                setprecision( 2 ) <<std::setw( 11 ) << ref_vel[i] <<
                setprecision( 2 ) <<std::setw( 11 ) << ref_acc[i] <<
                setprecision( 2 ) <<std::setw( 11 ) << ref_tau[i] <<
                endl;

    return 0;
}
