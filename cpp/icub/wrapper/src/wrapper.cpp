#include "wrapper.h"
#include <iostream>

using namespace Eigen;

Matrix3d wrapper::rotmatrix(Vector3d a)
{
	// a: roll/pitch/yaw or XYZ Tait-Bryan angles
	// https://en.wikipedia.org/wiki/Euler_angles
	double cos1 = cos(a[0]);
	double cos2 = cos(a[1]);
	double cos3 = cos(a[2]);
	double sin1 = sin(a[0]);
	double sin2 = sin(a[1]);
	double sin3 = sin(a[2]);

	Matrix3d dircos;
	dircos(0,0) = (cos2*cos3);
	dircos(0,1) = -(cos2*sin3);
	dircos(0,2) = sin2;
	dircos(1,0) = ((cos1*sin3)+(sin1*(cos3*sin2)));
	dircos(1,1) = ((cos1*cos3)-(sin1*(sin2*sin3)));
	dircos(1,2) = -(cos2*sin1);
	dircos(2,0) = ((sin1*sin3)-(cos1*(cos3*sin2)));
	dircos(2,1) = ((cos1*(sin2*sin3))+(cos3*sin1));
	dircos(2,2) = (cos1*cos2);
	return dircos;
}

VectorXd wrapper::dc2quat(Matrix3d dircos)
{
	double tmp,tmp1,tmp2,tmp3,tmp4,temp[10];

	tmp = (dircos(0,0)+(dircos(1,1)+dircos(2,2)));
	if (((tmp >= dircos(0,0)) && ((tmp >= dircos(1,1)) && (tmp >= dircos(2,2)))))
	{
		tmp1 = (dircos(2,1)-dircos(1,2));
		tmp2 = (dircos(0,2)-dircos(2,0));
		tmp3 = (dircos(1,0)-dircos(0,1));
		tmp4 = (1.+tmp);
	} 
	else 
	{
		if (((dircos(0,0) >= dircos(1,1)) && (dircos(0,0) >= dircos(2,2))))
		{
			tmp1 = (1.-(tmp-(2.*dircos(0,0))));
			tmp2 = (dircos(0,1)+dircos(1,0));
			tmp3 = (dircos(0,2)+dircos(2,0));
			tmp4 = (dircos(2,1)-dircos(1,2));
		}
		else
		{
			if((dircos(1,1) >= dircos(2,2)))
			{
				tmp1 = (dircos(0,1)+dircos(1,0));
				tmp2 = (1.-(tmp-(2.*dircos(1,1))));
				tmp3 = (dircos(1,2)+dircos(2,1));
				tmp4 = (dircos(0,2)-dircos(2,0));
			}
			else
			{
				tmp1 = (dircos(0,2)+dircos(2,0));
				tmp2 = (dircos(1,2)+dircos(2,1));
				tmp3 = (1.-(tmp-(2.*dircos(2,2))));
				tmp4 = (dircos(1,0)-dircos(0,1));
			}
		}
	}
	tmp = (1./sqrt(((tmp1*tmp1)+((tmp2*tmp2)+((tmp3*tmp3)+(tmp4*tmp4))))));

	VectorXd e = VectorXd::Zero(4);
	e[0] = (tmp*tmp1);
	e[1] = (tmp*tmp2);
	e[2] = (tmp*tmp3);
	e[3] = (tmp*tmp4);
	return e;
}

VectorXd wrapper::quat_mul(VectorXd a, VectorXd b)
{
	VectorXd res(4);
	res[0] =  a[1]*b[2] - a[2]*b[1] + a[0]*b[3] + a[3]*b[0];
	res[1] =  a[2]*b[0] - a[0]*b[2] + a[1]*b[3] + a[3]*b[1];
	res[2] =  a[0]*b[1] - a[1]*b[0] + a[2]*b[3] + a[3]*b[2];
	res[3] = -a[0]*b[0] - a[1]*b[1] - a[2]*b[2] + a[3]*b[3];
	if (res[3]<0)
		res *= -1;
	return res;
}

wrapper::wrapper()
{
}

wrapper::~wrapper()
{
}

int wrapper::checkRobot(int argc, char *argv[])
{
	params.fromCommand(argc, argv);
	if (!params.check("robot"))
	{
		fprintf(stderr, "Please specify the name of the robot\n");
		fprintf(stderr, "--robot name (e.g. icubSim)\n");
		return 1;
	}
	robotName = params.find("robot").asString();
	return 0;
}

int wrapper::setupSensorConnector(sensorConnector &F, std::string moduleName, std::string analog)
{
	F.port.open("/test" + moduleName);
	yarp::os::Network::connect(analog.c_str(), F.port.getName().c_str());
	return 0;
}

int wrapper::setupJointConnector(jointConnector &C, std::string moduleName, int size)
{
	std::string remotePorts = moduleName;

	std::string localPorts = "/test" + moduleName;

	yarp::os::Property options;
	options.put("device", "remote_controlboard");
	options.put("local", localPorts);
	options.put("remote", remotePorts);

	C.robotDevice.open(options);
	if (!C.robotDevice.isValid()) 
	{
		printf("Device not available.  Here are the known devices:\n");
		printf("%s", yarp::dev::Drivers::factory().toString().c_str());
		return 1;
	}

	bool ok;
	ok = C.robotDevice.view(C.pos);
	ok = ok && C.robotDevice.view(C.encs);
	ok = ok && C.robotDevice.view(C.ictrl);
	ok = ok && C.robotDevice.view(C.iint);
	ok = ok && C.robotDevice.view(C.iimp);
	ok = ok && C.robotDevice.view(C.itrq);
	ok = ok && C.robotDevice.view(C.ipid);
	ok = ok && C.robotDevice.view(C.ilimit);
	if (!ok) 
	{
		printf("Problems acquiring interfaces\n");
		return 1;
	}

	C.number = size;
	int nj = 0;
	C.pos->getAxes(&nj);
	//C.encoders.resize(nj);
	//C.torques.resize(nj);
	return 0;
}

int wrapper::initialize()
{
	/* Index reference and a natural posture of icub	   */
	/* global positions									   */
	/* pelvis pos:	 pos[0] pos[1] pos[2]				   */
	/* pelvis rot:	 pos[3] pos[4] pos[5] pos[38]		   */
	/*				 // right	  // left				   */
	/* head                 pos[11]						   */
	/* neck roll            pos[10]						   */
	/* neck pitch           pos[9]						   */
	/* shoulder pitch  pos[31]     pos[24]				   */
	/* shoulder roll   pos[32]     pos[25]				   */
	/* shoulder yaw	   pos[33]     pos[26]				   */
	/* elbow           pos[34]     pos[27]				   */
	/* forearm yaw     pos[35]     pos[28]				   */
	/* forearm roll    pos[36]     pos[29]				   */
	/* forearm pitch   pos[37]     pos[30]				   */
	/* torso pitch          pos[6]						   */
	/* torso roll           pos[7]						   */
	/* torso yaw            pos[8]						   */
	/* hip pitch       pos[18]     pos[12]				   */
	/* hip roll        pos[19]     pos[13]				   */
	/* hip yaw         pos[20]     pos[14]				   */
	/* knee            pos[21]     pos[15]				   */
	/* ankle pitch     pos[22]     pos[16]				   */
	/* ankle roll      pos[23]     pos[17]				   */

	map = MatrixXd(6,7);
	map <<  2 ,1 ,0 ,-1,-1,-1,-1,
			3 ,4 ,5 ,-1,-1,-1,-1,
			6 ,7 ,8 ,9 ,10,11,-1,
			12,13,14,15,16,17,-1,
			18,19,20,21,22,23,24,
			25,26,27,28,29,30,31;

	YAW = 0;
	std::string robotName1 = "/" + robotName;
	int status = 0;

	// controllers
	status |= setupJointConnector(JointPort[0], robotName1 + "/torso", 3);
	status |= setupJointConnector(JointPort[1], robotName1 + "/head", 3);
	status |= setupJointConnector(JointPort[2], robotName1 + "/left_leg", 6);
	status |= setupJointConnector(JointPort[3], robotName1 + "/right_leg", 6);
	status |= setupJointConnector(JointPort[4], robotName1 + "/left_arm", 7);
	status |= setupJointConnector(JointPort[5], robotName1 + "/right_arm", 7);

	// other sensors
	status |= setupSensorConnector(SensorPort[0], robotName1 + "/IMU_In:i", robotName1 + "/inertial");
	status |= setupSensorConnector(SensorPort[1], robotName1 + "/l_foot_FT_readPort", robotName1 + "/left_foot/analog:o");
	status |= setupSensorConnector(SensorPort[2], robotName1 + "/r_foot_FT_readPort", robotName1 + "/right_foot/analog:o");
	status |= setupSensorConnector(SensorPort[3], robotName1 + "/l_arm_FT_readPort", robotName1 + "/left_arm/analog:o");
	status |= setupSensorConnector(SensorPort[4], robotName1 + "/r_arm_FT_readPort", robotName1 + "/right_arm/analog:o");

	// global positions
	status |= setupSensorConnector(GlobalPosPort, robotName1 + "/RootlinkPose_In:i", robotName1 + "/get_root_link_WorldPose:o");

	// ====== Opening the port to publish the Aligned CoM in position+quaternion ====== //
	std::string CoMPortName;
	CoMPortName += robotName1;
	CoMPortName += "/CoMPose:o";

	CoMPose_port_Out.open(CoMPortName.c_str());
	CoMPoses.resize(7, 0.0);


	// Gazebo clock
	status |= setupSensorConnector(TimePort, robotName1 + "/clock", "/clock");

	//external wrench
	std::string portname = robotName1 + "/applyExternalWrench/rpc:i";
	ExternalWrenchPort.port.open(robotName1 + "/applyExternalWrench:o");
	yarp::os::Network::connect(ExternalWrenchPort.port.getName().c_str(), portname.c_str());

	sens_pos = VectorXd::Zero(32);
	sens_vel = VectorXd::Zero(32);
	sens_acc = VectorXd::Zero(32);
	sens_tau = VectorXd::Zero(32);
	Root	 = VectorXd::Zero(7);
	for(int k=0;k<10;k++)
		ObjectPos[k] = VectorXd::Zero(7);

	// time management
	double time = 0;
	TimePort.values = TimePort.port.read();
	if(TimePort.values == NULL)
		std::cout << "WARNING: problem with getting simulation time! run gazebo with -s libgazebo_yarp_clock.so" << std::endl;
	else
		time  = TimePort.values->get(0).asDouble() + TimePort.values->get(1).asDouble()/pow(10,9);
	startTime = time;
	return status;
}

void wrapper::readObject(int k, std::string name)
{
	if(k<0 || k>=10)
	{
		std::cout << "WARNING: not supported object number" << std::endl;
		return;
	}
	if(!ObjectPorts[k].port.isClosed())
		ObjectPorts[k].port.close();
	std::string robotName1 = "/" + robotName;
	setupSensorConnector(ObjectPorts[k], robotName1 + "/ObjectlinkPose_In:i", "/" + name + "/GraspedObject_WorldPose:o");
}

void wrapper::close()
{
	for(int i=0; i<6; i++)
		JointPort[i].robotDevice.close();
	for(int i=0; i<5; i++)
		SensorPort[i].port.close();
}

void wrapper::initializeJoint(VectorXd mode)
{
	for(int i=0; i<6; i++)
	{
		for(int j=0; j<JointPort[i].number;j++)
		{
			if(mode[map(i,j)] == 0)
			{
				JointPort[i].ictrl->setControlMode(j, VOCAB_CM_POSITION_DIRECT);
				JointPort[i].iint->setInteractionMode(j, yarp::dev::VOCAB_IM_STIFF);
			}
			else
			{
				JointPort[i].ictrl->setControlMode(j, VOCAB_CM_TORQUE);
				JointPort[i].iint->setInteractionMode(j, yarp::dev::VOCAB_IM_COMPLIANT);
			}
		}
	}
}

void wrapper::getJointLimits(double * minPosition, double * maxPosition)
{
	for(int i=0; i<6; i++)
		for(int j=0; j<JointPort[i].number;j++)
			JointPort[i].ilimit->getLimits(j, &minPosition[int(map(i,j))], &maxPosition[int(map(i,j))]);
}

void wrapper::setPidJoint(int k, double kp, double kd, double ki)
{
	yarp::dev::Pid * pid = new yarp::dev::Pid;
	for(int i=0; i<6; i++)
	{
		for(int j=0; j<JointPort[i].number;j++)
		{
			if(map(i,j) == k)
			{
				JointPort[i].ictrl->setControlMode(j, VOCAB_CM_POSITION_DIRECT);
				JointPort[i].iint->setInteractionMode(j, yarp::dev::VOCAB_IM_STIFF);
				JointPort[i].ipid->getPid(yarp::dev::VOCAB_PIDTYPE_POSITION,j,pid);
				//printf("%d,%d: %2.2f, %2.2f, %2.2f,	 %2.2f\n", i, j, pid->kp, pid->kd, pid->ki, pid->scale);
				pid->kp = kp;
				pid->kd = kd;
				pid->ki = ki;
				JointPort[i].ipid->setPid(yarp::dev::VOCAB_PIDTYPE_POSITION,j,*pid);
			}
		}
	}
}

void wrapper::controlJoint(VectorXd mode, VectorXd ref_pos, VectorXd ref_tau)
{
	for(int i=0; i<6; i++)
	{
		for(int j=0; j<JointPort[i].number;j++)
		{
			if(mode[map(i,j)] == 0)
			{
				JointPort[i].pos->setPosition(j, ref_pos[map(i,j)]);
			}
			else
			{
				JointPort[i].itrq->setRefTorque(j, ref_tau[map(i,j)]);
			}
		}
	}
}

int wrapper::applyExternalWrench(std::string link, VectorXd Force, double duration)
{
	yarp::os::Bottle& bot = ExternalWrenchPort.port.prepare();
	bot.clear();
	bot.addString(link);
	bot.addFloat64(Force[0]);
	bot.addFloat64(Force[1]);
	bot.addFloat64(Force[2]);
	bot.addFloat64(Force[3]);
	bot.addFloat64(Force[4]);
	bot.addFloat64(Force[5]);
	bot.addFloat64(duration);
	ExternalWrenchPort.port.write();
}

void wrapper::readSensors()
{
	// reading simulation time
	TimePort.values = TimePort.port.read();
	//if(!TimePort.values)
	//	std::cout << "WARNING: problem with getting simulation time! run gazebo with -s libgazebo_yarp_clock.so" << std::endl;
	double old_time = time;
	if(TimePort.values)
		time = TimePort.values->get(0).asDouble() + TimePort.values->get(1).asDouble()/pow(10,9);
	if(time<startTime)
		startTime = time;
	time-=startTime;
	double dt = std::max(time - old_time,0.005);

	// read IMU
	SensorPort[0].values = SensorPort[0].port.read();
	Vector3d rot;
	for (int j=0; j<3; j++)
	{
		rot[j]	= SensorPort[0].values->get(0+j).asDouble() / 180.0 * M_PI;
		linacc[j] = SensorPort[0].values->get(3+j).asDouble();
		angvel[j] = SensorPort[0].values->get(6+j).asDouble();
	}

	// WARNING: the real robot uses a different convention
	// https://github.com/robotology/icub-gazebo/issues/18
	Matrix3d X = rotmatrix(Vector3d(rot[0],0,0));
	Matrix3d Y = rotmatrix(Vector3d(0,rot[1],0));
	Matrix3d Z = rotmatrix(Vector3d(0,0,rot[2]));
	R = Z * Y * X;

	// x and y directions point opposite in gazebo
	R = rotmatrix(Vector3d(0,0,M_PI)) * R * rotmatrix(Vector3d(0,0,-M_PI));
	YAW = atan2(R(1,0), R(0,0));

	// WARNING: rotations of acc and vel to be checked, assumed global frame here
	angvel = rotmatrix(Vector3d(0,0,M_PI)) * angvel;
	linacc = rotmatrix(Vector3d(0,0,M_PI)) * linacc;

	// read joints
	for(int i=0; i<6; i++)
	{
		for(int j=0; j<JointPort[i].number;j++)
		{
			double position;
			JointPort[i].encs->getEncoder(j, &position);
			sens_vel[map(i,j)] = (position - sens_pos[map(i,j)]) / dt;
			sens_pos[map(i,j)] = position;
			//JointPort[i].encs->getEncoderSpeed(j, &sens_vel[map(i,j)]); // incorrect!
			//JointPort[i].encs->getEncoderAcceleration(j, &sens_acc[map(i,j)]);
			JointPort[i].itrq->getTorque(j, &sens_tau[map(i,j)]);
		}
	}

	// read contact sensors
	for(int i=1; i<5; i++)
	{
		SensorPort[i].values = SensorPort[i].port.read();
		for(int j=0; j<3; j++)
		{
			FTsensor[i-1][0][j] = SensorPort[i].values->get(j).asDouble();
			FTsensor[i-1][1][j] = SensorPort[i].values->get(j+3).asDouble();
		}
		FTsensor[i-1][0][0] *= -1;
		FTsensor[i-1][1][0] *= -1;
	}

	// reading robot position
	GlobalPosPort.values = GlobalPosPort.port.read();
	//if(!GlobalPosPort.values)
	//	std::cout << "WARNING: problem with GetLinkWorldPose!" << std::endl;
	if(GlobalPosPort.values)
	{	
		for(int i=0; i<7; i++){
			Root[i] = GlobalPosPort.values->get(i).asDouble();
			CoMPoses[i] = GlobalPosPort.values->get(i).asDouble();
		}
		Root.segment(3,4) = quat_mul(Root.segment(3,4), dc2quat(rotmatrix(Vector3d(0,0,M_PI))));
	}

	// Write Global Robot Position to output port
	yarp::sig::Vector &output_CoMPose = CoMPose_port_Out.prepare();
	output_CoMPose = CoMPoses;
	CoMPose_port_Out.write();

	// reading object positions
	for(int k=0;k<10;k++)
		if(!ObjectPorts[k].port.isClosed())
		{
			ObjectPorts[k].values = ObjectPorts[k].port.read(false);
			//if(!ObjectPorts[k].values)
			//	std::cout << "WARNING: object not found!" << std::endl;
			if(ObjectPorts[k].values)
				for(int i=0; i<7; i++)
					ObjectPos[k][i] = ObjectPorts[k].values->get(i).asDouble();
		}
}

