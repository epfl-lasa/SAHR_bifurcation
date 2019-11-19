#include "wrapper.h"

void *task(void * input)
{
	while(1)
	{	
		wrapper * d = ((wrapper*)input);
		pthread_mutex_lock(&(d->mutex));
		d->imu.update(d->EULER, d->LINACC, d->ANGVEL);
		pthread_mutex_unlock(&(d->mutex));
		Sleep(1);
	}
}

Eigen::Matrix3d rotmatrix(Eigen::Vector3d a)
{
	// a: roll/pitch/yaw or XYZ Tait-Bryan angles
	// https://en.wikipedia.org/wiki/Euler_angles
	double cos1 = cos(a[0]);
	double cos2 = cos(a[1]);
	double cos3 = cos(a[2]);
	double sin1 = sin(a[0]);
	double sin2 = sin(a[1]);
	double sin3 = sin(a[2]);

	Eigen::Matrix3d dircos;
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
	robotName = "/" + robotName;
	return 0;
}

int wrapper::setupSensorConnector(sensorConnector &F, std::string moduleName, std::string analog)
{
	F.port.open(moduleName);
	yarp::os::Network::connect(analog.c_str(), F.port.getName().c_str());
	return 0;
}

int wrapper::setupJointConnector(jointConnector &C, std::string moduleName, std::string robotName, int size)
{
	std::string remotePorts = robotName + moduleName;
	std::string localPorts = "/test" + robotName + moduleName;

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
	ok = C.robotDevice.view(C.posdirect);
	ok = ok && C.robotDevice.view(C.pos);
	ok = ok && C.robotDevice.view(C.encs);
	ok = ok && C.robotDevice.view(C.ictrl);
	ok = ok && C.robotDevice.view(C.iint);
	ok = ok && C.robotDevice.view(C.iimp);
	ok = ok && C.robotDevice.view(C.itrq);
	ok = ok && C.robotDevice.view(C.ipid);
	if (!ok) 
	{
		printf("Problems acquiring interfaces\n");
		return 1;
	}

	C.number = size;
	//int nj = 0;
	//C.pos->getAxes(&nj);
	//C.encoders.resize(nj);
	//C.torques.resize(nj);
	return 0;
}

int wrapper::initialize()
{
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

	map = Eigen::MatrixXd(6,7);
	map <<  2 ,1 ,0 ,-1,-1,-1,-1,
		3 ,4 ,5 ,-1,-1,-1,-1,
		6 ,7 ,8 ,9 ,10,11,-1,
		12,13,14,15,16,17,-1,
		18,19,20,21,22,23,24,
		25,26,27,28,29,30,31;

	gain = Eigen::MatrixXd(32,3);
	gain << 32000.00, 6000.00, 60.00,
		32000.00, 6000.00, 60.00,
		32000.00, 6000.00, 60.00,
		50.00, 500.00, 1.00,
		50.00, 500.00, 1.00,
		100.00, 700.00, 2.00,
		32000.00, 100.00, 60.00,
		-32000.00, -100.00, -60.00,
		32000.00, 100.00, 60.00, 
		-32000.00, -100.00, -60.00,
		-32000.00, -100.00, -60.00, 
		-32000.00, -100.00, -60.00, 
		32000.00, 100.00, 60.00, 
		-32000.00, -100.00, -60.00,
		32000.00, 100.00, 60.00, 
		-32000.00, -100.00, -60.00,
		-32000.00, -100.00, -60.00, 
		-32000.00, -100.00, -60.00, 
		32000.00, 50.00, 60.00, 
		32000.00, 50.00, 60.00, 
		10000.00, 0.00, 10.00, 
		32000.00, 20.00, 60.00,
		200.00, 1000.00, 1.00, 
		100.00, 100.00, 2.00, 
		100.00, 100.00, 2.00, 
		32000.00, 50.00, 60.00,
		32000.00, 50.00, 60.00, 
		10000.00, 0.00, 10.00, 
		32000.00, 20.00, 60.00, 
		200.00, 1000.00, 1.00,
		100.00, 100.00, 2.00,
		100.00, 100.00, 2.00;

	roll = 0;
	pitch = 0;
	yaw = 0;

	int status = 0;

	// controllers
	status |= setupJointConnector(C[0], "/torso", robotName, 3);
	status |= setupJointConnector(C[1], "/head", robotName, 3);
	status |= setupJointConnector(C[2], "/left_leg", robotName, 6);
	status |= setupJointConnector(C[3], "/right_leg", robotName, 6);
	status |= setupJointConnector(C[4], "/left_arm", robotName, 7);
	status |= setupJointConnector(C[5], "/right_arm", robotName, 7);

	// sensors
	status |= setupSensorConnector(F[0], robotName + "/IMU_In:i", robotName + "/inertial");
	status |= setupSensorConnector(F[1], robotName + "/l_foot_FT_readPort", robotName + "/left_leg/analog:o");
	status |= setupSensorConnector(F[2], robotName + "/r_foot_FT_readPort", robotName + "/right_leg/analog:o");
	status |= setupSensorConnector(F[3], robotName + "/l_arm_FT_readPort", robotName + "/left_arm/analog:o");
	status |= setupSensorConnector(F[4], robotName + "/r_arm_FT_readPort", robotName + "/left_arm/analog:o");

	// global positions
	//status |= setupSensorConnector(G[0], robotName + "/RootlinkPose_In:i", robotName + "/get_root_link_WorldPose:o");
	//status |= setupSensorConnector(G[1], robotName + "/ObjectlinkPose_In:i", "/GraspedObject_WorldPose:o");

	// Gazebo clock
	//status |= setupSensorConnector(T[0], robotName + "/clock", "/clock");


	sens_pos = Eigen::VectorXd::Zero(32);
	sens_vel = Eigen::VectorXd::Zero(32);
	sens_acc = Eigen::VectorXd::Zero(32);
	sens_tau = Eigen::VectorXd::Zero(32);
	Root     = Eigen::VectorXd::Zero(7);
	Object   = Eigen::VectorXd::Zero(7);

	// get init time
	double time  = yarp::os::Time::now();
	startTime = time;

	// initialize IMU
	printf("Check which tty is associated to the IMU:\n");
	printf("ls /dev/ttyACM (now press tab)\n");
	printf("sudo chmod 666 /dev/ttyACM0\n");
	printf("sudo adduser #user dialout\n");
	u32 com_port, baudrate;
	com_port = 0;
	baudrate = 115200;
	imu.init(com_port, baudrate);
	pthread_t thread1;
	pthread_mutex_init (&mutex , NULL);
	int i1 = pthread_create( &thread1, NULL, task, (void*)(this));

	return status;
}

void wrapper::close()
{
	for(int i=0; i<6; i++)
        	C[i].robotDevice.close();
	for(int i=0; i<5; i++)
        	F[i].port.close();
	pthread_mutex_destroy(&mutex);
}

void wrapper::initializeJoint(Eigen::VectorXd mode)
{
	for(int i=0; i<6; i++)
	{
		for(int j=0; j<C[i].number;j++)
		{
			if(mode[map(i,j)] == 0)
			{
				C[i].ictrl->setControlMode(j,VOCAB_CM_POSITION);
				C[i].iint->setInteractionMode(j, yarp::dev::VOCAB_IM_STIFF);
			}
			else if(mode[map(i,j)] == 1)
			{
				C[i].ictrl->setControlMode(j,VOCAB_CM_POSITION_DIRECT);
				C[i].iint->setInteractionMode(j, yarp::dev::VOCAB_IM_STIFF);
			}
			else if(mode[map(i,j)] == 2)
			{
				C[i].ictrl->setControlMode(j, VOCAB_CM_TORQUE);
				C[i].iint->setInteractionMode(j, yarp::dev::VOCAB_IM_COMPLIANT);
			}
		}
	}
}

void wrapper::getPidJoint(int k, double& kp, double& kd, double& ki)
{
	yarp::dev::Pid * pid = new yarp::dev::Pid;
	for(int i=0; i<6; i++)
	{
		for(int j=0; j<C[i].number;j++)
		{
			if(map(i,j) == k)
			{
				C[i].ipid->getPid(j,pid);
				kp = pid->kp;
				kd = pid->kd;
				ki = pid->ki;
				printf("%2.2f, %2.2f, %2.2f \n", pid->kp, pid->kd, pid->ki);
			}
		}
	}
}

void wrapper::setPidJoint(int k, double kp, double kd, double ki)
{
	yarp::dev::Pid * pid = new yarp::dev::Pid;
	for(int i=0; i<6; i++)
	{
		for(int j=0; j<C[i].number;j++)
		{
			if(map(i,j) == k)
			{
				C[i].ipid->getPid(j,pid);
				pid->kp = int(kp * gain(k,0));
				pid->kd = int(kd * gain(k,1));
				pid->ki = int(ki * gain(k,2));
				C[i].ipid->setPid(j,*pid);
			}
		}
	}
}

void wrapper::controlJoint(Eigen::VectorXd mode, Eigen::VectorXd ref_pos, Eigen::VectorXd ref_tau)
{
	for(int i=0; i<6; i++)
	{
		for(int j=0; j<C[i].number;j++)
		{
			if(mode[map(i,j)] == 0)
			{	
				//C[i].pos->positionMove(j, ref_pos[map(i,j)]);
			}
			else if(mode[map(i,j)] == 1)
			{
				C[i].posdirect->setPosition(j, ref_pos[map(i,j)]);
			}
			else if(mode[map(i,j)] == 2)
			{
				//C[i].itrq->setRefTorque(j, ref_tau[map(i,j)]);
			}
		}
	}
}

void wrapper::readSensors()
{
	
	///////////////////////////////////////////////////////////////////////////////////////
	// read IMU and convert to eigen
	Eigen::Vector3d rot;
	rot[0]=EULER[0]; 
	rot[1]=EULER[1]; 
	rot[2]=EULER[2]; 
	linacc[0]=LINACC[0]; 
	linacc[1]=LINACC[1]; 
	linacc[2]=LINACC[2]; 
	angvel[0]=ANGVEL[0]; 
	angvel[1]=ANGVEL[1]; 
	angvel[2]=ANGVEL[2]; 

	// make rotation matrix
	Eigen::Matrix3d X = rotmatrix(Eigen::Vector3d(rot[0],0,0));
	Eigen::Matrix3d Y = rotmatrix(Eigen::Vector3d(0,rot[1],0));
	Eigen::Matrix3d Z = rotmatrix(Eigen::Vector3d(0,0,rot[2]));

	// robot gives roll/pitch/yaw
	R = Z * Y * X;

	// x and y directions point opposite in gazebo
	R = R * rotmatrix(Eigen::Vector3d(0,M_PI/2,0));

	roll = -atan2(R(2,1), R(2,2));
	pitch = -asin(R(2,0));
	yaw = -atan2(R(1,0), R(0,0));

	R = rotmatrix(Eigen::Vector3d(0,pitch,0)) * rotmatrix(Eigen::Vector3d(roll,0,0));

	// WARNING: rotations of acc and vel to be checked
	angvel = rotmatrix(Eigen::Vector3d(0,M_PI/2,0)) * angvel;
	linacc = rotmatrix(Eigen::Vector3d(0,M_PI/2,0)) * linacc;
	///////////////////////////////////////////////////////////////////////////////////////

	// read joints
	for(int i=0; i<6; i++)
	{
		for(int j=0; j<C[i].number;j++)
		{
			C[i].encs->getEncoder(j, &sens_pos[map(i,j)]);
			C[i].encs->getEncoderSpeed(j, &sens_vel[map(i,j)]);
			C[i].encs->getEncoderAcceleration(j, &sens_acc[map(i,j)]);
			C[i].itrq->getTorque(j, &sens_tau[map(i,j)]);
		}
	}

	// read contact sensors
	for(int i=1; i<5; i++)
	{
		F[i].values = F[i].port.read();
		for(int j=0; j<3; j++)
		{
			FTsensor[i-1][0][j] = F[i].values->get(j).asDouble();
			FTsensor[i-1][1][j] = F[i].values->get(j+3).asDouble();
		}
		FTsensor[i-1][0][0] *= -1;
		FTsensor[i-1][1][0] *= -1;
	}

    //G[0].values = G[0].port.read();
    //for(int i=0; i<7; i++)
    //    Root[i] = G[0].values->get(i).asDouble();

    //G[1].values = G[1].port.read();
    //for(int i=0; i<7; i++)
    //    Object[i] = G[1].values->get(i).asDouble();


    time = yarp::os::Time::now();
    if(time<startTime)
        startTime = time;
    time-=startTime;
}

