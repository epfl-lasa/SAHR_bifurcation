#pragma once

#include <Eigen/Dense>
#include <Eigen/Core>
#include <yarp/os/Network.h>
#include <yarp/dev/ControlBoardInterfaces.h>
#include <yarp/dev/PolyDriver.h>
#include <yarp/os/Time.h>
#include <yarp/sig/Vector.h>
#include <yarp/os/Bottle.h>
//#include <yarp/dev/Control.h>
//#include <yarp/dev/PidEnums.h>

#include "IMU.h"
#include <pthread.h>
#include <sys/time.h>
#include <iostream>

Eigen::Matrix3d rotmatrix(Eigen::Vector3d a);

struct sensorConnector
{
	yarp::os::BufferedPort<yarp::os::Bottle> port;
	yarp::os::Bottle *values;
};

struct jointConnector
{
	yarp::dev::PolyDriver 			robotDevice;
	yarp::dev::IPositionDirect 		*posdirect;
	yarp::dev::IPositionControl 		*pos;
	yarp::dev::IEncoders 			*encs;
	yarp::dev::IControlMode2 		*ictrl;
	yarp::dev::IInteractionMode 		*iint;
	yarp::dev::IImpedanceControl 		*iimp;
	yarp::dev::ITorqueControl 		*itrq;
	yarp::dev::IPidControl			*ipid;
	int 					number;
};

struct gpsConnector
{
	yarp::os::BufferedPort<yarp::os::Bottle> port;
	yarp::os::Bottle *values;
};

class wrapper
{
public:
	wrapper();
	~wrapper();

	// YARP stuff
	yarp::os::Network yarp;
	yarp::os::Property params;
	std::string robotName;
	Eigen::MatrixXd map;
	Eigen::MatrixXd gain;
	jointConnector C[6];
	sensorConnector F[5];
	sensorConnector G[2];
	sensorConnector T[1];

	// setup functions
	int checkRobot(int argc, char *argv[]);
	int setupSensorConnector(sensorConnector &F, std::string moduleName, std::string analog);
	int setupJointConnector(jointConnector &C, std::string moduleName, std::string robotName, int size);
	int initialize();
	void close();

	// IMU
	pthread_mutex_t mutex;
	IMU imu;
	float EULER[3];
	float LINACC[3];
	float ANGVEL[3];

	// measured variables
	double startTime;
	double time;
	double roll, pitch, yaw;
	Eigen::Vector3d linacc;
	Eigen::Vector3d angvel;
	Eigen::MatrixXd R;
	Eigen::VectorXd sens_pos;
	Eigen::VectorXd sens_vel;
	Eigen::VectorXd sens_acc;
	Eigen::VectorXd sens_tau;
	Eigen::Vector3d FTsensor[4][2];
	Eigen::VectorXd Root;
	Eigen::VectorXd Object;

	// online functions
	void initializeJoint(Eigen::VectorXd mode);
	void controlJoint(Eigen::VectorXd mode, Eigen::VectorXd ref_pos, Eigen::VectorXd ref_tau);
	void setPidJoint(int k, double kp, double kd, double ki);
	void getPidJoint(int k, double& kp, double& kd, double& ki);
	void readSensors();
};
