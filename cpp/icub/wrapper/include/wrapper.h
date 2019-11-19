#pragma once

#include <Eigen/Dense>
#include <Eigen/Core>
#include <yarp/os/Network.h>
#include <yarp/dev/ControlBoardInterfaces.h>
#include <yarp/dev/PolyDriver.h>
#include <yarp/os/Time.h>
#include <yarp/sig/Vector.h>
#include <yarp/os/Bottle.h>
#include <yarp/dev/IPidControl.h>
#include <yarp/dev/ControlBoardPid.h>
#include <yarp/dev/PidEnums.h>
#include <yarp/dev/IControlLimits.h>


struct sensorConnector
{
	yarp::os::BufferedPort<yarp::os::Bottle> port;
	yarp::os::Bottle *values;
};

struct jointConnector
{
	yarp::dev::PolyDriver			robotDevice;
	yarp::dev::IPositionDirect		*pos;
	yarp::dev::IEncoders			*encs;
	yarp::dev::IControlMode			*ictrl;
	yarp::dev::IInteractionMode		*iint;
	yarp::dev::IImpedanceControl	*iimp;
	yarp::dev::ITorqueControl		*itrq;
	yarp::dev::IPidControl			*ipid;
	yarp::dev::IControlLimits		*ilimit;
	int								number;
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

	// rotation algebra
	Eigen::Matrix3d rotmatrix(Eigen::Vector3d a);
	Eigen::VectorXd quat_mul(Eigen::VectorXd a, Eigen::VectorXd b);
	Eigen::VectorXd dc2quat(Eigen::Matrix3d dircos);

	// YARP stuff
	yarp::os::Network yarp;
	yarp::os::Property params;
	std::string robotName;
	Eigen::MatrixXd map;
	jointConnector JointPort[6];
	sensorConnector SensorPort[5];

	// Creating port for publishing CoM pose of robot in world frame [to read in ROS!]
	yarp::os::BufferedPort<yarp::sig::Vector> CoMPose_port_Out;
	yarp::sig::Vector CoMPoses;
	// TODO: Make it nice.. with Salman's structure

	// external wrenches
	sensorConnector ExternalWrenchPort;
	int applyExternalWrench(std::string link, Eigen::VectorXd Force, double duration);

	// setup functions
	int checkRobot(int argc, char *argv[]);
	int setupSensorConnector(sensorConnector &F, std::string moduleName, std::string analog);
	int setupJointConnector(jointConnector &C, std::string moduleName, int size);
	int initialize();
	void close();

	// measured variables
	double YAW;
	Eigen::Vector3d linacc;
	Eigen::Vector3d angvel;
	Eigen::MatrixXd R;
	Eigen::VectorXd sens_pos;
	Eigen::VectorXd sens_vel;
	Eigen::VectorXd sens_acc;
	Eigen::VectorXd sens_tau;
	Eigen::Vector3d FTsensor[4][2];

	// time management
	sensorConnector TimePort;
	double startTime;
	double time;
	
	// global position
	sensorConnector GlobalPosPort;
	Eigen::VectorXd Root;

	// objects
	sensorConnector ObjectPorts[10];
	Eigen::VectorXd ObjectPos[10];
	void readObject(int k, std::string name);

	// online functions
	void initializeJoint(Eigen::VectorXd mode);
	void controlJoint(Eigen::VectorXd mode, Eigen::VectorXd ref_pos, Eigen::VectorXd ref_tau);
	void setPidJoint(int k, double kp, double kd, double ki);
	void readSensors();
	void getJointLimits(double * minPosition, double * maxPosition);
	
};
