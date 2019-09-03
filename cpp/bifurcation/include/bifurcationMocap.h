#include <signal.h>
#include <mutex>
#include <algorithm>
#include <boost/bind.hpp>
#include "transformations.h"
#include "parameters.h"
#include "ros/ros.h"
#include "geometry_msgs/Pose.h"
#include "geometry_msgs/PoseStamped.h"
#include "geometry_msgs/Twist.h"
#include "geometry_msgs/Quaternion.h"
#include "Eigen/Eigen"
#include "Utils.h"
#include <iostream>
#include <fstream>
#include "objectMocap.h"

// Dynamic reconfingure to change parameters at runtime
#include <dynamic_reconfigure/server.h>
#include <bifurcation/mocapObjectsConfig.h>

#define NB_ROBOTS 1
#define TOTAL_NB_MARKERS 7

class Bifurcation {

	private:
		// ROS variables
		ros::NodeHandle _n;
		ros::Rate _loopRate;
		
		// Subscribers and publishers definition
		ros::Subscriber _subRealPose;			// Subscribe to robot current pose
		ros::Publisher _pubDesiredTwist;		// Publish desired twist
		ros::Publisher _pubDesiredOrientation;  // Publish desired orientation
		ros::Subscriber _subOptitrackPose[TOTAL_NB_MARKERS];	// Subscribe to optitrack markers

		geometry_msgs::Pose _msgDesiredPose;
		geometry_msgs::Quaternion _msgDesiredOrientation;
		geometry_msgs::Twist _msgDesiredTwist;

		// Node variables
		Parameters *p;		
		Eigen::Vector3f x;			// Current position
		Eigen::Vector3f xd;			// Desired position
		float* vd;					// Desired velocity
		float dt;					// 1/frequency

		Eigen::Vector4f _q;
		Eigen::Vector4f _qd;
		Eigen::Matrix3f _wRb;

		bool _stop;
		bool poseReceived;
		// Class variables
		std::mutex _mutex;

		//Saving to file
		std::ofstream myfile;
		bool _saveToFile = true;

    	void getPosVel(const geometry_msgs::Pose::ConstPtr& msg);
    	void updatePosVel();
    	static void stopNode(int sig); 
    	void publishData();

    	static Bifurcation* me;
	    // Dynamic reconfigure (server+callback)
	    dynamic_reconfigure::Server<bifurcation::mocapObjectsConfig> _dynRecServer;
	    dynamic_reconfigure::Server<bifurcation::mocapObjectsConfig>::CallbackType _dynRecCallback;
	    void dynamicReconfigureCallback(bifurcation::mocapObjectsConfig &config, uint32_t level);

	    // Optitrack
	    void updateOptitrackPose(const geometry_msgs::PoseStamped::ConstPtr& msg, int k);
	    void computeObjectPose(int object_number);
	    Eigen::Matrix<float,3,TOTAL_NB_MARKERS> _markersPosition;       // Markers position in optitrack frame
    	Eigen::Matrix<float,4,TOTAL_NB_MARKERS> _markersOrientation;      // Markers orientation in opittrack frame
    	//Eigen::Matrix<uint32_t,TOTAL_NB_MARKERS,1> _markersSequenceID;  // Markers sequence ID
    	//Eigen::Matrix<uint16_t,TOTAL_NB_MARKERS,1> _markersTracked;     // Markers tracked state
    	ObjectMocap obj[(TOTAL_NB_MARKERS-1)/3];
    	bool _firstOptitrackPose[TOTAL_NB_MARKERS] = {0};
    	bool _firstObjectPose[(TOTAL_NB_MARKERS-1)/3] = {0};

	public:
		Bifurcation(ros::NodeHandle &n, Parameters *p, float frequency);
		int init();
		void run();
		void computeDesiredOrientation();

};
