#include <signal.h>
#include <mutex>
#include "transformations.h"
#include "parameters.h"
//#include "ros/ros.h"
//#include "geometry_msgs/Pose.h"
//#include "geometry_msgs/Twist.h"
//#include "geometry_msgs/Quaternion.h"
#include "Eigen/Eigen"
//#include "Utils.h"
#include <iostream>
#include <fstream>

// Dynamic reconfingure to change parameters at runtime
//#include <dynamic_reconfigure/server.h>
//#include <bifurcation/parametersDynConfig.h>

class Bifurcation {

	private:
		// ROS variables
		//ros::NodeHandle _n;
		//ros::Rate _loopRate = 100.0f;
		
		// Subscribers and publishers definition
		//ros::Subscriber _subRealPose;			// Subscribe to robot current pose
		//ros::Publisher _pubDesiredTwist;		// Publish desired twist
		//ros::Publisher _pubDesiredOrientation;  // Publish desired orientation

		//geometry_msgs::Pose _msgDesiredPose;
		//geometry_msgs::Quaternion _msgDesiredOrientation;
		//geometry_msgs::Twist _msgDesiredTwist;

		// Node variables
		float frequency = 100.0f;
		
		Eigen::Vector3f x;			// Current position
		Eigen::Vector3f xd;			// Desired position
		float * vd;					// Desired velocity
		float dt;					// 1/frequency

		Eigen::Vector4f _q;
		Eigen::Vector4f _qd;
		Eigen::Matrix3f _wRb;

		bool _stop;
		bool poseReceived = false;
		// Class variables
		std::mutex _mutex;

		//Saving to file
		std::ofstream myfile;
		bool _saveToFile = false;

    	//void getPosVel(const geometry_msgs::Pose::ConstPtr& msg);
    	void updatePosVel();
    	//static void stopNode(int sig); 
    	void publishData();

    	static Bifurcation* me;
	    // Dynamic reconfigure (server+callback)
	    //dynamic_reconfigure::Server<bifurcation::parametersDynConfig> _dynRecServer;
	    //dynamic_reconfigure::Server<bifurcation::parametersDynConfig>::CallbackType _dynRecCallback;
	    //void dynamicReconfigureCallback(bifurcation::parametersDynConfig &config, uint32_t level);

	public:
		//Bifurcation(ros::NodeHandle &n, Parameters *p, float frequency);
		Parameters *p;
		Bifurcation(Parameters *p, float frequency);
		int init();
		void run();
		void computeDesiredOrientation();
    	Eigen::Vector3f nextPos(Eigen::Vector3f x_prev,float dt); 
};
