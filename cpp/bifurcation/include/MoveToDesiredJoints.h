#ifndef __MOVE_TO_DESIRED_JOINTS_H__
#define __MOVE_TO_DESIRED_JOINTS_H__

#include "ros/ros.h"
#include "std_msgs/Float64MultiArray.h"
#include "sensor_msgs/JointState.h"
#include <vector>
#include <mutex>


#define NB_ROBOTS 2
#define NB_JOINTS 7

class MoveToDesiredJoints 
{

	public:
		enum Mode {SINGLE_LEFT = 0, SINGLE_RIGHT = 1, BOTH = 2};

	private:

		enum Robot {LEFT = 0, RIGHT = 1};


		// ROS variables
		ros::NodeHandle _n;
		ros::Rate _loopRate;

		// Subscribers and publishers definition
		ros::Subscriber _subCurrentJoints[NB_ROBOTS];
		ros::Publisher _pubDesiredJoints[NB_ROBOTS];

		// Node variables
		std_msgs::Float64MultiArray _currentJoints[NB_ROBOTS];
		std_msgs::Float64MultiArray _desiredJoints[NB_ROBOTS];
		float _jointTolerance;
		bool _firstJointsUpdate[NB_ROBOTS];

		// Class variables

		Mode _mode;
		std::mutex _mutex;


	public:
		MoveToDesiredJoints(ros::NodeHandle &n, float frequency, Mode mode, float jointTolerance = 1.0e-3f);

		// Initialize node
		bool init();

		// Run node main loop
		void run();

		// Set desired joint angles
		void setDesiredJoints(std_msgs::Float64MultiArray desiredJoints);

	private:


		// Callback to update joint position
		void updateCurrentJoints(const sensor_msgs::JointState::ConstPtr& msg, int k);

		// Check joints error
		bool checkJointsError(int k);

};


#endif