#include "ros/ros.h"
#include <signal.h>
#include "geometry_msgs/Pose.h"
#include "geometry_msgs/PoseStamped.h"
#include "geometry_msgs/Twist.h"
#include "geometry_msgs/Quaternion.h"
#include "Eigen/Eigen"
#include <iostream>
#include <fstream>


std::ofstream myfile;

bool _stop = false;

void getPosVel(const geometry_msgs::Pose::ConstPtr& msg);
static void stopNode(int sig);

int main(int argc, char **argv)
{
  // Ros initialization
  ros::init(argc, argv, "recordData");

  ros::NodeHandle n;
  ros::Rate frequency = 100.0f;

  if (argc > 1)
  	myfile.open(argv[1]);
  else
  	myfile.open("kinesthetic_data.txt");

  ROS_INFO("Data recording started.");

  ros::Subscriber subRealPose = n.subscribe<geometry_msgs::Pose>("/lwr/ee_pose", 1, &getPosVel, ros::TransportHints().reliable().tcpNoDelay());

  signal(SIGINT,stopNode);

  // Run loop
  while(ros::ok() && !_stop){

  ros::spinOnce();

  frequency.sleep();
  
  }

  myfile.close();
  ROS_INFO("Data recording stopped.");

  ros::spinOnce();
  frequency.sleep();

  ros::shutdown();

  return 0;

}

void getPosVel(const geometry_msgs::Pose::ConstPtr& msg) {

	Eigen::Vector3f x;

	// Get end effector position
  	x(0) = msg->position.x;
	x(1) = msg->position.y;
	x(2) = msg->position.z;
    myfile << ros::Time::now().toSec() << ", " << x(0) << ", " << x(1) << ", " << x(2) << "\n";

    return;
}

static void stopNode(int sig) {
	_stop = true;
}