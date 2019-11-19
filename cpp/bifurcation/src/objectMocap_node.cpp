#include "ros/ros.h"
#include "std_msgs/String.h"
#include "objectMocap.h"
#include "geometry_msgs/Pose.h"
#include "geometry_msgs/Twist.h"
#include "geometry_msgs/Quaternion.h"
#include "geometry_msgs/WrenchStamped.h"
#include "geometry_msgs/Wrench.h"
#include "geometry_msgs/Twist.h"
#include "geometry_msgs/PoseStamped.h"
#include "parameters.h"

typedef struct objPose{
	Eigen::Matrix3f position;
	Eigen::Matrix<float,3,4> orientation;
} ObjPose;

ObjPose o;

void updateOptitrackPose(const geometry_msgs::PoseStamped::ConstPtr& msg, int k) 
{
	o.position.row(k) << msg->pose.position.x, msg->pose.position.y, msg->pose.position.z;
	o.orientation.row(k) << msg->pose.orientation.w, msg->pose.orientation.x, msg->pose.orientation.y, msg->pose.orientation.z;
  std::cerr << "Position received: " << msg->pose.position.x << std::endl;
}


int main(int argc, char **argv)
{
  // Ros initialization
  ros::init(argc, argv, "objectMocap");

  ros::NodeHandle n;

  // Test with random values
  Eigen::Matrix<float,NB_MARKERS,3> p1 = Eigen::Matrix<float,NB_MARKERS,3>::Random();
  Eigen::Matrix<float,NB_MARKERS,4> o1 = Eigen::Matrix<float,NB_MARKERS,4>::Random();
  ObjectMocap obj1(p1,o1);

  // Test with mocap values
  n.subscribe<geometry_msgs::PoseStamped>("/vrpn_client_node/Ilaria1/pose", 1, boost::bind(&updateOptitrackPose,_1,0),ros::VoidPtr(),ros::TransportHints().reliable().tcpNoDelay());
  n.subscribe<geometry_msgs::PoseStamped>("/vrpn_client_node/Ilaria2/pose", 1, boost::bind(&updateOptitrackPose,_1,1),ros::VoidPtr(),ros::TransportHints().reliable().tcpNoDelay());
  n.subscribe<geometry_msgs::PoseStamped>("/vrpn_client_node/Ilaria3/pose", 1, boost::bind(&updateOptitrackPose,_1,2),ros::VoidPtr(),ros::TransportHints().reliable().tcpNoDelay());
  while(ros::ok())
  {
    ros::spinOnce();
    ros::Duration(0, 100000000).sleep();
  }
  std::cerr << o.position << std::endl;
  std::cerr << o.orientation << std::endl;
  ObjectMocap obj2(o.position,o.orientation);

  ros::spinOnce();
  ros::Duration(1, 0).sleep();
}

