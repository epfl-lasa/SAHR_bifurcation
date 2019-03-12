#include "ros/ros.h"
#include "std_msgs/String.h"
#include "MoveToDesiredJoints.h"
#include <sstream>


int main(int argc, char **argv)
{
  // Ros initialization
  ros::init(argc, argv, "move_to_desired_joints");

  std_msgs::Float64MultiArray desiredJoints;

  // Set the number of joints
  desiredJoints.data.resize(NB_JOINTS);

  // Initialize desired joints
  for(int k = 0; k < NB_JOINTS; k++)
  {
    desiredJoints.data[k] = 0.0f;
  }


  MoveToDesiredJoints::Mode mode;
  // Check if desired angles are specified with the command line
  if(argc == 8)
  {
    for(int k = 0; k < NB_JOINTS; k++)
    {
      desiredJoints.data[k] = atof(argv[k+1])*M_PI/180.0f;
    }
    
    mode = MoveToDesiredJoints::Mode::SINGLE_RIGHT;
  }
  else if(argc == 10)
  {
    for(int k = 0; k < NB_JOINTS; k++)
    {
      desiredJoints.data[k] = atof(argv[k+1])*M_PI/180.0f;
    }
    if(std::string(argv[8]) == "-m" && std::string(argv[9]) == "l")
    {
      mode = MoveToDesiredJoints::Mode::SINGLE_LEFT;
    }
    else if(std::string(argv[8]) == "-m" && std::string(argv[9]) == "r")
    {
      mode = MoveToDesiredJoints::Mode::SINGLE_RIGHT;
    }
    else if(std::string(argv[8]) == "-m" && std::string(argv[9]) == "b")
    {
      mode = MoveToDesiredJoints::Mode::BOTH;
    }
    else
    {
      ROS_ERROR("Wrong mode arguments, the command line arguments should be: j1 j2 j3 j4 j5 j6 j7 -m(mode) l(single left)/r(single right)/b(both)");
      return 0;
    }
  }
  else
  {
    ROS_ERROR("You are missing arguments, the command line arguments should be: j1 j2 j3 j4 j5 j6 j7 -m(mode) l(single left)/r(single right)/b(both)");
    return 0;
  }


  ros::NodeHandle n;
  float frequency = 100.0f;

  MoveToDesiredJoints moveToDesiredJoints(n,frequency, mode);

  if (!moveToDesiredJoints.init()) 
  {
    return -1;
  }
  else
  {
    moveToDesiredJoints.setDesiredJoints(desiredJoints);
    moveToDesiredJoints.run();
  }

  return 0;
}