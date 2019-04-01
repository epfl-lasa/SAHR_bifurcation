#include "ros/ros.h"
#include "std_msgs/String.h"
#include "bifurcation.h"
#include "parameters.h"


int main(int argc, char **argv)
{
  // Ros initialization
  ros::init(argc, argv, "bifurcation");

  ros::NodeHandle n;
  float frequency = 100.0f;
  int N = 3;

  float rho0 = 0.2;
  float M = 1;
  float R = 1; //R[2] = {1,1};
  float a[3] = {1,1,1};
  float x0[3] = {0.5,0.1,-0.3};
  float theta0[3] = {0,0,0};

  // if(argc >= 5) {
  //   rho0 = atof(argv[1]);
  //   M = atof(argv[2]);
  //   R[0] = atof(argv[3]);
  //   R[1] = atof(argv[4]);
  // }
  // if(argc >= 11) {
  //   for(int i=0; i<3; i++){
  //     a[i] = atof(argv[5+i]);
  //     x0[i] = atof(argv[8+i]);
  //   }
  // }

  // std::cerr << "Current radius and mass: " << rho0 << ", " << M << std::endl;
  // std::cerr << "Current rotational speed: " << R[0] << ", " << R[1] << std::endl;
  // std::cerr << "Current scaling: " << a[0] << ", " << a[1] << ", " << a[2] << std::endl;
  // std::cerr << "Current origin: " << x0[0] << ", " << x0[1] << ", " << x0[2] << std::endl; 

  Parameters *p = new Parameters(N, rho0, M, R, a, x0, theta0);

  Bifurcation bif(n,p,frequency);

  if (!bif.init()) 
  {
    return -1;
  }
  else
  {
    bif.run();
  }


  delete p;

  return 0;

};
