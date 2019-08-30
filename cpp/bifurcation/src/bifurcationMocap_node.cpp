#include "ros/ros.h"
#include "std_msgs/String.h"
#include "bifurcationMocap.h"
#include "parameters.h"


int main(int argc, char **argv)
{
  // Ros initialization
  ros::init(argc, argv, "bifurcationMocap");

  ros::NodeHandle n;
  float frequency = 100.0f;
  int N = 3;

  if(argc==13){
    float rho0 = atof(argv[1]);
    float M = atof(argv[2]);
    float R = atof(argv[3]);
    float a[3], x0[3], theta0[3];
    for(int i=0; i<3; i++){
      a[i] = atof(argv[4+i]);
      x0[i] = atof(argv[7+i]);
      theta0[i] = atof(argv[10+i]);
    }
  }
  else if(argc==4){
    float rho0 = atof(argv[1]);
    float M = atof(argv[2]);
    float R = atof(argv[3]);
    float a[3] = {1,1,1};
    float x0[3] = {0.5,0.1,-0.3};
    float theta0[3] = {0,0,0};
  }
  else{
    float rho0 = 0.2;
    float M = 1;
    float R = 1;
    float a[3] = {1,1,1};
    float x0[3] = {0.5,0.1,-0.3};
    float theta0[3] = {0,0,0};
  }

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
