#include "ros/ros.h"
#include "std_msgs/String.h"
#include "bifurcation_iiwa.h"
#include "parameters.h"


int main(int argc, char **argv)
{
  // Ros initialization
  ros::init(argc, argv, "bifurcation_iiwa");

  ros::NodeHandle n;
  float frequency = 100.0f;
  int N = 3;

  float rho0, M, R;
  float a[3], x0[3], theta0[3];
  int useMocap = 1;

  if(argc==13){
    rho0 = atof(argv[1]);
    M = atof(argv[2]);
    R = atof(argv[3]);
    a[3], x0[3], theta0[3];
    for(int i=0; i<3; i++){
      a[i] = atof(argv[4+i]);
      x0[i] = atof(argv[7+i]);
      theta0[i] = atof(argv[10+i]);
    }
  }
  else if(argc==4){
    rho0 = atof(argv[1]);
    M = atof(argv[2]);
    R = atof(argv[3]);
    for(int i=0; i<3; i++){
      a[i] = 1;
      theta0[i] = 0;
    }
    x0[0] = 0.5;
    x0[1] = 0.1;
    x0[2] = -0.3;
  }

  else {
    if(argc==2){
      useMocap = atoi(argv[1]);
    }

    rho0 = 0.2;
    M = 1;
    R = 1;
    for(int i=0; i<3; i++){
      a[i] = 1;
      theta0[i] = 0;
    }
    x0[0] = 0.5;
    x0[1] = 0.1;
    x0[2] = -0.3;
  }

  std::cerr << "Current radius and mass: " << rho0 << ", " << M << std::endl;
  std::cerr << "Current rotational speed: " << R << std::endl;
  std::cerr << "Current scaling: " << a[0] << ", " << a[1] << ", " << a[2] << std::endl;
  std::cerr << "Current origin: " << x0[0] << ", " << x0[1] << ", " << x0[2] << std::endl; 
  std::cerr << "Current rotation angle: " << theta0[0] << ", " << theta0[1] << ", " << theta0[2] << std::endl; 

  Parameters *p = new Parameters(N, rho0, M, R, a, x0, theta0);

  Bifurcation bif(n,p,frequency,useMocap);

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
