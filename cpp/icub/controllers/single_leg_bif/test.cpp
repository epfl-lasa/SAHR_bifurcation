#include <string>
#include <cstdio>
#include <iostream>
#include <fstream>
#include <cmath>
#include <signal.h>
#include <unistd.h>
#include "bifurcation.h"
#include "Eigen/Eigen"

using namespace std;

int main(int argc, char *argv[])
{
	// logger
	std::ofstream OutRecord;
	OutRecord.open("/home/lasapc28/catkin_ws/logs_icub/testlog.csv");

	Eigen::Vector3f pos(0,-0.065,0);
	Eigen::Vector3f prev;

	// initialize bifurcation and parameters (from txt file with parameters)
	Parameters *p = new Parameters(3, "/home/lasapc28/catkin_ws/logs_icub/params.txt");
	cout << p->getRho0() << ", " << p->getM() << ", " << p->getR() << endl;
	float *a = p->getA();
	float *x0 = p->getX0();
	cout << a[0] << ", " << a[1] << ", " << a[2] << endl;
	cout << x0[0] << ", " << x0[1] << ", " << x0[2] << endl;
	cout << p->getRotMat() << endl;
	Bifurcation bif(p,114.0f);
	float dt = 1/114.0;
	double time = 0;
	
	// start the loop
	while(time < 10)
	{
		time += dt;
		prev = pos;
		if(time < 5){
			pos = Eigen::Vector3f(0.35*time/5.0, -0.065 + (-0.035*time/5.0), 0.45*time/5.0);
		}
		else{
			pos = bif.nextPos(prev,dt);
		}

		OutRecord << time << ", " << pos(0) << ", " << pos(1) << ", " << pos(2) << endl;
	}

	OutRecord.close();
	delete p;
	return 0;
}

