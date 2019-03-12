#include <cmath>
#include <cstdio>
#include <algorithm>
#include <iterator>
#include <vector>
#include <iostream>
#include "Eigen/Eigen"

float* findVelocities(float data[], int M, int N, int startPnts[], int K, float dt);
float* removeFinalDatasample(float data[], int M, int N, int startPnts[], int K);
float* cart2hyper(float data[], int M, int N);
float* hyper2cart(float hyper[], int M, int N);
float* cart2sphvel(float data[], float hyper[], float vel[], int M, int N, int startPnts[], int K);
float* sph2cartvel(float hyper[], float sphVel[], int M, int N);
Eigen::Matrix3f eul2rotmat(float x, float y, float z);
Eigen::Vector4f eul2quat(float yaw, float pitch, float roll);
//float* rotm2eul(Eigen::Matrix3f RotMatrix);			//not needed
//float* matrixLeftDivide(float leftmat, float x);		//done through eigen
//float* matrixLeftMultiply(float leftmat, float x);	//done through eigen

// Remember to delete the vectors from stack when done!