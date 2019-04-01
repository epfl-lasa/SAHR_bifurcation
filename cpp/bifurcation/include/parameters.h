#ifndef PARAMETERS_H
#define PARAMETERS_H
#include "transformations.h"
#include "Eigen/Eigen"


class Parameters {

private:
    int N;          // Dimension of data
    float rho0;     // Radius
    float M;        // Mass
    float R;        // Rotation speed and direction, dimension N-1
    float* a;       // Scaling, dimension N
    float* x0;      // Shift of origin, dimension N
    float* theta0;  // Rotation angles around the origin
    Eigen::Matrix3f rotMat;     // Rotation matrix instead of theta0

public:
    Parameters(int N);
    Parameters(int N, float rho0, float M, float R);
    Parameters(int N, float rho0, float M, float R, float a[], float x0[]);
    Parameters(int N, float rho0, float M, float R, float a[], float x0[], float theta0[]);
    Parameters(int N, float rho0, float M, float R, float a[], float x0[], Eigen::Matrix3f rotMat);
    ~Parameters();

    void setParameters(float rho0, float M, float R, float a[], float x0[], float theta0[]);
    void setParameters(float rho0, float M, float R, float a[], float x0[], Eigen::Matrix3f rotMat);

    void changeRho0(float rho0);
    void changeM(float M);
    void changeR(float R);
    void changeA(float a[]);
    void changeX0(float x0[]);
    void changeTheta0(float theta0[]);
    void changeRotMat(Eigen::Matrix3f rotMat);

    int getN();
    float getRho0();
    float getM();
    float getR();
    float* getA();
    float* getX0();
    float* getTheta0();
    Eigen::Matrix3f getRotMat();
};


#endif // PARAMETERS_H
