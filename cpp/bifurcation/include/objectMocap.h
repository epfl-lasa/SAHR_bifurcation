#ifndef OBJECTMOCAP_H
#define OBJECTMOCAP_H
#include "Eigen/Eigen"
#include "Utils.h"
#include <cmath>

#define	NB_MARKERS 3

class ObjectMocap {
private:
    float rho0;     // Radius
    float* x0;      // Shift of origin, dimension N
    Eigen::Matrix3f rotMat;     // Rotation matrix instead of theta0

    Eigen::Matrix<float,NB_MARKERS,3> position = Eigen::Matrix<float,NB_MARKERS,3>::Zero();
    Eigen::Matrix<float,NB_MARKERS,4> orientation = Eigen::Matrix<float,NB_MARKERS,4>::Zero();

public:
	float getRho0();
    float* getX0();
    Eigen::Matrix3f getRotMat();

    ObjectMocap(Eigen::Matrix<float,NB_MARKERS,3> position, Eigen::Matrix<float,NB_MARKERS,4> orientation);
    void updateParameters(Eigen::Matrix<float,NB_MARKERS,3> position, Eigen::Matrix<float,NB_MARKERS,4> orientation);

};

#endif // OBJECTMOCAP_H