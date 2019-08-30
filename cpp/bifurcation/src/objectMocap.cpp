#include "objectMocap.h"

ObjectMocap::ObjectMocap(Eigen::Matrix<float,NB_MARKERS,3> position, Eigen::Matrix<float,NB_MARKERS,4> orientation)
{
	this->position = position;
	this->orientation = orientation;
	if(!this->position.isZero()){
		if(NB_MARKERS == 3) {		// markers positioned as an equilateral triangle (cycle inscribed)
			float a[3] = {(position.row(1)-position.row(0)).norm(),(position.row(2)-position.row(1)).norm(),(position.row(0)-position.row(2)).norm()};
			// Find radius of cycle (rho0)
			float p = a[0] + a[1] + a[2];
			this->rho0 = sqrt(((p-a[0])*(p-a[1])*(p-a[2]))/p);
			std::cerr << "Side lengths: " << a[0] << ", " << a[1] << ", " << a[2] << std::endl;
			std::cerr << "Calculated radius: " << this->rho0 << std::endl;
			// Find centre of cycle (x0)
			this->x0 = new float[3];
			for(int i=0; i<3; i++){
				this->x0[i] = (a[0]*position(0,i) + a[1]*position(1,i) + a[2]*position(2,i)) / (a[0]+a[1]+a[2]);
			}
			std::cerr << "Calculated centre of cycle: " << x0[0] << ", " << x0[1] << ", " << x0[2] << std::endl;
			// Find plane and rotation matrix (rotMat)
			this->rotMat = Utils::quaternionToRotationMatrix(orientation.row(1));
			std::cerr << "Calculated orientation: " << this->rotMat << std::endl;
		}
		else {		// first 4 markers are considered to be in a square, with 1st and 3rd being opposite
			// Find radius of cycle (rho0) as half the distance between first 2 markers
			this->rho0 = (position.row(1)-position.row(0)).norm()/2;
			// Find centre of mass (x0)
			this->x0 = new float[3];
			for(int i=0; i<3; i++){
				this->x0[i] = (position(0,i) + position(2,i)) / 2;
			}
			// Find plane and rotation matrix (rotMat)
			this->rotMat = Utils::quaternionToRotationMatrix(orientation.row(1));
		}
	}
}

void ObjectMocap::updateParameters(Eigen::Matrix<float,NB_MARKERS,3> position, Eigen::Matrix<float,NB_MARKERS,4> orientation) {
	if(!position.isZero()){
		if(NB_MARKERS == 3) {		// markers positioned as an equilateral triangle (cycle inscribed)
			float a[3] = {(position.row(1)-position.row(0)).norm(),(position.row(2)-position.row(1)).norm(),(position.row(0)-position.row(2)).norm()};
			// Find radius of cycle (rho0)
			float p = a[0] + a[1] + a[2];
			this->rho0 = sqrt(((p-a[0])*(p-a[1])*(p-a[2]))/p);
			// Find centre of cycle (x0)
			this->x0 = new float[3];
			for(int i=0; i<3; i++){
				this->x0[i] = (a[0]*position(0,i) + a[1]*position(1,i) + a[2]*position(2,i)) / (a[0]+a[1]+a[2]);
			}
			// Find plane and rotation matrix (rotMat)
			this->rotMat = Utils::quaternionToRotationMatrix(orientation.row(1));
		}
		else {		// first 4 markers are considered to be in a square, with 1st and 3rd being opposite
			// Find radius of cycle (rho0) as half the distance between first 2 markers
			this->rho0 = (position.row(1)-position.row(0)).norm()/2;
			// Find centre of mass (x0)
			this->x0 = new float[3];
			for(int i=0; i<3; i++){
				this->x0[i] = (position(0,i) + position(2,i)) / 2;
			}
			// Find plane and rotation matrix (rotMat)
			this->rotMat = Utils::quaternionToRotationMatrix(orientation.row(1));
		}
	}
}

float ObjectMocap::getRho0(){
	return this->rho0;
}

float* ObjectMocap::getX0(){
	return this->x0;
}

Eigen::Matrix3f ObjectMocap::getRotMat(){
	return this->rotMat;
}

