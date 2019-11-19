#include "transformations.h"
#include <algorithm>


/* Calculates velocities from data points.
 * Size of array of data is (M*N), with N columns added subsequently to the array.
 * Starting point of each trajectory is saved in startPnts.
 * Time difference between samples in dt.
 */
float* findVelocities(float data[], int M, int N, int startPnts[], int K, float dt){

    float *vel = new float[(M-K)*N];

    for(int k=0; k<K; k++){
        int start = startPnts[k];
        int end;
        if(k == K-1)
            end = M;
        else
            end = startPnts[k+1];

        for(int i=start+1; i<end; i++){
            for(int j=0; j<N; j++){
                vel[i-k-1 + (M-K)*j] = (data[i + M*j] - data[i-1 + M*j]) / dt;
            }
        }
    }

    return vel;
}


/* Reduces data dimension to fit calculated velocities' size from M*N to (M-k)*N,
 * where k is the number of trajectories. Removes last sample of each trajectory.
 */
float* removeFinalDatasample(float data[], int M, int N, int startPnts[], int K) {

    float *newData = new float[(M-K)*N];

    for(int k=0; k<K; k++){
        int start = startPnts[k];
        int end;
        if(k == K-1)
            end = M;
        else
            end = startPnts[k+1];

        for(int i=start; i<end-1; i++){
            for(int j=0; j<N; j++){
                newData[i-k + (M-K)*j] = data[i + M*j];
            }
        }
    }

    return newData;

}


/* Calculates hyperspherical coordinates hyper[M*N] from cartesian coordinates in data[M*N].
 */
float* cart2hyper(float data[], int M, int N){

    float *hyper = new float[M*N];
    float sumSquared;		// Stores the sum of squared dimensions of data[i]
    if(N < 2)
        return hyper;   // Can't calculate it if N<2

    for(int i=0; i<M; i++){
        sumSquared = pow(data[i],2.0);                  // Add power 2 of first cartesian dimension
        for(int j=1; j<N; j++){
            sumSquared += pow(data[i + M*j],2.0);       // Add power 2 of all other cartesian dimensions
            if(j == 1)
                // Calculate last hyperspherical dimension
                hyper[i + M*(N-j)] = atan2(data[i + M*j],data[i + M*(j-1)]);
            else
                // Calculate intermediate hyperspherical dimensions
                hyper[i + M*(N-j)] = asin(data[i + M*j]/sqrt(sumSquared));
        }
        hyper[i] = sqrt(sumSquared);                    // Calculate first hyperspherical dimension
    }

    return hyper;
}


/* Calculates cartesian coordinates data[M*N] from hyperspherical coordinates in hyper[M*N].
 */
float* hyper2cart(float hyper[], int M, int N){

    float *data = new float[M*N];
    float prodCosDim;		// Stores the product of the cosine of different dimensions of hyper[i]
    if(N < 2)
        return data;   // Can't calculate it if N<2

    for(int i=0; i<M; i++){
        prodCosDim = 1;
        for(int j=1; j<N; j++){
            if(j == 1)
                // Calculate last cartesian dimension using also radius (hyper[i])
                data[i + M*(N-j)] = hyper[i] * sin(hyper[i + M*j]);
            else
                // Calculate intermediate cartesian dimensions using also radius (hyper[i])
                data[i + M*(N-j)] = hyper[i] * prodCosDim * sin(hyper[i + M*j]);
            // Multiply cosine of all hyperspherical dimensions but radius
            prodCosDim = prodCosDim * cos(hyper[i + M*j]);
        }
        data[i] = hyper[i] * prodCosDim;           // Calculate first cartesian dimension
    }

    return data;
}


/* Calculates spherical/polar velocities sphVel[M*N] from cartesian coordinates in data[M*N],
 * spherical/polar coordinates in hyper[M*N] and cartesian velocities in vel[M*N].
 */
float* cart2sphvel(float data[], float hyper[], float vel[], int M, int N, int startPnts[], int K){

    float *sphVel = new float[M*N];
    if(N < 2 || !sphVel){
        printf("\nError in cart2sphvel.\n");
        return sphVel;
    }
    int multiplier = 1;         // Negative if sign of 2nd dimension (altitude) in 3D needs to be inverted
    float pi = 4*atan(1);

    std::vector<int> start(K,0);  // To allow operations on vectors
    for (int i=0; i<K; i++)
        start[i] = startPnts[i];

    float sumSquared;           // Stores sum of dimensions squared;
    float sumCartTimesVel;      // Stores sum of each dimension multiplied by corresponding velocity

    if(N < 2)
        return sphVel;   // Can't calculate it if N<2

    for(int i=0; i<M; i++){
        sumSquared = pow(data[i],2.0);                  // Add power 2 of first cartesian dimension
        // Add multiplication of first dimension coordinate and velocity
        sumCartTimesVel = data[i]*vel[i];
        for(int j=1; j<N; j++){
            if (j == 1)
                sumSquared += pow(data[i + M*j],2.0);       // Add power 2 of second dimension
            // Add multiplication of other dimensions coordinates and velocities
            sumCartTimesVel += data[i + M*j]*vel[i + M*j];
            if(j == N-2) {
                // Check whether i is at the beginning of a trajectory: if negative, change sign of 2nd dimension's velocity

                printf("\nIs i at the beginning of a trajectory? %d", std::any_of(start.begin(), start.end(), [&](int y){return y==i;}));

                if(std::any_of(start.begin(), start.end(), [&](int y){return y==i;})) {
                    if (hyper[i + M*2] < 0 || hyper[i + M*2] == pi)
                        multiplier = -1;
                    else
                        multiplier = 1;
                }
                else {
                    // Check whether there is a change of sign between subsequent samples in both 1st and 2nd dimensions
                    if (std::signbit(data[i-1]*data[i]) && (std::signbit(data[i-1 + M]*data[i + M]) || data[i + M] == 0) ) {
                        if (multiplier == 1)
                            multiplier = -1;
                        else
                            multiplier = 1;
                    }
                }
                sphVel[i + M*j] = - multiplier * (data[i + M*2]*sumCartTimesVel - vel[i + M*2]*sumSquared)
                        / ((sumSquared + pow(data[i + M*2],2.0)) * sqrt(sumSquared));
            }
            else if (j == N-1) {
                sphVel[i + M*j] = (vel[i + M]*data[i] - data[i + M]*vel[i]) / sumSquared;
                if (N == 3)
                    sumSquared += pow(data[i + M*j],2.0);          // Add power 2 of third dimension if N == 3
            }
        }
        sphVel[i] = sumCartTimesVel / sqrt(sumSquared);
    }

    return sphVel;
}


/* Calculates cartesian velocities vel[M*N] from spherical/polar velocities sphVel[M*N] and
 * spherical/polar coordinates hyper[M*N].
 */
float* sph2cartvel(float hyper[], float sphVel[], int M, int N){

    float *vel = new float[M*N];
    if(N < 2 || !sphVel){
        printf("\nError in sph2cartvel.\n");
        return vel;
    }

    for(int i = 0; i < M; i++) {
        if(N == 3) {
            // First cartesian velocity
            vel[i] = cos(hyper[i + M]) * cos(hyper[i + M*2]) * sphVel[i] - hyper[i] * sin(hyper[i + M]) * cos(hyper[i + M*2]) * sphVel[i + M] - \
                hyper[i] * cos(hyper[i + M]) * sin(hyper[i + M*2]) * sphVel[i + M*2];
            //Second cartesian velocity
            vel[i + M] = cos(hyper[i + M]) * sin(hyper[i + M*2]) * sphVel[i] - hyper[i] * sin(hyper[i + M]) * sin(hyper[i + M*2]) * sphVel[i + M] + \
                hyper[i] * cos(hyper[i + M]) * cos(hyper[i + M*2]) * sphVel[i + M*2];
            //Third cartesian velocity
            vel[i + M*2] = sin(hyper[i + M]) * sphVel[i] + hyper[i] * cos(hyper[i + M]) * sphVel[i + M];
        }
        else if (N == 2) {
            // First cartesian velocity
            vel[i] = cos(hyper[i + M]) * sphVel[i] - hyper[i] * sin(hyper[i + M]) * sphVel[i + M];
            //Second cartesian velocity
            vel[i + M] = sin(hyper[i + M]) * sphVel[i] + hyper[i] * cos(hyper[i + M]) * sphVel[i + M];
        }
    }

    return vel;
}


/* Calculates 3x3 rotation matrix given the euler angles or rotation, around x, y and z (ordered as roll-pitch-yaw).
*/
Eigen::Matrix3f eul2rotmat(float z, float y, float x){

	Eigen::Matrix3f RotMatrix = Eigen::Matrix3f::Identity();
	if(x != 0 || y != 0 || z != 0) {
		Eigen::Vector3f c;
		Eigen::Vector3f s;
		c(0) = cos(z);
		c(1) = cos(y);
		c(2) = cos(x);
		s(0) = sin(z);
		s(1) = sin(y);
		s(2) = sin(x);

		RotMatrix(0,0) = c(1)*c(0);
		RotMatrix(0,1) = s(2)*s(1)*c(0) - c(2)*s(0);
		RotMatrix(0,2) = c(2)*s(1)*c(0) + s(2)*s(0);
		RotMatrix(1,0) = c(1)*s(0);
		RotMatrix(1,1) = s(2)*s(1)*s(0) + c(2)*c(0);
		RotMatrix(1,2) = c(2)*s(1)*s(0) - s(2)*c(0);
		RotMatrix(2,0) = -s(1);
		RotMatrix(2,1) = s(2)*c(1);
		RotMatrix(2,2) = c(2)*c(1);
	}
	return RotMatrix;
}

/* Calculates euler angles (ordered as roll-pitch-yaw) given the 3x3 rotation matrix.
*/
/*float* rotm2eul(Eigen::Matrix3f RotMatrix){

	float eps = 0.00001;
	float angles[3] = {0,0,0};
	if (!RotMatrix.isIdentity(eps)) {
		float sy = sqrt(RotMatrix(1,1)*RotMatrix(1,1) + RotMatrix(2,1)*RotMatrix(2,1));
		if(sy < eps) {
			angles[3] = atan2(-RotMatrix(2,3),RotMatrix(2,2));
			angles[2] = atan2(-RotMatrix(3,1),sy);
		}
		else{
			angles[3] = atan2(RotMatrix(3,2),RotMatrix(3,3));		// psi: around x
			angles[2] = atan2(-RotMatrix(3,1),sy);					// theta: around y
			angles[1] = atan2(RotMatrix(2,1),RotMatrix(1,1));		// phi: around z
		}
	}
	return angles;
}
*/

/* Calculates quaternion given the euler angles or rotation, around x, y and z (ordered as roll-pitch-yaw).
*/
Eigen::Vector4f eul2quat(float yaw, float pitch, float roll){

    float cy = cos(yaw * 0.5);
    float sy = sin(yaw * 0.5);
    float cp = cos(pitch * 0.5);
    float sp = sin(pitch * 0.5);
    float cr = cos(roll * 0.5);
    float sr = sin(roll * 0.5);

    Eigen::Vector4f q;
    q(0) = cy * cp * cr + sy * sp * sr;
    q(1) = cy * cp * sr - sy * sp * cr;
    q(2) = sy * cp * sr + cy * sp * cr;
    q(3) = sy * cp * cr - cy * sp * sr;
    return q;
}