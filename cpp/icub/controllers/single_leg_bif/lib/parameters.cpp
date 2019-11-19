#include "parameters.h"


Parameters::Parameters(int N) {

    this->N = N;
    // Set other parameters to default parameters:
    this->rho0 = 1;
    this->M = 1;
    // Initialize arrays
    //this->R = new float[N-1];
    this->a = new float[N];
    this->x0 = new float[N];
    this->theta0 = new float[N];
    //for(int i=0; i<N-1; i++)
    //    this->R[i] = 1;
    this-> R = 1;       // new R one-dimensional
    for(int i=0; i<N; i++) {
        this->a[i] = 1;
        this->x0[i] = 0;
        this->theta0[i] = 0;
    }
    this->rotMat = Eigen::Matrix3f::Identity(); 
}

Parameters::Parameters(int N, float rho0, float M, float R) {

    this->N = N;
    this->rho0 = rho0;
    this->M = M;
    // Initialize arrays
    //this->R = new float[N-1];
    this->a = new float[N];
    this->x0 = new float[N];
    this->theta0 = new float[N];
    this->R = R;
    // Set other parameters to default parameters:
    for(int i=0; i<N; i++) {
        this->a[i] = 1;
        this->x0[i] = 0;
        this->theta0[i] = 0;
    }
    this->rotMat = Eigen::Matrix3f::Identity(); 
}

Parameters::Parameters(int N, float rho0, float M, float R, float a[], float x0[]) {

    this->N = N;
    this->rho0 = rho0;
    this->M = M;
    // Initiliaze arrays
    //this->R = new float[N-1];
    this->a = new float[N];
    this->x0 = new float[N];
    this->theta0 = new float[N];
    this->R = R;
    this->a = a;
    this->x0 = x0;
    // Set other parameters to default parameters:
    for(int i=0; i<N; i++) {
        this->theta0[i] = 0;
    }
    this->rotMat = Eigen::Matrix3f::Identity(); 
}


Parameters::Parameters(int N, float rho0, float M, float R, float a[], float x0[], float theta0[]) {
    
    this->N = N;
    this->rho0 = rho0;
    this->M = M;
    this->R = R;
    // Initiliaze arrays
    this->a = new float[N];
    this->x0 = new float[N];
    this->theta0 = new float[N];
    this->a = a;
    this->x0 = x0;
    this->theta0 = theta0;
    this->rotMat = Eigen::Matrix3f::Identity(); 
}

Parameters::Parameters(int N, float rho0, float M, float R, float a[], float x0[], Eigen::Matrix3f rotMat) {
    
    this->N = N;
    this->rho0 = rho0;
    this->M = M;
    this->R = R;
    // Initiliaze arrays
    this->a = new float[N];
    this->x0 = new float[N];
    this->a = a;
    this->x0 = x0;
    this->rotMat = rotMat;
}

Parameters::Parameters(int N, const char* ParamsFilename) {

    this->N = N;
    FILE * params = fopen(ParamsFilename,"r");
    this->a = new float[N];
    this->x0 = new float[N];
    this->rotMat = Eigen::Matrix3f::Identity();
    float tmp;

    if (params!=NULL) {
        fscanf(params,"%f",&this->rho0);
        fscanf(params,"%f",&this->M);
        fscanf(params,"%f",&this->R);
        fscanf(params,"%f",&this->a[0]);
        fscanf(params,"%f",&this->a[1]);
        fscanf(params,"%f",&this->a[2]);
        fscanf(params,"%f",&this->x0[0]);
        fscanf(params,"%f",&this->x0[1]);
        fscanf(params,"%f",&this->x0[2]);
        for(int i=0;i<3;i++){
            for(int j=0;j<3;j++){
                fscanf(params,"%f",&tmp);
                this->rotMat(i,j) = tmp;
            }
        }
        fclose(params);
    }
    else {
        std::cout << "File not valid. Parameters set to standard values." << std::endl;
        this->rho0 = 0.1;
        this->M = 1;
        this-> R = 1;
        for(int i=0; i<N; i++) {
            this->a[i] = 1;
            this->x0[i] = 0;
        }
    }        
}


Parameters::~Parameters() {

}

/* Allows to set all parameters at a different time than at initialization (e.g. after optimization).
 */
void Parameters::setParameters(float rho0, float M, float R, float a[], float x0[], float theta0[]) {

    this->rho0 = rho0;
    this->M = M;
    this->R = R;        // new R one-dimensional
    for (int i=0; i<this->N; i++) {
        //if (i < N-1)
        //    this->R[i] = R[i];
        this->a[i] = a[i];
        this->x0[i] = x0[i];
        this->theta0[i] = theta0[i];
    } 
}

/* Allows to set all parameters at a different time than at initialization (e.g. after optimization).
 */
void Parameters::setParameters(float rho0, float M, float R, float a[], float x0[], Eigen::Matrix3f rotMat) {

    this->rho0 = rho0;
    this->M = M;
    this->R = R;        // new R one-dimensional
    for (int i=0; i<this->N; i++) {
        //if (i < N-1)
        //    this->R[i] = R[i];
        this->a[i] = a[i];
        this->x0[i] = x0[i];
    }
    this->rotMat = rotMat;
}


/* Allows to change radius rho0.
 */
void Parameters::changeRho0(float rho0) {
    this->rho0 = rho0;
}

/* Allows to change mass M.
 */
void Parameters::changeM(float M) {
    this->M = M;
}

/* Allows to change rotation R.
 */
void Parameters::changeR(float R) {
    this->R = R;
}

/* Allows to change scaling a.
 */
void Parameters::changeA(float a[]) {
    this->a = a;
}

/* Allows to change shift of origin x0.
 */
void Parameters::changeX0(float x0[]) {
    this->x0 = x0;
}

/* Allows to change rotation angles around origin.
 */
void Parameters::changeTheta0(float theta0[]) {
    this->theta0 = theta0;
}
// Allows to change rotation matrix.
void Parameters::changeRotMat(Eigen::Matrix3f rotMat) {
    this->rotMat = rotMat;
}   


/* Return each parameter.W
 */
int Parameters::getN() {
    return this->N;
}

float Parameters::getRho0() {
    return this->rho0;
}

float Parameters::getM() {
    return this->M;
}

float Parameters::getR() {
    return this->R;
}

float* Parameters::getA() {
    return this->a;
}

float* Parameters::getX0() {
    return this->x0;
}

float* Parameters::getTheta0() {
    return this->theta0;
}

Eigen::Matrix3f Parameters::getRotMat() {
    return this->rotMat;
}

