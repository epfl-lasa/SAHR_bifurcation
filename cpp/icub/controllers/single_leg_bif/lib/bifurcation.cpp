#include "bifurcation.h"

Bifurcation* Bifurcation::me = NULL;

// Bifurcation::Bifurcation(ros::NodeHandle &n, Parameters *p, float frequency) :
// 	_n(n),
// 	p(p),
//  	_loopRate(frequency)
// {
// 	this->dt = 1.0 / frequency;
// 	_stop = false;
// 	this->poseReceived = false;
// 	//this->xd = new float[p->getN()];     //initialized as Eigen::Vector3f
// 	this->vd = new float[p->getN()];

// 	me = this;
// }

Bifurcation::Bifurcation(Parameters *p, float frequency) :
 p(p),
 frequency(frequency)
 {
  this->dt = 1.0 / frequency;
  _stop = false;
  me = this;
 }

/*
int Bifurcation::init(){

  _subRealPose = _n.subscribe<geometry_msgs::Pose>("/lwr/ee_pose", 1, &Bifurcation::getPosVel, this ,ros::TransportHints().reliable().tcpNoDelay());
  _pubDesiredTwist = _n.advertise<geometry_msgs::Twist>("/lwr/joint_controllers/passive_ds_command_vel", 1);
  _pubDesiredOrientation = _n.advertise<geometry_msgs::Quaternion>("/lwr/joint_controllers/passive_ds_command_orient", 1);

  // Dynamic reconfigure definition
  _dynRecCallback = boost::bind(&Bifurcation::dynamicReconfigureCallback, this, _1, _2);
  _dynRecServer.setCallback(_dynRecCallback);
  signal(SIGINT,Bifurcation::stopNode);

	if (_n.ok())
	{ 
	  // Wait for callback to be called
		ros::spinOnce();
		ROS_INFO("The ros node is ready.");

    if(_saveToFile == true){
      //float* theta0 = p->getTheta0();
      float* a = p->getA();
      std::string params = std::to_string(p->getRho0()) + "_" + std::to_string(p->getM()) + "_" + std::to_string(p->getR()) + \
      "_a" + std::to_string(a[0]) + "_a" + std::to_string(a[1]) ; //+ "_th" + std::to_string(theta0[0]) + "_th" + std::to_string(theta0[1]) + \
      "_th" + std::to_string(theta0[2]);
      std::cout << params << std::endl;
      myfile.open ("/home/lasapc28/catkin_ws/bagfiles/as_savefile/" + params + "_position.csv");
      myfile << "time, x, y, z\n";
    }

		return 1;
	}
	else 
	{
		ROS_ERROR("The ros node has a problem.");
		return 0;
	}

}
*/

/*void Bifurcation::run(){
	while (!_stop) {
  	if(this->poseReceived) {
  		_mutex.lock();
    		// Compute control command

      updatePosVel();

    		// Publish data to topics
   		publishData();

 	 		_mutex.unlock();
  	}

  ros::spinOnce();

  _loopRate.sleep();
	}

	for(int i=0;i<3;i++)
    vd(i) = 0.0;

	publishData();
  myfile.close();

	ros::spinOnce();
	_loopRate.sleep();

	ros::shutdown();
}*/

/*
void Bifurcation::stopNode(int sig) {
	me->_stop = true;
}*/

/*
void Bifurcation::publishData()
{
  // Publish desired twist (passive ds controller)
  _msgDesiredTwist.linear.x  = this->vd[0];
  _msgDesiredTwist.linear.y  = this->vd[1];
  _msgDesiredTwist.linear.z  = this->vd[2];

  // Convert desired end effector frame angular velocity to world frame
  _msgDesiredTwist.angular.x = 0.0;
  _msgDesiredTwist.angular.y = 0.0;
  _msgDesiredTwist.angular.z = 0.0; 


  // Publish desired orientation
  // Considers current rotation angle around origin;
  // float* theta0 = this->p->getTheta0();
  // Eigen::Vector4f q = eul2quat(theta0[0],theta0[1]+4*atan(1),theta0[2]);    // add pi to pitch
  // _msgDesiredOrientation.w = 0.0;
  // _msgDesiredOrientation.x = 0.0;
  // _msgDesiredOrientation.y = 1.0;
  // _msgDesiredOrientation.z = 0.0;

  computeDesiredOrientation();

  _msgDesiredOrientation.w = _qd(0);//0.0;
  _msgDesiredOrientation.x = _qd(1);//0.0;
  _msgDesiredOrientation.y = _qd(2);//1.0;
  _msgDesiredOrientation.z = _qd(3);//0.0;

   _pubDesiredTwist.publish(_msgDesiredTwist);
   _pubDesiredOrientation.publish(_msgDesiredOrientation);
}*/

/*
void Bifurcation::getPosVel(const geometry_msgs::Pose::ConstPtr& msg) {
	// Get end effector position
  	x(0) = msg->position.x;
  	x(1) = msg->position.y;
  	x(2) = msg->position.z;
    myfile << ros::Time::now().toSec() << ", " << x(0) << ", " << x(1) << ", " << x(2) << "\n";

    _q << msg->orientation.w, msg->orientation.x, msg->orientation.y, msg->orientation.z;
    _wRb = Utils::quaternionToRotationMatrix(_q);
  	if(!this->poseReceived)
  		this->poseReceived = true;
}*/

/*
void Bifurcation::computeDesiredOrientation()
{
  //float* theta0 = this->p->getTheta0();
  //Eigen::Matrix3f rotMat = eul2rotmat(theta0[0],theta0[1],theta0[2]);
  // Get actual rotation matrix instead
  Eigen::Matrix3f rotMat = this->p->getRotMat();

  // Compute rotation error between current orientation and plane orientation using Rodrigues' law
  Eigen::Vector3f k,temp;
  temp = -rotMat.col(2);
  k = _wRb.col(2).cross(temp);
  float c = _wRb.col(2).transpose()*temp;  
  float s = k.norm();
  k /= s;
  
  Eigen::Matrix3f K;
  K << Utils::getSkewSymmetricMatrix(k);

  Eigen::Matrix3f Re;
  if(fabs(s)< FLT_EPSILON)
  {
    Re = Eigen::Matrix3f::Identity();
  }
  else
  {
    Re = Eigen::Matrix3f::Identity()+s*K+(1-c)*K*K;
  }
  
  // Convert rotation error into axis angle representation
  Eigen::Vector3f omega;
  float angle;
  Eigen::Vector4f qtemp = Utils::rotationMatrixToQuaternion(Re);
  Utils::quaternionToAxisAngle(qtemp,omega,angle);

  // Compute final quaternion on plane
  _qd = Utils::quaternionProduct(qtemp,_q);

  // // Compute needed angular velocity to perform the desired quaternion
  // Eigen::Vector4f qcurI, wq;
  // qcurI(0) = _q(0);
  // qcurI.segment(1,3) = -_q.segment(1,3);
  // wq = 5.0f*Utils::quaternionProduct(qcurI,_qd-_q);
  // Eigen::Vector3f omegaTemp = _wRb*wq.segment(1,3);
  // _omegad = omegaTemp; 
}*/

/* Function that receives the current cartesian position x and returns 
the next cartesian position using a dynamical system with bifurcation.
 */
void Bifurcation::updatePosVel() {

  int N = this->p->getN();
  float rho0 = this->p->getRho0();
  float M = this->p->getM();
  float R = this->p->getR();
  float* a = this->p->getA();
  float* x0 = this->p->getX0();
  Eigen::Matrix3f rotMat = this->p->getRotMat();

  // float max_v = 0.1;
  // float norm_v;

  float next_r[N], sphvel[N];
  Eigen::VectorXf x_shape(N);
  float x_shape2[N];

  for (int i=0; i<N; i++)
    x_shape(i) = a[i] * (this->x(i) + x0[i]);
  x_shape = rotMat.transpose() * x_shape;
  for (int i=0; i<N; i++)
    x_shape2[i] = x_shape(i);
  float* r = cart2hyper(x_shape2, 1, N);

  sphvel[0] = (-sqrt(2 * M) * (r[0] - rho0));
  for(int i=1; i<N; i++) {
    // new one-dimensional R
    if(i==N-1)
      sphvel[i] = (R * exp(-pow(2 * M * (r[0] - rho0),2)));
    else
      sphvel[i] = (-sqrt(2 * M) * r[i]);
  }

  // bound speed
  // norm_v = sqrt(pow(sphvel[0],2) + pow(sphvel[1],2));
  for(int i=0; i<N; i++) {
    // if(i<2){
    //   if(norm_v > max_v){
    //     sphvel[i] = sphvel[i] * max_v / norm_v;
    //   }
    // }
    next_r[i] = r[i] + sphvel[i] * this->dt;
  }

  this->xd = (Eigen::Vector3f) hyper2cart(next_r, 1, N);
  this->xd = rotMat * this->xd;
  for(int i=0; i<N; i++) {
    this->xd(i) /= a[i];
    this->xd(i) -= x0[i];
  }
  this->x = this->xd;

}

Eigen::Vector3f Bifurcation::nextPos(Eigen::Vector3f x_prev, float dt){
  //this->dt = dt;
  if (!poseReceived){
    this->x = x_prev;
    poseReceived = true;
  }
  updatePosVel();
  // std::cout << "X_des: " <<this->xd << std::endl;
  return this->xd;
}

/*
void Bifurcation::dynamicReconfigureCallback(bifurcation::parametersDynConfig &config, uint32_t level) {
  ROS_INFO("Reconfigure request. Updating the parameters ...");

  float anew[3], x0new[3]; //theta0new[3];
  Eigen::Matrix3f rotMatnew;
  //float Rnew[2];
  //Rnew[0] = config.R_1;
  //Rnew[1] = config.R_2;
  anew[0] = config.a_1;
  anew[1] = config.a_2;
  anew[2] = config.a_3;
  x0new[0] = -config.x0_1;
  x0new[1] = -config.x0_2;
  x0new[2] = -config.x0_3;
  //theta0new[0] = config.theta0_3;   // angle around z
  //theta0new[1] = config.theta0_2;   // angle around y
  //theta0new[2] = config.theta0_1;   // angle around x
  rotMatnew << config.rotMat_11, config.rotMat_12, config.rotMat_13,
              config.rotMat_21, config.rotMat_22, config.rotMat_23,
              config.rotMat_31, config.rotMat_32, config.rotMat_33;

  std::cerr << "Current radius and mass: " << config.rho0 << ", " << config.M << std::endl;
  std::cerr << "Current rotational speed: " << config.R << std::endl;
  std::cerr << "Current scaling: " << anew[0] << ", " << anew[1] << ", " << anew[2] << std::endl;
  std::cerr << "Current origin: " << x0new[0] << ", " << x0new[1] << ", " << x0new[2] << std::endl; 
  //std::cerr << "Current rotation around origin: " << theta0new[0] << ", " << theta0new[1] << ", " << theta0new[2] << std::endl; 
  std::cerr << "Current rotation matrix: " << rotMatnew << std::endl;

  this->p->setParameters(config.rho0, config.M, config.R, anew, x0new, rotMatnew);//theta0new);

}*/
