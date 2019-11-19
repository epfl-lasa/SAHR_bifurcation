/*!
 * \file Contact_Manager.hpp
 *
 * \author Salman Faraji
 * \date 10.2.2015
 *
 * In this file a list of all interesting contacts is defined
 */

#pragma once
#include "Model.h"

# define NUM_Contact 5
enum Contact_Name       {CN_CM=0, CN_LF, CN_RF, CN_LH, CN_RH};
enum Point_Status       {PS_NO_CONTROL=0, PS_CONTACTED, PS_FLOATING};
enum leg_state          {STATE_AIR = -2, STATE_SSR, STATE_DS, STATE_SSL, STATE_QUAD};


/*!
 * \class Geom_Point
 *
 * \brief Contains geometric information about a Cartesian point
 *
 * Includes position, velocity and acceleration
 *
 * \author Salman Faraji
 * \date 10.2.2015
 */
struct Geom_Point{
    Geom_Point() {pos=zero_v3; vel=zero_v3; acc=zero_v3;}
    Cvector3 pos;
    Cvector3 vel;
    Cvector3 acc;
};

/*!
 * \class Geom_Rot
 *
 * \brief Contains geometric information about a Cartesian rotation
 *
 * Includes position, velocity and acceleration
 *
 * \author Salman Faraji
 * \date 10.2.2015
 */
struct Geom_Rot{
    Geom_Rot() {pos=zero_quat; vel=zero_v3; acc=zero_v3;}
    Cvector4 pos;
    Cvector3 vel;
    Cvector3 acc;
};

/*!
* \brief Prints a geometric point
* \param[in] in the geometric point
* \param[in] str the name label
*/
extern void Geom_Point_print(Geom_Point in, __const char *str);
/*!
* \brief Prints two geometric points by putting corresponding elements beside
* \param[in] str the name
* \param[in] ref reference geometric point
* \param[in] actual actual geometric point
*/
extern void print_comparitive(__const char *str, Geom_Point ref, Geom_Point actual);

////////////////////////////////////////////////////////////////////////////////////////////////////////////

/*!
* \brief Apply exponential A*(1-exp(-(t/tau)^2))
* \param[in] A amplitude
* \param[in] t current time
* \param[in] tau time constant
* \param[out] pos position
* \param[out] vel velocity
* \param[out] acc acceleration
* \warning Adds to the previous values
*/
extern void apply_exp(double A, double t, double tau, double * pos, double * vel, double * acc);
/*!
* \brief Apply exponential A*(1-exp(-(t/tau)^2))
* \param[in] A amplitude
* \param[in] t current time
* \param[in] tau time constant
* \param[out] pos position
* \param[out] vel velocity
* \param[out] acc acceleration
* \param[in] index the X,Y,Z direction regarding ::Dir_Axis
* \warning Adds to the previous values
*/
extern void apply_exp(double A, double t, double tau, Cvector& pos, Cvector& vel, Cvector& acc, int index);
/*!
* \brief Apply exponential A*(1-exp(-(t/tau)^2))
* \param[in] A amplitude
* \param[in] t current time
* \param[in] tau time constant
* \param[out] in geometric point
* \param[in] index the X,Y,Z direction regarding ::Dir_Axis
* \warning Adds to the previous values
*/
extern void apply_exp(double A, double t, double tau, Geom_Point &in, Dir_Axis index);

////////////////////////////////////////////////////////////////////////////////////////////////////////////

/*!
* \brief Apply sinusoidal A*sin(w*t+phi)
* \param[in] A amplitude
* \param[in] t current time
* \param[in] w frequency
* \param[in] phi phase
* \param[out] pos position
* \param[out] vel velocity
* \param[out] acc acceleration
* \warning Adds to the previous values
*/
extern void apply_sin(double A, double t, double w, double phi, double * pos, double * vel, double * acc);
/*!
* \brief Apply sinusoidal A*sin(w*t+phi)
* \param[in] A amplitude
* \param[in] t current time
* \param[in] w frequency
* \param[in] phi phase
* \param[out] pos position
* \param[out] vel velocity
* \param[out] acc acceleration
* \param[in] index the X,Y,Z direction regarding ::Dir_Axis
* \warning Adds to the previous values
*/
extern void apply_sin(double A, double t, double w, double phi, Cvector& pos, Cvector& vel, Cvector& acc, int index);
/*!
* \brief Apply sinusoidal A*sin(w*t+phi)
* \param[in] A amplitude
* \param[in] t current time
* \param[in] w frequency
* \param[in] phi phase
* \param[out] in geometric point
* \param[in] index the X,Y,Z direction regarding ::Dir_Axis
* \warning Adds to the previous values
*/
extern void apply_sin(double A, double t, double w, double phi, Geom_Point &in, Dir_Axis index);

////////////////////////////////////////////////////////////////////////////////////////////////////////////

/*!
* \brief Apply sinusoidal A*sin(w*t)^2
* \param[in] A amplitude
* \param[in] t current time
* \param[in] w frequency
* \param[out] pos position
* \param[out] vel velocity
* \param[out] acc acceleration
* \warning Adds to the previous values
*/
extern void apply_sin2(double A, double t, double w, double * pos, double * vel, double * acc);
/*!
* \brief Apply sinusoidal A*sin(w*t)^2
* \param[in] A amplitude
* \param[in] t current time
* \param[in] w frequency
* \param[out] in geometric point
* \param[in] index the X,Y,Z direction regarding ::Dir_Axis
* \warning Adds to the previous values
*/
extern void apply_sin2(double A, double t, double w, Geom_Point &in, Dir_Axis index);

////////////////////////////////////////////////////////////////////////////////////////////////////////////

/*!
* \brief Apply zero pos/vel/acc arc of sin
* \param[in] A amplitude
* \param[in] t current time
* \param[in] w frequency
* \param[out] pos position
* \param[out] vel velocity
* \param[out] acc acceleration
* \warning Adds to the previous values
*/
extern void apply_sin3(double A, double t, double w, double * pos, double * vel, double * acc);
/*!
* \brief Apply zero pos/vel/acc arc of sin
* \param[in] A amplitude
* \param[in] t current time
* \param[in] w frequency
* \param[out] in geometric point
* \param[in] index the X,Y,Z direction regarding ::Dir_Axis
* \warning Adds to the previous values
*/
extern void apply_sin3(double A, double t, double w, Geom_Point &in, Dir_Axis index);

////////////////////////////////////////////////////////////////////////////////////////////////////////////

/*!
* \brief Apply exponential sinusoidal A*sin(w*t+phi)*(1-exp(-(t/tau)^2))
* \param[in] A amplitude
* \param[in] t current time
* \param[in] tau time constant
* \param[in] w frequency
* \param[in] phi phase
* \param[out] pos position
* \param[out] vel velocity
* \param[out] acc acceleration
* \warning Adds to the previous values
*/
extern void apply_expsin(double t, double A, double tau, double w, double phi, double* pos, double* vel, double* acc);
/*!
* \brief Apply exponential sinusoidal A*sin(w*t+phi)*(1-exp(-(t/tau)^2))
* \param[in] A amplitude
* \param[in] t current time
* \param[in] tau time constant
* \param[in] w frequency
* \param[in] phi phase
* \param[out] pos position
* \param[out] vel velocity
* \param[out] acc acceleration
* \param[in] index the X,Y,Z direction regarding ::Dir_Axis
* \warning Adds to the previous values
*/
extern void apply_expsin(double t, double A, double tau, double w, double phi, Cvector3& pos, Cvector3& vel, Cvector3& acc, int index);
/*!
* \brief Apply exponential sinusoidal A*sin(w*t+phi)*(1-exp(-(t/tau)^2))
* \param[in] A amplitude
* \param[in] t current time
* \param[in] tau time constant
* \param[in] w frequency
* \param[in] phi phase
* \param[out] in geometric point
* \param[in] index the X,Y,Z direction regarding ::Dir_Axis
* \warning Adds to the previous values
*/
extern void apply_expsin(double t, double A, double tau, double w, double phi, Geom_Point &in, Dir_Axis index);

////////////////////////////////////////////////////////////////////////////////////////////////////////////

/*!
* \brief Apply exponentially smoothed arbitrary function: A*f(t)*(1-exp(-(t/tau)^2))
* \param[in] A amplitude
* \param[in] t current time
* \param[in] tau time constant
* \param[in] f function value at time t
* \param[in] df function derivative at time t
* \param[in] ddf function double derivative at time t
* \param[out] pos position
* \param[out] vel velocity
* \param[out] acc acceleration
* \param[in] index the X,Y,Z direction regarding ::Dir_Axis
* \warning Adds to the previous values
*/
extern void apply_fexp(double t, double A, double tau, double f, double df, double ddf, Cvector& pos, Cvector& vel, Cvector& acc, int index);

////////////////////////////////////////////////////////////////////////////////////////////////////////////

/*!
* \brief Exponential transition between two different positions/velocities/accelerations
* \param[in] t the time passed from starting time of transition
* \param[in] T total duration of transition
* \param[in] tau time constant of transition
* \param[in] p initial position
* \param[in] v initial velocity
* \param[in] a initial acceleration
* \param[in] p_des final position
* \param[in] v_des final velocity
* \param[in] a_des final acceleration
* \param[out] p_out position
* \param[out] v_out velocity
* \param[out] a_out acceleration
*/
extern void exp_transition(double t, double tau,
                    double p, double v, double a,
                    double p_des, double v_des, double a_des,
                    double &p_out, double &v_out, double &a_out);

/*!
* \brief Exponential transition between two different positions/velocities/accelerations
* \param[in] t the time passed from starting time of transition
* \param[in] T total duration of transition
* \param[in] tau time constant of transition
* \param[in] p initial position
* \param[in] v initial velocity
* \param[in] a initial acceleration
* \param[in] p_des final position
* \param[in] v_des final velocity
* \param[in] a_des final acceleration
* \param[out] p_out position
* \param[out] v_out velocity
* \param[out] a_out acceleration
*/
extern void exp_transition(double t, double tau,
                    Cvector3 p, Cvector3 v, Cvector3 a,
                    Cvector3 p_des, Cvector3 v_des, Cvector3 a_des,
                    Cvector3 &p_out, Cvector3 &v_out, Cvector3 &a_out);
/*!
* \brief Exponential transition between two different positions/velocities/accelerations
* \param[in] t the time passed from starting time of transition
* \param[in] T total duration of transition
* \param[in] tau time constant of transition
* \param[in] init initial positions
* \param[in] end final positions
* \param[out] out resulting geometric point
*/
extern void exp_transition(double t, double tau,  Geom_Point & init, Geom_Point & end, Geom_Point & out);

////////////////////////////////////////////////////////////////////////////////////////////////////////////

/*!
* \brief Apply slerp transition between two geometric points using exponential seed
* \param[in] t the time passed from starting time of transition
* \param[in] T total duration of transition
* \param[in] tau time constant of transition
* \param[in] init initial positions
* \param[in] end final positions
* \param[out] out resulting geometric point
*/
extern void slerp(double t, double T, double tau, Geom_Point & init, Geom_Point & end, Geom_Point & out);
/*!
* \brief Apply slerp transition between two geometric points using exponential seed
* \param[in] t the time passed from starting time of transition
* \param[in] T total duration of transition
* \param[in] tau time constant of transition
* \param[in] init initial positions
* \param[in] end final positions
* \param[out] out resulting geometric point
* \param[in] index the X,Y,Z direction regarding ::Dir_Axis
*/
extern void slerp(double t, double T, double tau, Geom_Point & init, Geom_Point & end, Geom_Point & out, Dir_Axis index);
/*!
* \brief Apply slerp transition between two geometric points using sinusoidal seed
* \param[in] t the time passed from starting time of transition
* \param[in] T total duration of transition
* \param[in] init initial positions
* \param[in] end final positions
* \param[out] out resulting geometric point
*/
extern void slerp_sin(double t, double T, Geom_Point & init, Geom_Point & end, Geom_Point & out);
/*!
* \brief Apply slerp transition between two geometric points using sinusoidal seed
* \param[in] t the time passed from starting time of transition
* \param[in] T total duration of transition
* \param[in] init initial positions
* \param[in] end final positions
* \param[out] out resulting geometric point
* \param[in] index the X,Y,Z direction regarding ::Dir_Axis
*/
extern void slerp_sin(double t, double T, Geom_Point & init, Geom_Point & end, Geom_Point & out, Dir_Axis index);
/*!
* \brief Apply transition between two geometric points using polynomial seed
* \param[in] t the time passed from starting time of transition
* \param[in] T total duration of transition
* \param[in] init initial positions
* \param[in] end final positions
* \param[out] out resulting geometric point
*/
extern void slerp_poly(double t, double T, Geom_Point & init, Geom_Point & end, Geom_Point & out);
/*!
* \brief Apply transition between two geometric points using polynomial seed
* \param[in] t the time passed from starting time of transition
* \param[in] T total duration of transition
* \param[in] init initial positions
* \param[in] end final positions
* \param[out] out resulting geometric point
* \param[in] index the X,Y,Z direction regarding ::Dir_Axis
*/
extern void slerp_poly(double t, double T, Geom_Point & init, Geom_Point & end, Geom_Point & out, Dir_Axis index);
/*!
* \brief Exponential transition between two different orientations/velocities/accelerations using quaternions
* \param[in] t the time passed from starting time of transition
* \param[in] T total duration of transition
* \param[in] tau time constant of transition
* \param[in] init initial positions
* \param[in] end final positions
* \param[out] out resulting geometric point
*/
extern void exp_transition_quat(double t, double T, double tau, Geom_Rot & init, Geom_Rot & end, Geom_Rot & out);


/*!
 * \class Dyn_Point
 *
 * \brief Contains dynamic information about a Cartesian point.
 *
 * Note that one needs to define two instances of this class for a point which has both translational and rotational motion
 *
 * Includes:
 * - Jacobian and its derivative
 * - Measured, desired and estimated forces
 * - Costs for force and acceleration
 * - PID gains (K)
 *
 * \author Salman Faraji
 * \date 10.2.2015
 */
struct Dyn_Point
{
    Dyn_Point() {acc_ref=zero_v3;  dJdq=zero_v3;  F_ref=zero_v3;  F_sens=zero_v3;  F_est=zero_v3;  slack=zero_v3;  F_cost=zero_v3;  K=zero_v3;  slack_cost=zero_v3; }
    // variables
    //! The desired 3D acceleration
    Cvector3 acc_ref;
    //! The corresponding Jacobian
    Cmatrix J;
    //! The derivative of jacobian times the derivative of state vector
    Cvector3 dJdq;
    //! Desired force at the contact
    Cvector3 F_ref;
    //! Measured force at the contact (if there is a sensor)
    Cvector3 F_sens;
    //! Estimated force, coming out of inverse dynamics (should ideally be equal to Dyn_Point::F_ref if Dyn_Point::F_cost is high)
    Cvector3 F_est;
    //! Estimated slack, coming out of inverse dynamics (should ideally be equal to zero if Dyn_Point::F_cost is high)
    Cvector3 slack;

    //constants
    //! The cost of force tracking in inverse dynamics. If high, then Dyn_Point::F_est tends to Dyn_Point::F_ref.
    Cvector3 F_cost;
    //! The PID gains for controlling the joint in Cartesian space. ** Note the notation is P-D-I in the 3D vector
    Cvector3 K;
    //! The slack cost of the desired Cartesian acceleration Dyn_Point::acc_ref in the inverse dynamics. If low, Dyn_Point::acc_ref is not realized in the output.
    Cvector3 slack_cost;
};

/*!
 * \class Contact
 *
 * \brief Contains basic information about a contact
 *
 * In this class we include all the information about a contact point in the robot.
 *
 * \author Salman Faraji
 * \date 10.2.2015
 */
class Contact
{
public:
    Contact_Name name;

    //! The Cartesian point is located on which body of the robot
    int body;

    //! The offset of the point wrt. the body mass center
    Cvector3 offset;

    //! Determines if this contact is translational, rotational or both
    constraint_type contact_type;

    //! Determines if the contact is active, deactive
    Point_Status status;

    //! If this cpntact is used for odometery
    bool used_for_odometry;

    //! Initial position
    Geom_Point init_p;

    //! Initial orientation
    Geom_Rot init_o;

    //! Actual position
    Geom_Point p;

    //! Actual orientation
    Geom_Rot o;

    //! Reference position
    Geom_Point ref_p;

    //! Reference orientation
    Geom_Rot ref_o;

    //! Translational dynamnics information
    Dyn_Point T;

    //! Rotational dynamnics information
    Dyn_Point R;

    //! The Center of Pressure of the joint, calculated in perception level
    Cvector3 cop;

    //! Contact frame X direction
    Cvector3 n1;

    //! Contact frame Y direction
    Cvector3 n2;

    //! Contact frame Z direction (should be the contact normal used to calculate the friction polyhedral)
    Cvector3 n3;

    //! Contact length (along X direction), approximated with rectangle if having a surface
    Cvector3 w_x;

    //! Contact width (along Y direction), approximated with rectangle if having a surface
    Cvector3 w_y;

    //! Coefficient of translational friction
    double mu;

    //! Coefficient of rotational friction, if having a surface
    double muR;

    // Constructor, Destructor
    Contact();
    ~Contact();

    /*!
    * \brief initializing a contact with most important information
    *
    * \param[in] Name the contact name among the list of ::Contact_Name
    * \param[in] Body the index of the body link on the robot
    * \param[in] Offset The offset of the point with respect to the body link, expressed in the link coordinates frame
    * \param[in] Contact_type the type of this contact (and number of constraints) among the list of ::constraint_type
    * \param[in] Status The control policy for this point chosen from ::Point_Status
    * \param[in] KP the translational PID gains
    * \param[in] KO the rotational PID gains
    * \note if a point is mearly translational, the the geometry, rotational friction and cop will be ignored
    * \warning First calls Contact::initialize_contact and then overrides all the values
    */
    void init(  Contact_Name Name, 
				int Body, 
				Cvector3 Offset,
				constraint_type Contact_type, 
				Point_Status Status, 
				double length,
				double width);

    /*!
    * \brief Set all variables to the default values
    * \warning Look at the implementation for more details
    */
    void initialize_contact();
};

/*!
 * \class Contact_Manager
 *
 * \brief The list of contact points as well as updating and plotting functions
 *
 * [detailed description]
 *
 * \author Salman Faraji
 * \date 10.2.2015
 */
class Contact_Manager
{
public:

    //! The main vector of Cartesian points
    std::vector<Contact> C;

    //! A copy of the robot's model
    Model * M;

	// check true if you use COM commands instead of pelvis
	bool ifCoM;

    // Constructor, destructor
    Contact_Manager();
    ~Contact_Manager();

    /*!
    * \brief Initialize the array of Cartesian points
    *
    * All the points are initialized by default values
    *
    * \param[in] model the pointer to the robots model, needed for calculating jacobians
    * \param[in] size the number of contact points considered
    */
    void initialize(Model * model, int size);

    /*!
    * \brief Record initial positions
    *
    * In the first time the controller is called, this function records the geometric status of all the contacts.
    *
    * \warning sets velocities and accelerations to zero
    */
    void save_initial_contacts();

    /*!
    * \brief Update information for all contacts after perception is done
    *
    * Since sdfast does not calculate the derivative of the Jcobian, we use a sample joint acceleration
    * vector to calculate Cartesian accelerations and the subtract Jacobian * dq. Note that the term
    * Dyn_Point::dJdq is independent of accelerations. Therefore a zero acceleration vector already works.
    */
    void update_kinematics();

	/*!
    * \param[in] acc the double derivative of the robot's state vector. This vector is not used
    * \note The acc vector is not used for the moment
    * \warning The calculations in this function are mostly related to loading information from the robot's model
    */
    void update_dynamics();

    /*!
    * \brief Returns size of the array
    */
    unsigned int size();

    /*!
    * \brief Indexing function for the array of contacts
    */
    Contact &operator [](int n) { return C.data()[n];}

    /*!
    * \brief Concatenate all Cartesian variables
    *
    * For the Cartesian points which are in contact or floating, this function concatenates all geometric information
    *
    * \param[out] J all the jacobians
    * \param[out] dJdq all the Dyn_Point::dJdq
    * \param[out] acc desired accelerations (either translational or rotational
    * \param[out] slack all the slack costs for these constraints
    *
    * \warning The order of concatenation is point after point. This function does not separate translational and rotational constraints.
    */
    void tasks(Cmatrix &J, Cvector &dJdq, Cvector &acc, Cvector &slack);

    /*!
    * \brief Calculates the summation of all jacobian transpose times contact forces
    *
    * \return Cvector summation of all external forces in the Equation om Motion
    * \note Used for checking purposes
    * \warning after inverse dynamics calculations
    */
    Cvector get_mult();

	/*!
    * \brief Calculates the summation of all jacobian transpose times virtual contact forces
    *
    * \return Cvector summation of all external forces in the Equation om Motion
    * \note Used for checking purposes
    * \warning after inverse dynamics calculations
    */
    Cvector get_virtual_mult();

    /*!
    * \brief Control a Cartesian point to the desired position/velocity/acceleration
    *
    * \param[in] index the contact index in the list of ::Contact_Name
    * \param[in] ref the desired 3D position/velocity/acceleration
    * \warning Integrator is disabled for the moment
    */
    void control_position(Contact_Name index, Geom_Point ref);

    /*!
    * \brief Control a Cartesian point to the desired orientaion/velocity/acceleration
    *
    * This function translates the current motion into the frame of desired motion and then reduces the error
    *
    * \param[in] index the contact index in the list of ::Contact_Name
    * \param[in] ref the desired 3D orientaion/velocity/acceleration
    * \warning Integrator is disabled for the moment
    */
    void control_orientation(Contact_Name index, Geom_Rot ref);

    /*!
    * \brief Control a Cartesian point to the desired position/velocity/acceleration
    *
    * Combines Contact_Manager::control_position and Contact_Manager::control_orientation
    *
    * \param[in] index the contact index in the list of ::Contact_Name
    * \param[in] ref_pos the desired 3D position/velocity/acceleration
    * \param[in] ref_ori the desired 3D orientaion/velocity/acceleration
    */
    void control(Contact_Name index, Geom_Point ref_pos, Geom_Rot ref_ori, bool if_force);

    /*!
    * \brief Printing all estimated forces Dyn_Point::F_est after inverse dynamics
    */
    void print_forces();

    /*!
    * \brief Printing all desired accelerations Dyn_Point::acc_ref before inverse dynamics
    */
    void print_acc_ref();

	/*!
    * \brief Printing all inverse kinematics errors
    */
	void print_IK_errors();

	/*!
    * \brief Printing all inverse dynamics errors
    */
	void print_ID_errors();
};

