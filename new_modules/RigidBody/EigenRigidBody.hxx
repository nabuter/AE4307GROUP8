/* ------------------------------------------------------------------   */
/*      item            : EigenRigidBody.hxx
        made by         : Rene van Paassen
        date            : 040414
	category        : header file 
        description     : Implements the dynamics of a rigid body. 
	                  After applying forces and moments on the
			  body, and specifying the gravity, one can
			  ask for an integration step.
	documentation   : DUECA_API
	changes         : 040414 first version
	                  060427 integration in DUECA-extra, 
        language        : C++
*/

#ifndef EigenRigidBody_hxx
#define EigenRigidBody_hxx

#ifdef EigenRigidBody_cxx
static const char h_id[] =
"$Id: EigenRigidBody.hxx,v 1.1 2015/04/14 12:18:29 ostroosma Exp $";
#endif

#include <Eigen/Dense>

/** \file EigenRigidBody.hxx Helper class for implementing rigid body
    vehicle simulation. */

// A 3x3 matrix, allocates its own storage
typedef Eigen::Matrix3d Matrix3;
// A 4x4 matrix, allocates its own storage
typedef Eigen::Matrix4d Matrix4;
// a normal matrix, allocates its own storage
typedef Eigen::MatrixXd Matrix;
// a matrix that takes external storage
typedef Eigen::Map<Eigen::MatrixXd> MatrixE;
// a 3x1 vector, allocates its own storage
typedef Eigen::Vector3d Vector3;
// a 4x1 vector, allocates its own storage
typedef Eigen::Vector4d Vector4;
// a normal vector, allocates its own storage
typedef Eigen::VectorXd Vector;
// a vector that takes external storage
typedef Eigen::Map<Eigen::VectorXd> VectorE;
 
/** Rigid body dynamics function, calculates derivative of a rigid
    body, given sum of moments and forces and the acting gravity
    field.
    
    The contents of the state vector are:

    u, v, w, x, y, z, p, q, r, l1, l2, l3, L

    <ul>
    <li> u, v, w are the velocity of the body, in body coordinates. 
    <li> x, y, z are the position of the body, in inertial reference
    coordinates. 
    <li> p, q, r are the components of the rotational speed vector, in
    body coordinates
    <li> l1, l2, l3, L are the elements of the quaternion describing
    the body attitude. 
    </ul>

    Note that the EigenRigidBody class only calculates the derivatives. If
    you want to integrate the equations of motion, you need an
    integration routine. Two templated integration functions are
    available, integrate_euler() and integrate_rungekutta(). 

    The normal procedure would be to create a class that derives from
    the EigenRigidBody class. You can also indicate that a number of
    additional state variables is needed, the EigenRigidBody class will
    make these available to you, from index 13 onward.  

    The normal "work cycle" would be to call EigenRigidBody::zeroForces(),
    which zeroes forces and gravity applied to the body, then
    repeatedly call EigenRigidBody::applyBodyForce(),
    EigenRigidBody::applyInertialForce(), EigenRigidBody::applyBodyMoment()
    EigenRigidBody::addInertialGravity() as needed, until all forces (drag,
    lift, wheel forces, thrusters, whatever) and all gravitational
    effects have been applied.

    Then call  EigenRigidBody::derivative() to obtain the derivative. 

    EigenRigidBody::specific(), EigenRigidBody::X() and EigenRigidBody::phi(), 
    EigenRigidBody::theta(), EigenRigidBody::psi() can be used to get your
    outputs. 
 */
class EigenRigidBody
{
protected:
  /** State vector. Contents: u, v, w, x, y, z, 
      then p, q, r, l1, l2, l3, L.
      After this, thus from state 13 onwards, the state vector
      contains states for derived classes.*/
  Vector x;

private:
  /** Auxiliary output. \{ */
  /** Speed. */
  double iV;

  /** Angle of attack. */
  double ialpha;

  /** Sideslip angle. */
  double ibeta; 

  /** Euler angles. \{ */
  double iphi, itheta, ipsi; /// \}

  /** \} */

  /** Properties of the rigid body. */
  /// \{
  /** Body mass. */
  double mass;

  /** Normalized inertia matrix. */
  Matrix3 Jn;

  /** Inverse normalized inertia matrix */
  Matrix3 Jninv;
  /// \}
  
  /** Working variables. */
  /// \{

  /** Attitude conversion matrix, earth-to-body (its transpose does
      the reverse). */
  Matrix3 _A;

  /** Rotation cross-product matrix. */
  Matrix3 Omega;

  /** Summed moments for this step, body axis. */
  Vector3 moment;

  /** Summed forces for this step, body axis. */
  Vector3 force;

  /** Two temporary vectors, 3 elt each. \{ */
  Vector3 tmp0, tmp1; /// \}

  /** Temporary vector for quat derivative. */
  Vector4 dq;
  
  /** Gravitational field, in inertial coordinates. */
  Vector3 einstein;

  /// \} 
public:
  /** Simple constructor for a rigid body dynamics module. Note
      that motion is always described around center of mass. 
      \param mass    mass of the object
      \param Jxx     Moment of inertia around x axis
      \param Jyy     Moment of inertia around y axis
      \param Jzz     Moment of inertia around z axis
      \param Jxy     Inertia cross product x and y
      \param Jxz     Inertia cross product x and z
      \param Jyz     Inertia cross product y and z
      \param extrastates Number of additional states needed by derived
                     classes. 
  */
  EigenRigidBody(double mass, 
	    double Jxx, double Jyy, double Jzz, 
	    double Jxy, double Jxz, double Jyz, int extrastates = 0);
  
  /** Destructor. */
  ~EigenRigidBody();
  
public:
  /** Initialize a state vector x. Remember that x has 13 elements,
      due to the use of a quaternion representation.
      \param x       Carthesian position, x direction
      \param y       Carthesian position, y direction
      \param z       Carthesian position, z direction
      \param u       Velocity, along body x axis
      \param v       Velocity, along body y axis
      \param w       Velocity, along body z axis
      \param phi     Euler angle phi
      \param theta   Euler angle theta
      \param psi     Euler angle psi
      \param p       Rotational velocity, along body x axis
      \param q       Rotational velocity, along body y axis
      \param r       Rotational velocity, along body z axis
  */
  void initialize(double x, double y, double z, 
		  double u, double v, double w, 
		  double phi, double theta, double psi,
		  double p, double q, double r);
  
  /** Obtain and calculate the derivative, given sum of forces and moments
      and the gravitational field acting on the body. 
      \param xd      Resultant derivative vector
      \param unused  Unused variable, for compatibility with
                     integration functions. 
      \returns       Reference to the vector with the derivative
                     state.  */
  void derivative(Vector& xd, double unused = 0.0);

  /** Calculate specific forces and moments, forces stored in elements
      0, 1, and 2, moments in elements 3, 4 and 6. Forces and moments
      calculated in body coordinates.
      \param sp      Vector with 6 elements, for result. */
  void specific(Vector& sp);

  /** Initialize sum of forces on the body to zero. Call before
      applying a new set of forces and moments. */
  void zeroForces();

  /** Apply a force expressed in body coordinates. at a specific body
      point. */
  void applyBodyForce(const Vector3& Fb, const Vector3& point);

  /** Apply a force expressed in inertial coordinates, at a specific
      body point, give in body coordinates. */
  void applyInertialForce(const Vector& Fi, const Vector& point);
  
  /** Apply a moment expressed in body coordinates. */
  void applyBodyMoment(const Vector& M);

  /** Change the state vector by a quantity dx. */
  void changeState(const Vector& dx);

  /** Add a gravitational field; 3 element vector, gravitation
      expressed in the inertial system. */
  void addInertialGravity(const Vector& g);

  /** Put in a new state vector. Note that you need a 13-element
      vector */
  void setState(const Vector& newx);

  /** Add to the mass of the body. */
  void changeMass(const double dm);

  /** Set a new mass. */
  void setMass(const double newm);

  /** Calculate auxiliary outputs, V, \f$\alpha\f$, \f$\beta\f$, \f$\phi\f$,
      \f$\theta\f$ and \f$\Psi\f$. */
  void output();

  /** Obtain the state, 13 elements. */
  inline const Vector& X() const {return x;} 

  /** Obtain speed. */
  inline const double V() const {return iV;}

  /** Obtain alpha. */
  inline const double alpha() const {return ialpha;}
  
  /** Obtain beta. */
  inline const double beta() const {return ibeta;}

  /** Get current mass. */
  inline const double getMass() const {return mass;}

  /** Get euler angle phi */
  inline const double phi() const {return iphi;}
  /** Get euler angle theta */
  inline const double theta() const {return itheta;}
  /** Get euler angle psi */
  inline const double psi() const {return ipsi;}\

  /** Get the transformation matrix, for transforming a vector in
      earth coordinates to one in the body's coordinate system. 
      
      Note that for the reverse translation, you need the transpose of
      this matrix. 
  */
  inline const Matrix3& A() { return _A; }

private:
  /** Prepare for the next iteration step. Call this before applying
      any forces or moment on the body */
  void prepare();
};

#endif
