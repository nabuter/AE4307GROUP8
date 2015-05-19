/* ------------------------------------------------------------------   */
/*      item            : EigenRigidBody.cxx
        made by         : Rene' van Paassen
        date            : 040414
	category        : body file 
        description     : 
	changes         : 040414 first version
        language        : C++
*/

const static char c_id[] =
"$Id: EigenRigidBody.cxx,v 1.1 2015/04/14 12:18:29 ostroosma Exp $";

#define EigenRigidBody_cxx
#include "EigenRigidBody.hxx"

#include <iostream>
using namespace std;

EigenRigidBody::EigenRigidBody(double mass, 
			       double Jxx, double Jyy, double Jzz, 
			       double Jxy, double Jxz, double Jyz, 
			       int extrastates) :
  x(13 + extrastates),
  iV(0.0),
  ialpha(0.0),
  ibeta(0.0),
  mass(mass)
{
  // inertia matrix
  Jn <<  Jxx, -Jxy, -Jxz,
    -Jxy,  Jyy, -Jyz,
    -Jxz, -Jyz,  Jzz;
  
  // and normalized
  Jn /= mass;
  
  // calculate inverse, put into Jinv matrix
  Jninv = Jn.inverse();

  // initialize state at pos 0, att 0 etc.
  x = Vector::Zero(13 + extrastates);
  x(12) = 1.0;
  moment.setZero();
  force.setZero();

  // calculate outputs
  output();
}

EigenRigidBody::~EigenRigidBody()
{
  // 
}


void EigenRigidBody::initialize(double x, double y, double z, 
				double u, double v, double w, 
				double phi, double theta, double psi,
				double p, double q, double r)
{
  this->x[ 0] = u;
  this->x[ 1] = v;
  this->x[ 2] = w;
  this->x[ 3] = x;
  this->x[ 4] = y;
  this->x[ 5] = z;
  this->x[ 6] = p;
  this->x[ 7] = q;
  this->x[ 8] = r;
  this->x[12] = cos(0.5*phi)*cos(0.5*theta)*cos(0.5*psi) +
    sin(0.5*phi)*sin(0.5*theta)*sin(0.5*psi);
  this->x[ 9] = sin(0.5*phi)*cos(0.5*theta)*cos(0.5*psi) -
    cos(0.5*phi)*sin(0.5*theta)*sin(0.5*psi); 
  this->x[10] = cos(0.5*phi)*sin(0.5*theta)*cos(0.5*psi) +
    sin(0.5*phi)*cos(0.5*theta)*sin(0.5*psi); 
  this->x[11] = cos(0.5*phi)*cos(0.5*theta)*sin(0.5*psi) -
    sin(0.5*phi)*sin(0.5*theta)*cos(0.5*psi);

  // calculate outputs
  output();
}

static void quat_der(const Vector& q, const Vector& omega, Vector4& dq)
{
  // in principle:
  // dq = 0.5 [ dot(-omega, q(2..4)) ;
  //       q(1) * omega - omega x q(2..4)]
  dq[3] = -0.5 * (omega[0]*q[0] + omega[1]*q[1] + omega[2]*q[2]);
  dq[0] =  0.5 * (q[3]*omega[0] - (omega[1]*q[2] - omega[2]*q[1]));
  dq[1] =  0.5 * (q[3]*omega[1] - (omega[2]*q[0] - omega[0]*q[2]));
  dq[2] =  0.5 * (q[3]*omega[2] - (omega[0]*q[1] - omega[1]*q[0]));
}

/** Calculate a 3by3 rotation matrix for a rotation defined by a
    quaternion. 
    \param q     The quaternion, in the form of 
                 (lambda_x, lambda_y, lambda_z, Lambda).
    \param Uq    Result, the rotation matrix. */
static void u_quat(const Vector& q, Matrix3& Uq)
{

  double u = sqrt(q.dot(q));
  double Au = q[0] / u;
  double Bu = q[1] / u;
  double Cu = q[2] / u;
  double Du = q[3] / u;
  // as per Stevens + Lewis, DCM, is transverse, inertial->body
  Uq(0,0) = Du*Du + Au*Au - Bu*Bu - Cu*Cu;
  Uq(0,1) = 2*(Au*Bu + Cu*Du);
  Uq(0,2) = 2*(Au*Cu - Bu*Du);
  Uq(1,0) = 2*(Au*Bu - Cu*Du);
  Uq(1,1) = Du*Du - Au*Au + Bu*Bu - Cu*Cu;
  Uq(1,2) = 2*(Bu*Cu + Au*Du);
  Uq(2,0) = 2*(Au*Cu + Bu*Du);
  Uq(2,1) = 2*(Bu*Cu - Au*Du);
  Uq(2,2) = Du*Du - Au*Au - Bu*Bu + Cu*Cu;
}

void EigenRigidBody::derivative(Vector& xd, double unused)
{
  // derivatives of u, v, w (x(0,2)
  // \dot{u} = force/mass - Omega u
  xd.head(3) = -1.0 * Omega * x.head(3) + force / mass;
  
  // add gravitational acceleration, Gravitation is in inertial, so
  // translate to the body, since we are considering body coordinate accel.
  xd.head(3) += _A * einstein;

  // derivative of the earth position, calculate from speed in body
  // coordinates, the speed in inertial coordinates, that is the
  // derivative
  // remember A is DCM, gives transformation from inertial to body,
  // have to go reverse here
  xd.segment(3,3) = _A * x.head(3);

  // derivative of the rotation speed vector \omega_b
  // \dot{\omega} = Jn^{-1} \left[ M - \Omega Jn \omega \right]
  xd.segment(6,3) = Jninv * (-1.0 * Omega * (Jn * x.segment(6,3)) + moment / mass);

  // derivative of the attitude quaternion, from rotational speed
  quat_der(x.segment(9,4), x.segment(6,3), dq); 
  // quat_der does not handle ranges
  xd.segment(9,4) = dq;
}

void EigenRigidBody::specific(Vector& sp)
{
  // specific forces, in body ax, in elements 0 - 2
  sp.head(3) = force / mass;

  // specific moment, in body ax, in elements 3 - 5
  sp.segment(3,3) = Jninv * moment / mass;
}

void EigenRigidBody::prepare() 
{
  // calculate rotation matrix
  u_quat(x.segment(9,4), _A);

  // calculate cross-product matrix
  Omega <<  0.0, -x(8), x(7),
            x(8), 0.0, -x(6),
           -x(7), x(6), 0.0;
}

void EigenRigidBody::zeroForces()
{
  // zero force and moment sums, and gravitation
  force.setZero();
  moment.setZero();
  einstein.setZero();
}

void EigenRigidBody::changeState(const Vector& dx)
{
  x += dx;
  prepare();
}

void EigenRigidBody::setState(const Vector& newx)
{
  x = newx;
  prepare();
}

void EigenRigidBody::changeMass(const double dm)
{
  mass += dm;
}

void EigenRigidBody::setMass(const double newm)
{
  mass = newm;
}

void EigenRigidBody::output()
{
  iV = sqrt(x[0]*x[0] + x[1]*x[1] + x[2]*x[2]);
  ialpha = atan2(x[2], x[0]);
  ibeta = atan2(x[1], x[0]);  
  
  // on the side, also normalize the quaternion
  Vector q = x.segment(9,4) / sqrt(x.segment(9,4).dot(x.segment(9,4)));
  
  double Au = q[0];
  double Bu = q[1];
  double Cu = q[2];
  double Du = q[3];

  iphi = atan2(2*(Bu*Cu + Au*Du), Du*Du - Au*Au - Bu*Bu + Cu*Cu);
  itheta = asin(-2.0*(Au*Cu - Bu*Du));
  ipsi = atan2(2*(Au*Bu + Cu*Du), Du*Du + Au*Au - Bu*Bu - Cu*Cu);
}

void EigenRigidBody::applyBodyForce(const Vector3& Fb, const Vector3& point)
{
  // sum the force
  force += Fb;
  // add the moment, cross product of force and position of exertion
  tmp0 = point.cross(Fb);
  applyBodyMoment(tmp0);
}

void EigenRigidBody::applyInertialForce(const Vector& Fi, const Vector& point)
{
  // translate to body axes and apply
  tmp1 = _A * Fi;
  applyBodyForce(tmp1, point);
}

void EigenRigidBody::applyBodyMoment(const Vector& M)
{
  moment += M;
}

void EigenRigidBody::addInertialGravity(const Vector& g)
{
  einstein += g;
}
