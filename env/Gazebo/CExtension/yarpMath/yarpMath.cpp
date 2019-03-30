

#include <yarp/math/Math.h>
#include <yarp/math/SVD.h>
#include <yarp/eigen/Eigen.h>
#include <yarp/sig/Matrix.h>
#include <vector>
#include <Eigen/Eigenvalues>
#include <iostream>
#include <cmath>
#include <cassert>
 
using namespace yarp::eigen;
using namespace yarp::math;
using namespace yarp::sig;
using namespace std;

// this struct is used for dewrapping the swig object
typedef struct {
  PyObject_HEAD
  void *ptr; // This is the pointer to the actual C++ instance
  void *ty;  // swig_type_info originally, but shouldn't matter
  int own;
  PyObject *next;
} SwigPyObject;


 std::vector<double> dcmToAxis(void* M)
 {
    
    Matrix* R_ptr=reinterpret_cast<Matrix*>(M);
    
    Matrix R=*R_ptr;
    
     std::vector<double> ret (4);
     
 
    Vector v(4);
     v[0]=R(2,1)-R(1,2);
     v[1]=R(0,2)-R(2,0);
     v[2]=R(1,0)-R(0,1);
     v[3]=0.0;
     double r=yarp::math::norm(v);
     double theta=atan2(0.5*r,0.5*(R(0,0)+R(1,1)+R(2,2)-1));
 
 
     if (r<1e-9)
     {
         // if we enter here, then
         // R is symmetric; this can
         // happen only if the rotation
         // angle is 0 (R=I) or 180 degrees
         Matrix A=R.submatrix(0,2,0,2);
         Matrix U(3,3), V(3,3);
         Vector S(3);
 
         // A=I+sin(theta)*S+(1-cos(theta))*S^2
         // where S is the skew matrix.
         // Given a point x, A*x is the rotated one,
         // hence if Ax=x then x belongs to the rotation
         // axis. We have therefore to find the kernel of
         // the linear application (A-I).
         SVD(A-eye(3,3),U,S,V);
 
         v[0]=V(0,2);
         v[1]=V(1,2);
         v[2]=V(2,2);
         r=yarp::math::norm(v);
     }
 
     v=(1.0/r)*v;
     v[3]=theta;

     ret[0]=v[0];
     ret[1]=v[1];
     ret[2]=v[2];
     ret[3]=v[3];

 
     return ret;
 }





 

   



