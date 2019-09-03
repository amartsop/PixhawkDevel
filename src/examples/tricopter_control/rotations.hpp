#ifndef ROTATIONS_H
#define ROTATIONS_H

template <class L, class M, class N>
class Rotations{
    public:
        // Constructor
        Rotations();

        // Main methods
        N rotation_x(L x);
        N rotation_y(L x);
        N rotation_z(L x);
        N euler_rot(M x);
        N rotation_angular(M x);
        N skew_sym(M x);
};

template <class L, class M, class N>
Rotations<L, M, N>::Rotations(){};

/*
Calculates the basic rotation matrix with respect to x axis.
-x is the angle of rotation in rad.
-returns: the basic rotation matrix.
 */
template <class L, class M, class N>
N Rotations<L, M, N>::rotation_x(L x){

    N y; y(0, 0) = 1; y(0, 1) = 0; y(0, 2) = 0;
    y(1, 0) = 0; y(1, 1) = cos(x); y(1, 2) = -sin(x);
    y(2, 0) = 0; y(2, 1) = sin(x); y(2, 2) = cos(x); 
    return y;
}

/*
Calculates the basic rotation matrix with respect to y axis.
-x is the angle of rotation in rad.
-returns: the basic rotation matrix.
 */
template <class L, class M, class N>
N Rotations<L, M, N>::rotation_y(L x){
    N y; y(0, 0) = cos(x); y(0, 1) = 0; y(0, 2) = sin(x);
    y(1, 0) = 0; y(1, 1) = 1; y(1, 2) = 0;
    y(2, 0) = -sin(x); y(2, 1) = 0; y(2, 2) = cos(x);
    return y;
}

/*
Calculates the basic rotation matrix with respect to z axis.
-x is the angle of rotation in rad.
-returns: the basic rotation matrix.
 */
template <class L, class M, class N>
N Rotations<L, M, N>::rotation_z(L x){
    N y; y(0, 0) = cos(x); y(0, 1) = -sin(x); y(0, 2) = 0;
    y(1, 0) = sin(x); y(1, 1) = cos(x); y(1, 2) = 0;
    y(2, 0) = 0; y(2, 1) = 0; y(2, 2) = 1;
    return y;
}

/*
Calculates the rotation matrix of a frame with respect to another.
-x is a 3D vector containing the euler angles.
-returns: the rotation matrix using z-y'-x'' euler convention.
 */

template <class L, class M, class N>
N Rotations<L, M, N>::euler_rot(M x){
   
    L phi_ = (L) x(0, 0); L theta_ = (L) x(1, 0); L psi_ = (L) x(2, 0);
    N r_x = rotation_x(phi_); N r_y = rotation_y(theta_); 
    N r_z = rotation_z(psi_); N y = r_z*r_y*r_x; return y;
}

/*
Calculates the rotation matrix that maps angular velocity from 
    body/wind frame to the inertial frame.
-x is a 3D vector containing the euler angles
 */
template <class L, class M, class N>
N Rotations<L, M, N>::rotation_angular(M x){
    L phi_ = (L) x(0, 0); L theta_ = (L) x(1, 0);
    N y; y(0, 0) = 1; y(0, 1) =  sin(phi_)*tan(theta_); 
    y(0, 2) = cos(phi_)*tan(theta_); y(1, 0) = 0; y(1, 1) = cos(phi_);
    y(1, 2) = -sin(phi_); y(2, 0) = 0; y(2, 1) = sin(phi_)/cos(theta_); 
    y(2, 2) = cos(phi_)/cos(theta_); return y;
}

/*
Calculates the skew symmetric matrix of x vector.
-x is a 3D vector.
*/
template <class L, class M, class N>
N Rotations<L, M, N>::skew_sym(M x){
    N y; memset(&y, 0, sizeof(y)); y(0, 1) = -x(2, 0); y(0, 2) = x(1, 0);
    y(1, 0) = x(2, 0); y(1, 2) = -x(0, 0); y(2, 0) = -x(1, 0); y(2, 1) = x(0, 0);
    return y;
}

#endif