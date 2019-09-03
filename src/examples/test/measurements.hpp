#ifndef MEASUREMENTS_H
#define MEASUREMENTS_H

#include <uORB/topics/vehicle_attitude.h>
#include <uORB/topics/sensor_combined.h>
#include <uORB/topics/estimator_status.h>
#include <math.h>
#include <matrix/math.hpp>


#include "structs_main.h"
#include "rotations.hpp"
#include "print_debug.hpp"



class Measurements{

    public:

        // Constructor
        Measurements();

        //Main methods

        static void measurements_est(
            struct measurements_vec_s *measurements_vec,
            const struct vehicle_attitude_s *att, 
            const struct estimator_status_s *estimator_stat);
};


Measurements::Measurements(){}

void Measurements::measurements_est(
            struct measurements_vec_s *measurements_vec,
            const struct vehicle_attitude_s *att, 
            const struct estimator_status_s *estimator_stat){

        // measurements vector estimate [u, v, w, p, q, r, phi, theta, psi]';

        // Take Euler angles
        matrix::Eulerf att_euler = matrix::Quatf(att->q); 

        // Calculate the rotation matrix of body frame wrt inertial frame

        // Vector3f and Matrix3f data types
        typedef matrix::Matrix<float, 3, 1> vector3f;
        typedef matrix::Matrix<float, 3, 3> matrix3f;
        
        // Euler angles vector
        vector3f euler_angles;     
        euler_angles(0, 0) = att_euler.phi();
        euler_angles(1, 0) = att_euler.theta(); 
        euler_angles(2, 0) = att_euler.psi();

        Rotations<float, vector3f, matrix3f> rot;
        matrix3f rot_fb_fe = rot.euler_rot(euler_angles);
        matrix3f rot_fe_fb = rot_fb_fe.transpose();

        //Velocity inertial frame
        vector3f u_fe;
        u_fe(0, 0) = estimator_stat->states[4];
        u_fe(1, 0) = estimator_stat->states[5];
        u_fe(2, 0) = estimator_stat->states[6];
    
        // Velocity in body frame
        vector3f u_fb = rot_fe_fb*u_fe; 
        
        // Rotational velocity body frame
        vector3f w_fb;
        w_fb(0, 0) = att->rollspeed;
        w_fb(1, 0) = att->pitchspeed;
        w_fb(2, 0) = att->yawspeed;

        // Record measurement data to structure mesuremetns_vec
        measurements_vec->u = (float) u_fb(0, 0);
        measurements_vec->v = (float) u_fb(1, 0);
        measurements_vec->w = (float) u_fb(2, 0);

        measurements_vec->p = (float) w_fb(0, 0);
        measurements_vec->q = (float) w_fb(1, 0);
        measurements_vec->r = (float) w_fb(2, 0);

        measurements_vec->phi_ = (float) euler_angles(0, 0);
        measurements_vec->theta_ = (float) euler_angles(1, 0);
        measurements_vec->psi_ = (float) euler_angles(2, 0);
    }

#endif