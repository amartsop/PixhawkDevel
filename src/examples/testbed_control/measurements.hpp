#ifndef MEASUREMENTS_H
#define MEASUREMENTS_H

#include <uORB/topics/vehicle_attitude.h>
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
            const struct sensor_bias_s *sen_bias);
        
        static void measurements_init(
            struct measurements_vec_s *measurements_vec,
            const struct vehicle_attitude_s *att, 
            const struct sensor_bias_s *sen_bias);

    private:
        
        static void unwrapping(float *x, float y);

        static void meas_acceleration(struct measurements_vec_s *measurements_vec, 
            const struct sensor_bias_s *sen_bias);
};


Measurements::Measurements(){}

void Measurements::measurements_est(
            struct measurements_vec_s *measurements_vec,
            const struct vehicle_attitude_s *att, 
            const struct sensor_bias_s *sen_bias){

    // measurements vector estimate [u, v, w, p, q, r, phi, theta, psi]';

    // Take Euler angles
    matrix::Eulerf att_euler = matrix::Quatf(att->q); 
    
    // Euler angles vector    
    measurements_vec->phi_ = (float) att_euler.phi();
    measurements_vec->theta_ = (float) att_euler.theta();
    measurements_vec->psi_ = (float) att_euler.psi();


    // Translational acceleration body frame
    meas_acceleration(measurements_vec, sen_bias); float alpha_ewma = 0.25;
    measurements_vec->axb =  SignalProcessing<float>::exp_moving_average(
        measurements_vec->axb, measurements_vec->axb_p, alpha_ewma);
    measurements_vec->ayb =  SignalProcessing<float>::exp_moving_average(
        measurements_vec->ayb, measurements_vec->ayb_p, alpha_ewma);
    measurements_vec->azb =  SignalProcessing<float>::exp_moving_average(
        measurements_vec->azb, measurements_vec->azb_p, alpha_ewma);

    // Current time
    uint64_t scale = 1.e3;
    measurements_vec->t = att->timestamp / scale;
    
    // Time step (s)
    float h = (float) (measurements_vec->t - measurements_vec->t_p) / 
        (float) scale;
    
    // Translational velocity body frame
    measurements_vec->u = measurements_vec->up + (h / 2) * (measurements_vec->axb + measurements_vec->axb_p);
    measurements_vec->v = measurements_vec->vp + (h / 2) * (measurements_vec->ayb + measurements_vec->ayb_p);
    measurements_vec->w = measurements_vec->wp + (h / 2) * (measurements_vec->azb + measurements_vec->azb_p);

    float rho_dcblocker = 0.995;
    measurements_vec->u  = rho_dcblocker*measurements_vec->up + 
        (measurements_vec->u - measurements_vec->up);
    measurements_vec->v  = rho_dcblocker*measurements_vec->vp + 
        (measurements_vec->v - measurements_vec->vp);
    measurements_vec->w  = rho_dcblocker*measurements_vec->wp + 
        (measurements_vec->w - measurements_vec->wp);

    // Rotational velocity body frame
    measurements_vec->p = (float) att->rollspeed;
    measurements_vec->q = (float) att->pitchspeed;
    measurements_vec->r = (float) att->yawspeed;

    // Record previous measurement data to structure mesuremetns_vec
    measurements_vec->up = measurements_vec->u;
    measurements_vec->vp = measurements_vec->v;
    measurements_vec->wp = measurements_vec->w;

    measurements_vec->pp = measurements_vec->p;
    measurements_vec->qp = measurements_vec->q;
    measurements_vec->rp = measurements_vec->r;

    measurements_vec->phi_p = measurements_vec->phi_;
    measurements_vec->theta_p = measurements_vec->theta_;
    measurements_vec->psi_p =  measurements_vec->psi_;

    measurements_vec->axb_p = measurements_vec->axb;
    measurements_vec->ayb_p = measurements_vec->ayb;
    measurements_vec->azb_p = measurements_vec->azb; 

    measurements_vec->t_p = measurements_vec->t;
}

void Measurements::measurements_init(
            struct measurements_vec_s *measurements_vec,
            const struct vehicle_attitude_s *att, 
            const struct sensor_bias_s *sen_bias){
    
    // Previous acceleration in body frame
    meas_acceleration(measurements_vec, sen_bias);
    measurements_vec->axb_p = measurements_vec->axb;
    measurements_vec->ayb_p = measurements_vec->ayb;
    measurements_vec->azb_p = measurements_vec->azb;
    uint64_t scale = 1.e3;
    measurements_vec->t_p = att->timestamp / scale;

}

void Measurements::meas_acceleration(struct measurements_vec_s *measurements_vec, 
     const struct sensor_bias_s *sen_bias){
        
    // Data types
    typedef matrix::Matrix<float, 3, 1> vector3f;
    typedef matrix::Matrix<float, 3, 3> matrix3f;

    // Euler angles
    matrix::Matrix<float, 3, 1> eul_angles;
    eul_angles(0, 0) = measurements_vec->phi_;
    eul_angles(1, 0) = measurements_vec->theta_;
    eul_angles(2, 0) = measurements_vec->psi_;

    // Rotation matrix body frame to inertial
    Rotations<float, vector3f, matrix3f> rot;
    matrix3f rot_fb_fe = rot.euler_rot(eul_angles);
    matrix3f rot_fe_fb = rot_fb_fe.transpose();

    // Translational acceleration due to normal forces in earth frame
    vector3f an_fe; an_fe(2, 0) = CONSTANTS_ONE_G;

    // Translational acceleration due to normal forces in earth frame
    vector3f an_fb = rot_fe_fb * an_fe;

    // Measured translational acceleration in body frame
    vector3f am_fb; am_fb(0, 0) = (float) sen_bias->accel_x;
    am_fb(1, 0) = (float) sen_bias->accel_y;
    am_fb(2, 0) = (float) sen_bias->accel_z;

    // Acceleration in body frame
    vector3f a_fb = am_fb + an_fb;

    measurements_vec->axb = a_fb(0, 0);
    measurements_vec->ayb = a_fb(1, 0);
    measurements_vec->azb = a_fb(2, 0);

}

#endif