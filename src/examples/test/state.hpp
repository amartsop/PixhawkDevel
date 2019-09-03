#ifndef STATE_H
#define STATE_H


#include <math.h>
#include <poll.h>
#include <matrix/math.hpp>

#include <uORB/topics/battery_status.h>
#include <uORB/topics/actuator_controls.h>
#include <uORB/topics/vehicle_attitude.h>
#include <uORB/topics/vehicle_attitude_setpoint.h>


class State{


    public:
        // Constructor
        State();

        // Main methods
        static void state_est(struct state_vec_s *state_vec, 
            struct measurements_vec_s *measurements_vec,
            struct estimator_status_s *estimator_stat); 
};


State::State(){}

void State::state_est(struct state_vec_s *state_vec, 
    struct measurements_vec_s *measurements_vec,
    struct estimator_status_s *estimator_stat){


    // Data types definition vector3f
    typedef matrix::Matrix<float, 3, 1> vector3f;
    
    // Euler angles
    vector3f euler_angles;   
    euler_angles(0, 0) = measurements_vec->phi_;
    euler_angles(1, 0) = measurements_vec->theta_;
    euler_angles(2, 0) = measurements_vec->psi_;


    // Translational velocity in body frame
    vector3f u_fb;
    u_fb(0, 0) = measurements_vec->u;
    u_fb(1, 0) = measurements_vec->v;
    u_fb(2, 0) = measurements_vec->w;

    // Rotational velocity in body frame
    vector3f w_fb;
    w_fb(0, 0) = measurements_vec->p;
    w_fb(1, 0) = measurements_vec->q;
    w_fb(2, 0) = measurements_vec->r;

    // Data types definition matrix3f
    typedef matrix::Matrix<float, 3, 3> matrix3f;

    // Rotattion object
    Rotations<float, vector3f, matrix3f> rot;
    matrix3f rot_fb_fe = rot.euler_rot(euler_angles); 
    matrix3f rot_ang = rot.rotation_angular(euler_angles);

    // Translational velocity inertial frame
    vector3f u_fe = rot_fb_fe*u_fb; 

    // Rotational velocity inertial frame
    vector3f w_fe = rot_ang*w_fb;

    // State components		
    state_vec->z = (float) 0; 			  // can't be measured or estimated
    state_vec->theta = euler_angles(1, 0);// pitch angle(rad)
    state_vec->w = u_fe(2, 0); 		// velocity in z inertial axis (m/sec)
    state_vec->q = w_fe(1, 0);  //	rotational velocity in y inertial 
                                    //	axis( rad/sec)
}

#endif