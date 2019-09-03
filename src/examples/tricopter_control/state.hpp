#ifndef STATE_H
#define STATE_H


#include <math.h>
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
            struct estimator_status_s *estimator_stat, float *psi_previous); 

    private:

        static void unwrapping(float *x, float y);
};


State::State(){}

void State::state_est(struct state_vec_s *state_vec, 
    struct measurements_vec_s *measurements_vec,
    struct estimator_status_s *estimator_stat, float *psi_previous){


    // Euler angles
    state_vec->phi_ = measurements_vec->phi_;
    state_vec->theta_ = measurements_vec->theta_;

    // Psi measurement unwrapped
    float psi_now = measurements_vec->psi_;
    State::unwrapping(&psi_now, *psi_previous);
    state_vec-> psi_ = psi_now;
    *psi_previous = psi_now;

    // Translational velocity in body frame
    state_vec->u = measurements_vec->u;
    state_vec->v = measurements_vec->v;
    state_vec->w = measurements_vec->w;

    // Rotational velocity in body frame
    state_vec->p = measurements_vec->p;
    state_vec->q = measurements_vec->q;
    state_vec->r = measurements_vec->r;

}

void State::unwrapping(float *x, float y){
    float tol = 10e-4;

    while ((*x - y + (float) M_PI) <= tol){
        *x = *x + 2 * (float) M_PI;
    }

    while((*x - y - (float) M_PI) >= tol) {
        *x = *x - 2 * (float) M_PI;
    }
}


#endif