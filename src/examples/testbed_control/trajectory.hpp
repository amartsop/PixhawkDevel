#ifndef TRAJECTORY_H
#define TRAJECTORY_H

#include <math.h>
#include <matrix/math.hpp>

#include "signal_processing.hpp"

class Trajectory{
    
    public:
        // Constructor
        Trajectory();

        // Main methods
        static matrix::Matrix<float, 2, 1> trajectory_p(struct setpoint_vec_s 
            setpoint_vec, struct state_vec_s state_vec);

        static matrix::Matrix<float, 2, 1> trajectory_pdot(struct setpoint_vec_s
            setpoint_vec, struct state_vec_s state_vec);

    private:
    
        // thetadot gain
        constexpr static float ktheta = 1;
        
        // theta_dot limit
        constexpr static float theta_dot_upper_lim = 0.2;
        constexpr static float theta_dot_lower_lim = -0.2;
};

Trajectory::Trajectory(){}

matrix::Matrix<float, 2, 1> Trajectory::trajectory_p(struct setpoint_vec_s 
    setpoint_vec, struct state_vec_s state_vec){

    matrix::Matrix<float, 2, 1> y;
    y(0, 0) = setpoint_vec.w;
    y(1, 0) = -ktheta*(state_vec.theta - setpoint_vec.theta);  
    y(1, 0) = SignalProcessing<float>::clipping(y(1, 0), theta_dot_lower_lim,
        theta_dot_upper_lim);

    return y;
}

matrix::Matrix<float, 2, 1> Trajectory::trajectory_pdot(struct setpoint_vec_s 
    setpoint_vec, struct state_vec_s state_vec){
    matrix::Matrix<float, 2, 1> y;
    y(0, 0) = 0; y(1, 0) = 0;
    return y;
}


#endif