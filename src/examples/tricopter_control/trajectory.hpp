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
        static matrix::Matrix<float, 6, 1> trajectory_q(struct setpoint_vec_s 
            setpoint_vec, struct state_vec_s state_vec);

        static matrix::Matrix<float, 6, 1> trajectory_qdot(struct setpoint_vec_s
            setpoint_vec, struct state_vec_s state_vec);

    private:

};

Trajectory::Trajectory(){}

matrix::Matrix<float, 6, 1> Trajectory::trajectory_q(struct setpoint_vec_s 
    setpoint_vec, struct state_vec_s state_vec){
    
    matrix::Matrix<float, 6, 1> y;
    return y;
}

matrix::Matrix<float, 6, 1> Trajectory::trajectory_qdot(struct setpoint_vec_s 
    setpoint_vec, struct state_vec_s state_vec){
    
    matrix::Matrix<float, 6, 1> y;
    return y;
}


#endif