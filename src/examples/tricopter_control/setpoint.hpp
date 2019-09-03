#ifndef SETPOINT_H
#define SETPOINT_H

#include "signal_processing.hpp"
#include <uORB/topics/input_rc.h>

// Setpoint for testbed platform - Connect to the testbed channel 

class Setpoint{

    public:
        // Constuctor
        Setpoint();

        // Main methods
        static void set_reference(struct setpoint_vec_s *setpoint_vec, 
            struct input_rc_s *input_rc_signal);

        static float reference_mapping(struct input_rc_s *input_rc_signal,
            float lower_value, float upper_value, int channel);

    private:

        // Channels upper and lower limit
        constexpr static float channel_upper_limit = 2006;
        constexpr static float channel_lower_limit = 982;

        // Translational velocity upper and lower limit (m/s)
        constexpr static float vbar_upper_limit = 0.5; 
        constexpr static float vbar_lower_limit = -0.5;
        
        // Channel for translational velocity command
        constexpr static int8_t channel_vbar = 0;

        // Roll angle upper and lower limit (rad)
        constexpr static float muv_upper_limit = 0.3;
        constexpr static float muv_lower_limit = -0.3;

        // Channel for roll command
        constexpr static int8_t channel_muv = 1;

        // Pitch angle upper and lower limit (rad)
        constexpr static float gammav_upper_limit = 0.3;
        constexpr static float gammav_lower_limit = -0.3;

        // Channel for pitch command
        constexpr static int8_t channel_gammav = 2;

        // Yaw rate upper and lower limit (rad/sec)
        constexpr static float rv_upper_limit = 0.3;
        constexpr static float rv_lower_limit = -0.3;

        // Channel for roll rate command
        constexpr static int8_t channel_rv = 3;

};

Setpoint::Setpoint(){}


void Setpoint::set_reference(struct setpoint_vec_s *setpoint_vec, 
    struct input_rc_s *input_rc_signal){
    
    // // Set reference for translation velocity
    // setpoint_vec->w = reference_mapping(input_rc_signal, w_lower_limit,
    // w_upper_limit, channel_w);

    // // Set reference for rotational velocity
    // setpoint_vec->theta = reference_mapping(input_rc_signal, theta_lower_limit, 
    //     theta_upper_limit, channel_theta);

    setpoint_vec->vbar = 0.3;
    setpoint_vec->muv = 0;
    setpoint_vec->gammav = 0;
    setpoint_vec->rv = 0;
}

/*
    Maps the values of the radio to to the range [lower_value, upper_value] 
    The channel number is defined by the argument "channel"
 */

float Setpoint::reference_mapping(struct input_rc_s *input_rc_signal,
        float lower_value, float upper_value, int channel){

    return SignalProcessing<float>::map(input_rc_signal->values[channel], 
        channel_lower_limit, channel_upper_limit, lower_value, upper_value);
}


#endif