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

    private:

        static float reference_mapping(struct input_rc_s *input_rc_signal,
        float lower_value, float upper_value, int channel);

        // Channels upper and lower limit
        constexpr static float channel_upper_limit = 2006;
        constexpr static float channel_lower_limit = 982;

        // Translational velocity upper and lower limit (m/s)
        constexpr static float w_upper_limit = -0.2; 
        constexpr static float w_lower_limit = 0.2;
        
        // Channel for translational velocity command
        constexpr static int8_t channel_w = 2;

        // Rotational velocity upper and lower limit (rad/s)
        constexpr static float theta_upper_limit = 0.7;
        constexpr static float theta_lower_limit = -0.7;

        // Channel for pitch command
        constexpr static int8_t channel_theta = 0;
};

Setpoint::Setpoint(){}


void Setpoint::set_reference(struct setpoint_vec_s *setpoint_vec, 
    struct input_rc_s *input_rc_signal){
    
    // Set reference for translation velocity
    setpoint_vec->w = reference_mapping(input_rc_signal, w_lower_limit,
    w_upper_limit, channel_w);

    // Set reference for rotational velocity
    setpoint_vec->theta = reference_mapping(input_rc_signal, theta_lower_limit, 
        theta_upper_limit, channel_theta);

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