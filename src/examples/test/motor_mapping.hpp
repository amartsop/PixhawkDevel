#ifndef MOTOR_MAPPING_H
#define MOTOR_MAPPING_H

#include <math.h>
#include "signal_processing.hpp"

template <class M>
class MotorMapping{
    public:

        // Constuctor
        MotorMapping();

        // Main methods
        static M rads2pwm(M x, M current_voltage, M kv);
        
    private:
        static M rads2rpm(M x);
        static M rpm2rads(M x);
        static M rads2volt(M x, M kv);
};

template <class M>
MotorMapping<M>::MotorMapping(){}

template <class M>
M MotorMapping<M>::rads2rpm(M x){
    return x * 30 / (M) M_PI;
}

template <class M>
M MotorMapping<M>::rpm2rads(M x){
    return x * (M) M_PI / 30;
}

template <class M>
M MotorMapping<M>::rads2volt(M x, M kv){
    return rads2rpm(x) / kv;
}

template <class M>
M MotorMapping<M>::rads2pwm(M x, M current_voltage, M kv){
    
    // Calculates the required voltage
    M volts_d = rads2volt(x, kv);

    // Calculates the real voltage that can be provided
    M volts_r = SignalProcessing<M>::clipping(volts_d, 0, current_voltage);

    // Maps the real voltage to [-1, 1] range
    return SignalProcessing<M>::map(volts_r, 0, current_voltage, -1, 1);
}






#endif