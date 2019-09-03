#ifndef FAILSAFE_H
#define FAILSAFE_H


class Failsafe{
    public:
    // Constructor
    Failsafe();

    //Main methods
    static void failsafe_check(bool *close_actuators, 
        struct input_rc_s *input_rc_signal);

    private:
        constexpr static int8_t left_switch = 4;
        constexpr static int8_t right_switch = 5;

        constexpr static float switch_on = 982; 
        constexpr static float switch_off = 2006; 
        constexpr static float tol = 1e-3;
};  

Failsafe::Failsafe(){}

void Failsafe::failsafe_check(bool *close_actuators, 
    struct input_rc_s *input_rc_signal){

        
    if ( abs(input_rc_signal->values[left_switch] - switch_on) <= tol && 
        abs(input_rc_signal->values[right_switch] - switch_on) <= tol)
    { 
        *close_actuators = false;
    }
    else
    {
        *close_actuators = true;
    }

}

#endif