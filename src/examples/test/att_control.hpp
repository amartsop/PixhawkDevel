#ifndef ATT_CONTROL_H
#define ATT_CONTROL_H

#include "motor_mapping.hpp"
#include "signal_processing.hpp"
#include "rotations.hpp"
#include "print_debug.hpp"
#include "trajectory.hpp"


#include <math.h>
#include <matrix/math.hpp>

#include <uORB/topics/battery_status.h>
#include <uORB/topics/actuator_controls.h>

class AttControl{
    public:
        // Contructor
        AttControl();

		// Main methods
		static void control_law(struct setpoint_vec_s *setpoint_vec, 
			struct state_vec_s *state_vec, 
			struct actuator_controls_s *actuators,
			struct battery_status_s *bat_status, bool close_actuators);

		private:

		// mass matrix 
		static matrix::Matrix<float, 2, 2> h_mat(struct state_vec_s state_vec);

		// c vector
		static matrix::Matrix<float, 2, 1> c_vec(struct state_vec_s state_vec);		

		// g matrix
		static matrix::Matrix<float, 2, 2> g_mat(struct state_vec_s state_vec);

 
		// system properties
		constexpr static float mass = 2.069; // kg
      	constexpr static float lc = 44.848e-3; // m
        constexpr static float inertia = 45294.148e-6; // kgm^2
        constexpr static float g = 9.81; // m/s^2
        constexpr static float atl = 3.1262e-05; // Ns^2/rad^2
        constexpr static float atr = 3.1262e-05; // Ns^2/rad^2
        constexpr static float lxl = 0.314725; // m
        constexpr static float lxr = 0.314725; // m	
		constexpr static float kv = 1200; // rpm/V

		// controller gains
		constexpr static float k_zdot = 100;
		constexpr static float k_thetadot = 100;

		// control limits
		constexpr static float w_t_l_lim = 0;
		constexpr static float w_t_u_lim = 2607037;

		// sliding gain
		constexpr static float zeta_z = 5;
		constexpr static float zeta_theta = 100;

		// saturation limits (ONLY POSSITIVES (not zero or negative))
		constexpr static float phi_z = 1;
		constexpr static float phi_theta = 1;

};


AttControl::AttControl(){}

void AttControl::control_law(struct setpoint_vec_s *setpoint_vec, 
	struct state_vec_s *state_vec,  struct actuator_controls_s *actuators,
	struct battery_status_s *bat_status, bool close_actuators){

	if (close_actuators){
		for (int8_t i = 0; i < actuators->NUM_ACTUATOR_CONTROLS; i++) {
			actuators->control[i] = -1.0f;
		}
	}
	else{

 		// state
		matrix::Matrix<float, 2, 1> p; p(0, 0) = state_vec->w;
		p(1, 0) = state_vec->q;

		// desired state
		matrix::Matrix<float, 2, 1> pd = Trajectory::trajectory_p(*setpoint_vec,
			*state_vec); 
			
		// desired dot state
		matrix::Matrix<float, 2, 1> pd_dot = Trajectory::trajectory_pdot(*setpoint_vec,
			*state_vec);

 		// matrix h
		matrix::Matrix<float, 2, 2>  h_ = h_mat(*state_vec);

		// matrix h inverse
		matrix::SquareMatrix<float, 2> h_sqr = (matrix::SquareMatrix<float, 2>) h_;
		matrix::SquareMatrix<float, 2> h_inv = matrix::inv(h_sqr);

		// vector c
		matrix::Matrix<float, 2, 1> c_ = c_vec(*state_vec);

		// matrix g
		matrix::Matrix<float, 2, 2> g_ = g_mat(*state_vec);

		// vector a
		matrix::Matrix<float, 2, 1> a_ = -h_inv*c_;

		// matrix B
		matrix::Matrix<float, 2, 2> b_ = h_inv*g_;
		matrix::SquareMatrix<float, 2> b_sqr = (matrix::SquareMatrix<float, 2>) b_;
		matrix::SquareMatrix<float, 2> b_inv = matrix::inv(b_sqr);

		// gain matrix
		matrix::Matrix<float, 2, 2> k_; k_(0, 0) = k_zdot; k_(0, 1) = 0;
		k_(1, 0) = 0; k_(1, 1) = k_thetadot;

		//v control
		matrix::Matrix<float, 2, 1> v_ = pd_dot - k_*(p - pd);

		// sigma 
		matrix::Matrix<float, 2, 1> sigma_ = p - pd;

		// zeta gain
		matrix::Matrix<float, 2, 2> z_; z_(0, 0) = zeta_z; z_(0, 1) = 0;
		z_(1, 0) = 0; z_(1, 1) = zeta_theta;

		// sliding part
		matrix::Matrix<float, 2, 1> u_sld; 
		u_sld(0, 0) = SignalProcessing<float>::saturation(sigma_(0, 0), 
			phi_z, 10e-4);
		u_sld(1, 0) = SignalProcessing<float>::saturation(sigma_(1, 0),
			phi_theta, 10e-4);
 

		//u control
		matrix::Matrix<float, 2, 1> u_ = b_inv*(v_ - a_ - u_sld);

		// Clipping
		u_(0, 0) = SignalProcessing<float>::clipping(u_(0, 0), w_t_l_lim, 
			w_t_u_lim);
		u_(1, 0) = SignalProcessing<float>::clipping(u_(1, 0), w_t_l_lim, 
			w_t_u_lim);


		// u_real control (rad/sec)
		matrix::Matrix<float, 2, 1> u_r;
		u_r(0, 0) = sqrtf(u_(0, 0));
		u_r(1, 0) = sqrtf(u_(1, 0));

		// Actuators values pwm
		actuators->control[0] = MotorMapping<float>::rads2pwm(u_r(0, 0),
			bat_status->voltage_v, kv);

		actuators->control[1] = MotorMapping<float>::rads2pwm(u_r(1, 0),
			bat_status->voltage_v, kv);

	}
}

matrix::Matrix<float, 2, 2> AttControl::h_mat(struct state_vec_s state_vec){
	matrix::Matrix<float, 2, 2> y;
	y(0, 0) = mass; y(0, 1) = mass*lc*sinf(state_vec.theta);
	y(1, 0) = 0; y(1, 1) = inertia + mass*powf(lc, 2);
	return y;		
}

matrix::Matrix<float, 2, 1> AttControl::c_vec(struct state_vec_s state_vec){
	matrix::Matrix<float, 2, 1> y;
	y(0, 0) = mass*lc*powf(state_vec.q, 2)*cosf(state_vec.theta) - mass*g;
	y(1, 0) = -mass*g*lc*sinf(state_vec.theta);
	return y;
}

matrix::Matrix<float, 2, 2> AttControl::g_mat(struct state_vec_s state_vec){
	matrix::Matrix<float, 2, 2> y;
	y(0, 0) = -atl*cosf(state_vec.theta); y(0, 1) = -atr*cosf(state_vec.theta);
	y(1, 0) = lxl*atl; y(1, 1) = -lxr*atr;
	return y;
}




#endif