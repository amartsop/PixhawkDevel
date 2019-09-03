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


/* State, Measurement and control dimensions */
#define Ns 4
#define N 2
#define M 2
#define P 3

class AttControl{
    public:

        // Contructor
        AttControl();

		// Main methods
		static void control_law(struct setpoint_vec_s *setpoint_vec, 
			struct state_vec_s *state_vec, 
			struct actuator_controls_s *actuators,
			struct control_vec_s *control_vec,
			struct battery_status_s *bat_status, bool close_actuators);

		constexpr static float lc = 44.848e-3; // m
		
				// mass matrix 
		static matrix::Matrix<float, N, N> h_mat(matrix::Matrix<float, Ns, 1> x_);

		// c vector
		static matrix::Matrix<float, N, 1> c_vec(matrix::Matrix<float, Ns, 1> x_);		

		// g matrix
		static matrix::Matrix<float, N, M> g_mat(matrix::Matrix<float, Ns, 1> x_);

		/* State vector indices */
		constexpr static int8_t w_i = 0;
		constexpr static int8_t q_i = 1;
		constexpr static int8_t z_i = 2;
		constexpr static int8_t theta_i = 3;

		/* Control vector indices */
		constexpr static int8_t wl_i = 0;
		constexpr static int8_t wr_i = 1;
		
		private:

		/* System properties */
		constexpr static float mass = 3.3; // kg
        constexpr static float inertia = 45294.148e-6; // kgm^2
        constexpr static float g = 9.80665; // m/s^2
        constexpr static float atl = 2.8915e-05; // Ns^2/rad^2
        constexpr static float atr = 2.8915e-05; // Ns^2/rad^2
        constexpr static float lxl = 0.314725; // m
        constexpr static float lxr = 0.314725; // m	
		constexpr static float kv = 1200; // rpm/V

		// controller gains
		constexpr static float k_zdot = 10;
		constexpr static float k_thetadot = 30;

		// control limits
		constexpr static float w_t_l_lim = 0;
		constexpr static float w_t_u_lim = 2505889;

		// sliding gain
		constexpr static float zeta_z = 5;
		constexpr static float zeta_theta = 40;   

		// saturation limits (ONLY POSSITIVES (not zero or negative))
		constexpr static float phi_z = 0.15;
		constexpr static float phi_theta = 0.15;
};


AttControl::AttControl(){}

void AttControl::control_law(struct setpoint_vec_s *setpoint_vec, 
	struct state_vec_s *state_vec,  struct actuator_controls_s *actuators,
	struct control_vec_s *control_vec,
	struct battery_status_s *bat_status, 
	bool close_actuators){

	if (close_actuators){
		for (int8_t i = 0; i < actuators->NUM_ACTUATOR_CONTROLS; i++) {
			actuators->control[i] = -1.0f;
		}
	}
	else{

		// State
		matrix::Matrix<float, Ns, 1> x_;
        x_(w_i, 0) = state_vec->w; x_(q_i, 0) = state_vec->q;
        x_(z_i, 0) = state_vec->z; x_(theta_i, 0) = state_vec->theta;
 		
		// substate
		matrix::Matrix<float, N, 1> p; p(0, 0) = x_(w_i, 0); 
		p(1, 0) = x_(q_i, 0); 
		
		// desired substate
		matrix::Matrix<float, N, 1> pd = Trajectory::trajectory_p(*setpoint_vec,
			*state_vec); 
			
		// desired dot state
		matrix::Matrix<float, N, 1> pd_dot = Trajectory::trajectory_pdot(*setpoint_vec,
			*state_vec);

		 // matrix h
		matrix::Matrix<float, N, N>  h_ = h_mat(x_);

		// matrix h inverse
		matrix::SquareMatrix<float, N> h_sqr = (matrix::SquareMatrix<float, N>) h_;
		matrix::SquareMatrix<float, N> h_inv = matrix::inv(h_sqr);

		// vector c
		matrix::Matrix<float, N, 1> c_ = c_vec(x_);

		// matrix g
		matrix::Matrix<float, N, M> g_ = g_mat(x_);

		// vector a
		matrix::Matrix<float, N, 1> a_ = -h_inv*c_;

		// matrix B
		matrix::Matrix<float, N, M> b_ = h_inv*g_;
		matrix::SquareMatrix<float, M> b_sqr = (matrix::SquareMatrix<float, M>) b_;
		matrix::SquareMatrix<float, M> b_inv = matrix::inv(b_sqr);

		// gain matrix
		matrix::Matrix<float, N, N> k_; k_(0, 0) = k_zdot; k_(0, 1) = 0;
		k_(1, 0) = 0; k_(1, 1) = k_thetadot;
		
		//v control
		matrix::Matrix<float, N, 1> v_ = pd_dot - k_*(p - pd);
		
		// sigma 
		matrix::Matrix<float, N, 1> sigma_ = p - pd;

		// zeta gain
		matrix::Matrix<float, N, N> z_; z_(0, 0) = zeta_z; z_(0, 1) = 0;
		z_(1, 0) = 0; z_(1, 1) = zeta_theta;

		// sliding part
		matrix::Matrix<float, N, 1> u_sld; 
		u_sld(0, 0) = SignalProcessing<float>::saturation(sigma_(0, 0), phi_z);
		u_sld(1, 0) = SignalProcessing<float>::saturation(sigma_(1, 0), phi_theta);
 
		//u control
		matrix::Matrix<float, M, 1> u_ = b_inv*(v_ - a_ - z_*u_sld);

		// Clipping
		u_(wl_i, 0) = SignalProcessing<float>::clipping(u_(wl_i, 0), w_t_l_lim, 
			w_t_u_lim);
		u_(wr_i, 0) = SignalProcessing<float>::clipping(u_(wr_i, 0), w_t_l_lim, 
			w_t_u_lim);

		// Sent control to control struct
		control_vec->wl_t = u_(wl_i, 0); control_vec->wr_t = u_(wr_i, 0);

		// u_real control (rad/sec)
		matrix::Matrix<float, N, 1> u_r;
		u_r(wl_i, 0) = sqrtf(u_(wl_i, 0));
		u_r(wr_i, 0) = sqrtf(u_(wr_i, 0));

		// Actuators values pwm
		actuators->control[wl_i] = MotorMapping<float>::rads2pwm(u_r(wl_i, 0),
			bat_status->voltage_v, kv);

		actuators->control[wr_i] = MotorMapping<float>::rads2pwm(u_r(wr_i, 0),
			bat_status->voltage_v, kv);
	}
}

matrix::Matrix<float, N, N> AttControl::h_mat(matrix::Matrix<float, Ns, 1> x_){
	matrix::Matrix<float, N, N> y;
	y(0, 0) = mass; y(0, 1) = mass*lc*sinf(x_(theta_i, 0));
	y(1, 0) = 0; y(1, 1) = inertia + mass*powf(lc, 2);
	return y;		
}

matrix::Matrix<float, N, 1> AttControl::c_vec(matrix::Matrix<float, Ns, 1> x_){
	matrix::Matrix<float, N, 1> y; 
	y(0, 0) = mass*lc*powf(x_(q_i, 0), 2)*cosf(x_(theta_i, 0)) - mass*g;
	y(1, 0) = -mass*g*lc*sinf(x_(theta_i, 0));
	return y;
}

matrix::Matrix<float, N, M> AttControl::g_mat(matrix::Matrix<float, Ns, 1> x_){
	matrix::Matrix<float, N, M> y;
	y(0, 0) = -atl*cosf(x_(theta_i, 0)); y(0, 1) = -atr*cosf(x_(theta_i, 0));
	y(1, 0) = lxl*atl; y(1, 1) = -lxr*atr;
	return y;
}




#endif