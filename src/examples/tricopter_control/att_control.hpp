#ifndef ATT_CONTROL_H
#define ATT_CONTROL_H

#include "motor_mapping.hpp"
#include "signal_processing.hpp"
#include "rotations.hpp"
#include "print_debug.hpp"
#include "trajectory.hpp"


#include <math.h>
#include <matrix/math.hpp>
#include <px4_tasks.h>

#include <uORB/topics/battery_status.h>
#include <uORB/topics/actuator_controls.h>

/* State and  control dimensions */
#define N 6
#define M 5
#define N2 3

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
		
		/* System properties */
		constexpr static float mass = 4.165; // Vehicle mass (kg)
		constexpr static float g = 9.81; // Acceleration of gravity (m / s^2)
		constexpr static float Ixx = 291365.193e-6; // Moment of inertia (xx) (kg x m^2)
		constexpr static float Iyy = 269325.097e-6; // Moment of inertia (yy) (kg x m^2)
		constexpr static float Izz = 553060.15e-6; // Moment of inertia (zz) (kg x m^2)
 		constexpr static float Ixz = 0; // Moment of inertia (xz) (kg x m^2)
		constexpr static float Ixy = 0; // Moment of inertia (xy) (kg x m^2)
		constexpr static float Iyz = 0; // Moment of inertia (yz) (kg x m^2)
		constexpr static float l1 = 0.4015; // Characteristic length l1 (m)
		constexpr static float l2 = 0.5845; // Characteristic length l2 (m)
		constexpr static float l3 = 0.319; // Characteristic length l3 (m)
		constexpr static float l4 = 0; // Characteristic length l4 (m)
		constexpr static float l5 = 0; // Characteristic length l5 (m)
		constexpr static float sl = -1; // Front-left motor direction of torque.
		constexpr static float sr = -1; // Front-right motor direction of torque.
		constexpr static float sb = 1; // Rear motor direction of torque.
 
        constexpr static float atl = 3.1262e-05; // Ns^2/rad^2
        constexpr static float atr = 3.1262e-05; // Ns^2/rad^2
		constexpr static float atb = 4.7983e-05; // Ns^2/rad^2

        constexpr static float bnl = 1.0529e-06; // Nms^2/rad^2
        constexpr static float bnr = 1.0529e-06; // Nms^2/rad^2
		constexpr static float bnb = 1.7779e-06; // Nms^2/rad^2

		/* Equilibrium point */
		constexpr static float wl_eq = 622.50345; // (rad/sec)
		constexpr static float wr_eq = 622.50343; // (rad/sec)
		constexpr static float wb_eq = 588.84799; // (rad/sec)
		constexpr static float tl_eq = 0.02578; // (rad)
		constexpr static float tr_eq = -0.02579; // (rad)

		/* Control vector indices */
		constexpr static int8_t wl_i = 0;
		constexpr static int8_t wr_i = 1;
		constexpr static int8_t wb_i = 2;
		constexpr static int8_t tl_i = 3;
		constexpr static int8_t tr_i = 4; 

		/* Controller gain */
		constexpr static float k_u = 10; constexpr static float k_v = 10; 
		constexpr static float k_w = 10; constexpr static float k_p = 10; 
		constexpr static float k_q = 10; constexpr static float k_r = 10;

		/* Sliding gain */
		constexpr static float z_u = 30; constexpr static float z_v = 30; 
		constexpr static float z_w = 30; constexpr static float z_p = 30; 
		constexpr static float z_q = 30; constexpr static float z_r = 30;

		/* Saturation limits */
		constexpr static float phi_u = 0.2; constexpr static float phi_v = 0.2; 
		constexpr static float phi_w = 0.2; constexpr static float phi_p = 0.2; 
		constexpr static float phi_q = 0.2; constexpr static float phi_r = 0.2;		
		
		/* Virtual control limits */
		constexpr static float wi_t_l_lim = 0;
		constexpr static float wl_t_u_lim = 2505889;
		constexpr static float wr_t_u_lim = 2505889;
		constexpr static float wb_t_u_lim = 1471369;
		constexpr static float tl_lim = 0.349;
		constexpr static float tr_lim = 0.349;

		/* Motors kv */
		constexpr static float kv_f = 1200; // (rpm/v)		
		constexpr static float kv_r = 920; // (rpm/v)

		// Main private methods

		/* Calculates the affine_a function */
		static matrix::Matrix<float, N, 1> affine_a(struct state_vec_s state_vec);
		
		/* Calculates the affine_b function */
		static matrix::Matrix<float, N, M> affine_b(struct state_vec_s state_vec);

		/* Calculates the mass matrix */
		static matrix::Matrix<float, N2, N2> mass_matrix(void);

		/* Calculates the mass matrix inverse */
		static matrix::Matrix<float, N2, N2> mass_matrix_inv(void);

		/* Calculates the inertia matrix */
		static matrix::Matrix<float, N2, N2> inertia_matrix(void);

		/* Calculates the inertia matrix inverse */
		static matrix::Matrix<float, N2, N2> inertia_matrix_inv(void);

		/* Calculates the state dependent vectorfield of the 
    		external forces */
		static matrix::Matrix<float, N2, 1> force_x(struct state_vec_s
			state_vec);

		/* Calculates the control matrix of external forces */
		static matrix::Matrix<float, N2, M> force_u(struct state_vec_s
			state_vec);

		/* Calculates the state dependent vectorfield of the external moments */
		static matrix::Matrix<float, N2, 1> moment_x(struct state_vec_s
			state_vec);

		/* Calculates the control matrix of external moments */
		static matrix::Matrix<float, N2, M> moment_u(struct state_vec_s
			state_vec);
		
		/********************* Force Functions *********************/

		/* Calculates the state dependent vectorfield of the weight forces */
		static matrix::Matrix<float, N2, 1> weight_body(struct state_vec_s
			state_vec);

		/* Calculates the state dependent vectorfield of the thrust forces */
		static matrix::Matrix<float, N2, 1> thrustforce_x(void);

		/* Calculates the control matrix of the thrust forces */
		static matrix::Matrix<float, N2, M>thrustforce_u(void);
		
		/********************* Moment Functions *********************/

		/* Calculates the state dependent vectorfield of the thrust moments */
		static matrix::Matrix<float, N2, 1> thrustmoment_x(void);

		/* Calculates the control matrix of the thrust moments */
		static matrix::Matrix<float, N2, M>thrustmoment_u(void);

		/************ Thrust Force and Moment for Linearization ************/

		/* Calculates the equilibrum point of hover */
		static matrix::Matrix<float, M, 1> trim_point(void);
		
		/* Calculates the thrust forces */
		static matrix::Matrix<float, N2, 1> thrust_forces(matrix::Matrix<float, 
			M, 1> u);

		/* Calculates the thrust moments */
		static matrix::Matrix<float, N2, 1> thrust_moments(matrix::Matrix<float,
			M, 1> u);

		/* Calculates the jacobian of thrust forces with respect to the 
			control vector */
		static matrix::Matrix<float, N2, M> thrustforce_jac(matrix::Matrix<float,
			M, 1> u);
		
		/*  Calculates the jacobian of thrust moments with respect to 
    		the control vector*/
		static matrix::Matrix<float, N2, M> thrustmoment_jac(matrix::Matrix<float,
			M, 1> u);
};


AttControl::AttControl(){}

void AttControl::control_law(struct setpoint_vec_s *setpoint_vec, 
	struct state_vec_s *state_vec,  struct actuator_controls_s *actuators,
	struct battery_status_s *bat_status, bool close_actuators){

	// Only for debugging -- CHANGE
	close_actuators = false;

	if (close_actuators){
		for (int8_t i = 0; i < actuators->NUM_ACTUATOR_CONTROLS; i++) {
			actuators->control[i] = -1.0f;
		}
	}
	else{

		// State (translational and rotational velocities)
		matrix::Matrix<float,N, 1> q_; q_(0, 0) = state_vec->u;
		q_(1, 0) = state_vec->v; q_(2, 0) = state_vec->w;
		q_(3, 0) = state_vec->p; q_(4, 0) = state_vec->q; 
		q_(5, 0) = state_vec->r; 

		// Desired states
		matrix::Matrix<float, N, 1> qd = Trajectory::trajectory_q(*setpoint_vec,
			*state_vec);
		
		// Desired states dot
		matrix::Matrix<float, N, 1> qd_dot = Trajectory::trajectory_qdot(
			*setpoint_vec, *state_vec);

		// Gain matrix
		matrix::Matrix<float, N, N> K; K(0, 0) = k_u; K(1, 1) = k_v;
		K(2, 2) = k_w; K(3, 3) = k_p; K(4, 4) = k_q; K(5, 5) = k_r;

		// Sliding gain
		matrix::Matrix<float, N, N> Z; Z(0, 0) = z_u; Z(1, 1) = z_v;
		Z(2, 2) = z_w; Z(3, 3) = z_p; Z(4, 4) = z_q; Z(5, 5) = z_r;	

		// Feedback linearization (v control)
		matrix::Matrix<float, N, 1> v = qd_dot - K*(q_ - qd);

		// Sigma sliding
		matrix::Matrix<float, N, 1> sigma_ = q_ - qd;

		// Sliding part
		matrix::Matrix<float, N, 1> u_sld; 
		u_sld(0, 0) = SignalProcessing<float>::saturation(sigma_(0, 0), phi_u);
		u_sld(1, 0) = SignalProcessing<float>::saturation(sigma_(1, 0), phi_v);
		u_sld(2, 0) = SignalProcessing<float>::saturation(sigma_(2, 0), phi_w);  
		u_sld(3, 0) = SignalProcessing<float>::saturation(sigma_(3, 0), phi_p);
		u_sld(4, 0) = SignalProcessing<float>::saturation(sigma_(4, 0), phi_q);
		u_sld(5, 0) = SignalProcessing<float>::saturation(sigma_(5, 0), phi_r);

		// Affine_a
		matrix::Matrix<float, N, 1> a_ = affine_a(*state_vec);

		// Control law unmapped to control space
		matrix::Matrix<float, N, 1> u_unm = v - a_ - Z*u_sld;

		// Control law mapped to control space
		matrix::Matrix<float, N, M>  b_ = affine_b(*state_vec);
		matrix::LeastSquaresSolver<float, N, M> qrd = 
			matrix::LeastSquaresSolver<float, N, M>(b_);
		matrix::Matrix<float, M, 1> u_ = qrd.solve(u_unm);

		// Real control (rad/sec & rad)
		u_(wl_i, 0) = SignalProcessing<float>::clipping(u_(wl_i, 0), wi_t_l_lim, 
			wl_t_u_lim); u_(wl_i, 0) = sqrtf(u_(wl_i, 0)); 
		u_(wr_i, 0) = SignalProcessing<float>::clipping(u_(wr_i, 0), wi_t_l_lim, 
			wr_t_u_lim); u_(wr_i, 0) = sqrtf(u_(wr_i, 0));
		u_(wb_i, 0) = SignalProcessing<float>::clipping(u_(wb_i, 0), wi_t_l_lim, 
			wb_t_u_lim); u_(wb_i, 0) = sqrtf(u_(wb_i, 0));	
		u_(tl_i, 0) = SignalProcessing<float>::clipping(u_(tl_i, 0), -tl_lim, 
			tl_lim);
		u_(tr_i, 0) = SignalProcessing<float>::clipping(u_(tr_i, 0), -tr_lim, 
			tr_lim);

		// Actuators values pwm
		actuators->control[0] = MotorMapping<float>::rads2pwm(u_(wl_i, 0),
			bat_status->voltage_v, kv_f);

		actuators->control[1] = MotorMapping<float>::rads2pwm(u_(wr_i, 0),
			bat_status->voltage_v, kv_f);

		actuators->control[2] = MotorMapping<float>::rads2pwm(u_(wb_i, 0),
			bat_status->voltage_v, kv_r);
		
		actuators->control[3] = SignalProcessing<float>::map(u_(tl_i, 0),
			-tl_lim, tl_lim, -1, 1);

		actuators->control[4] = SignalProcessing<float>::map(u_(tr_i, 0),
			-tr_lim, tr_lim, -1, 1);

	}
}

/* Calculates the affine_a function */
matrix::Matrix<float, N, 1> AttControl::affine_a(struct state_vec_s state_vec){
	
	typedef matrix::Matrix<float, N2, 1> vector3f;
	typedef matrix::Matrix<float, N2, N2> matrix3f;
	
	// Mass matrix
	matrix3f m_ = mass_matrix();
	matrix3f m_inv = mass_matrix_inv();

	// Inertia matrix
	matrix3f ib_ = inertia_matrix();
	matrix3f ib_inv = inertia_matrix_inv();

	// External loads x
	vector3f fx = force_x(state_vec);
	vector3f mx = moment_x(state_vec);

	//Translational velocity
	vector3f u_; u_(0, 0) = state_vec.u; u_(1, 0) = state_vec.v;
	u_(2, 0) = state_vec.w;

	// Rotational velocity
	vector3f w_; w_(0, 0) = state_vec.p; w_(1, 0) = state_vec.q; 
	w_(2, 0) = state_vec.r;
	
	// Skew symmetric matrix
	Rotations<float, vector3f, matrix3f> rot;
	matrix3f skew_ = rot.skew_sym(w_);

	// u_dot
	vector3f u_dot = m_inv*fx - m_inv*skew_*m_*u_;
	
	// w_dot
	vector3f w_dot = ib_inv*mx - ib_inv*skew_*ib_*w_;

	//
	matrix::Matrix<float, N, 1> y; y(0, 0) = u_dot(0, 0); y(1, 0) = u_dot(1, 0);
	y(2, 0) = u_dot(2, 0); y(3, 0) = w_dot(0, 0); y(4, 0) = w_dot(1, 0);
	y(5, 0) = w_dot(2, 0); return y;

}

/* Calculates the affine_b function */
matrix::Matrix<float, N, M> AttControl::affine_b(struct state_vec_s state_vec){
	
	// External loads u
	matrix::Matrix<float, N2, M> fu = force_u(state_vec);
	matrix::Matrix<float, N2, M> mu = moment_u(state_vec);

	// Mass matrix
	matrix::Matrix<float, N2, N2> m_inv = mass_matrix_inv();

	// Inertia matrix inverse
	matrix::Matrix<float, N2, N2> ib_inv = inertia_matrix_inv();	
	
	// u_dot
	matrix::Matrix<float, N2, M> u_dot = m_inv*fu;

	// w_dot
	matrix::Matrix<float, N2, M> w_dot = ib_inv*mu;

	matrix::Matrix<float, N, M> y;  y.set(u_dot, 0, 0); y.set(w_dot, 3, 0);
	return y;
}

/* Calculates the mass matrix */
matrix::Matrix<float, N2, N2> AttControl::mass_matrix(void){
	
	matrix::Matrix<float, N2, N2> m_; m_.setIdentity(); return (mass*m_);
}

/* Calculates the mass and inertia matrix inverse */
matrix::Matrix<float, N2, N2> AttControl::mass_matrix_inv(void){

	matrix::Matrix<float, N2, N2> m_inv; m_inv.setIdentity(); 
	return ((1/mass)*m_inv);
}

/* Calculates the inertia matrix */
matrix::Matrix<float, N2, N2> AttControl::inertia_matrix(void){

	matrix::Matrix<float, N2, N2> ib_; ib_(0, 0) = Ixx; ib_(0, 1) = -Ixy;
	ib_(0, 2) = -Ixz; ib_(1, 0) = -Ixy; ib_(1, 1) = Iyy; ib_(1, 2) = -Iyz;
	ib_(2, 0) = -Ixz; ib_(2, 1) = -Iyz; ib_(2, 2) = Izz; return ib_;	
}

/* Calculates the inertia matrix inverse */
matrix::Matrix<float, N2, N2> AttControl::inertia_matrix_inv(void){
	matrix::Matrix<float, N2, N2> ib_inv; ib_inv(0, 0) = powf(Iyz, 2) - Iyy*Izz; 
	ib_inv(0, 1) = Ixz*Iyz - Ixy*Izz; ib_inv(0, 2) = -(Ixy*Iyz - Ixz*Iyy);
	ib_inv(1, 0) = -(Ixz*Iyz + Ixy*Izz); ib_inv(1, 1) = -(powf(Ixz, 2) + Ixx*Izz);
	ib_inv(1, 2) = (Ixy*Ixz - Ixx*Iyz); ib_inv(2, 0) = -(Ixy*Iyz + Ixz*Iyy);
	ib_inv(2, 1) = -(Ixy*Ixz + Ixx*Iyz); ib_inv(2, 2) =  powf(Ixy, 2) - Ixx*Iyy;
	float den_ = Izz*powf(Ixy, 2) - Iyy*powf(Ixz, 2) + Ixx*powf(Iyz, 2) -
		Ixx*Iyy*Izz; return (ib_inv/den_); 
}

/* Calculates the state dependent vectorfield of the external forces */
matrix::Matrix<float, N2, 1> AttControl::force_x(struct state_vec_s state_vec){
	
	return (weight_body(state_vec) + thrustforce_x());
}

/* Calculates the control matrix of external forces */
matrix::Matrix<float, N2, M> AttControl::force_u(struct state_vec_s state_vec){
	
	return thrustforce_u();
}

/* Calculates the state dependent vectorfield of the external moments */
matrix::Matrix<float, N2, 1> AttControl::moment_x(struct state_vec_s state_vec){
	
	return thrustmoment_x();
}

/* Calculates the control matrix of external moments */
matrix::Matrix<float, N2, M> AttControl::moment_u(struct state_vec_s state_vec){
	
	return thrustmoment_u();
}

/********************* Force Functions *********************/

/* Calculates the state dependent vectorfield of the weight forces */
matrix::Matrix<float, N2, 1> AttControl::weight_body(struct state_vec_s state_vec){
	
	// Data types definition
	typedef matrix::Matrix<float, N2, N2> matrix3f;
	typedef matrix::Matrix<float, N2, 1> vector3f;

	// Mass matrix
	matrix3f mass_mat; memset(&mass_mat, 0, sizeof(mass_mat));
	mass_mat(0, 0) = mass; mass_mat(1, 1) = mass; mass_mat(2, 2) = mass;

	// g vector
	vector3f g_vec; memset(&g_vec, 0, sizeof(g_vec));
	g_vec(0, 2) = g;

	// weight in inertial frame
	vector3f w_fe = mass_mat * g_vec;

	// euler angles
	vector3f eul; eul(0, 0) = state_vec.phi_; eul(1, 0) = state_vec.theta_;
	eul(2, 0) = state_vec.psi_;
	
	// Rotation matrix body wrt inertial
	Rotations<float, vector3f, matrix3f> rot;
	matrix3f r_fb_fe = rot.euler_rot(eul);

	// Rotation matrix inertial wrt body
	matrix3f r_fe_fb = r_fb_fe.transpose();

	// weight in body frame
	vector3f w_fb = r_fe_fb * w_fe; return w_fb;
} 

/* Calculates the state dependent vectorfield of the thrust forces */
matrix::Matrix<float, N2, 1> AttControl::thrustforce_x(void){
	matrix::Matrix<float, M, 1> u0 = trim_point();
	matrix::Matrix<float, N2, 1> y;
	y = thrust_forces(u0) - thrustforce_jac(u0)*u0;
	return y;
}

/* Calculates the control matrix of the thrust forces */
matrix::Matrix<float, N2, M> AttControl::thrustforce_u(void){
	matrix::Matrix<float, M, 1> u0 = trim_point();
	return thrustforce_jac(u0);
} 

/********************* Moment Functions *********************/

/* Calculates the state dependent vectorfield of the thrust moments */
matrix::Matrix<float, N2, 1> AttControl::thrustmoment_x(void){
	matrix::Matrix<float, M, 1> u0 = trim_point();
	matrix::Matrix<float, N2, 1> y;
	y = thrust_moments(u0) - thrustmoment_jac(u0)*u0;
	return y;
}

/* Calculates the control matrix of the thrust moments */
matrix::Matrix<float, N2, M> AttControl::thrustmoment_u(void){
	matrix::Matrix<float, M, 1> u0 = trim_point();
	return thrustmoment_jac(u0);
} 


/************ Thrust Force and Moment for Linearization ************/

/* Calculates the equilibrum point of hover */
matrix::Matrix<float, M, 1> AttControl::trim_point(void){
	matrix::Matrix<float, M, 1> u0;u0(wl_i, 0) = powf(wl_eq, 2);
	u0(wr_i, 0) = powf(wr_eq, 2); u0(wb_i, 0) = powf(wb_eq, 2); 
	u0(tl_i, 0) = tl_eq; u0(tr_i, 0) = tr_eq; return u0;
}

/* Calculates the thrust forces */
matrix::Matrix<float, N2, 1> AttControl::thrust_forces(matrix::Matrix<float,
	5, 1> u){
	matrix::Matrix<float, N2, 1> y; 
	y(0, 0) = -atl*sinf(u(tl_i, 0))*u(wl_i, 0) - atr*sinf(u(tr_i, 0))*
		u(wr_i, 0); y(1, 0) = 0;
	y(2, 0) = -atl*cosf(u(tl_i, 0))*u(wl_i, 0) - atr*cosf(u(tr_i, 0))*
		u(wr_i, 0) - atb*u(wb_i, 0);
	return y;
}

/* Calculates the thrust moments */
matrix::Matrix<float, N2, 1> AttControl::thrust_moments(matrix::Matrix<float,
	M, 1> u){
	matrix::Matrix<float, N2, 1> y;
	y(0, 0) = l3*atl*cosf(u(tl_i, 0))*u(wl_i, 0) -
		sl*bnl*sinf(u(tl_i, 0))*u(wl_i, 0) -
		l3*atr*cosf(u(tr_i, 0))*u(wr_i, 0) -
		sr*bnr*sinf(u(tr_i, 0))*u(wr_i, 0);
	
	y(1, 0) = l1*cosf(u(tl_i, 0))*atl*u(wl_i, 0) -
		l4*sinf(u(tl_i, 0))*atl*u(wl_i, 0) -
		l4*sinf(u(tr_i, 0))*atr*u(wr_i, 0) + 
		l1*cosf(u(tr_i, 0))*atr*u(wr_i, 0) -
		l2*atb*u(wb_i, 0);

	y(2, 0) = -l3*sinf(u(tl_i, 0))*atl*u(wl_i, 0) -
		sl*bnl*cosf(u(tl_i, 0))*u(wl_i, 0) + 
		l3*sinf(u(tr_i, 0))*atr*u(wr_i, 0) - 
		sr*bnr*cosf(u(tr_i, 0))*u(wr_i, 0) - 
		sb*bnb*u(wb_i, 0);
	return y;
}

/* Calculates the jacobian of thrust forces with respect to the control vector */
matrix::Matrix<float, N2, M> AttControl::thrustforce_jac(matrix::Matrix<float,
	M, 1> u){
	matrix::Matrix<float, N2, M> y;
	y(0, wl_i) = -atl*sinf(u(tl_i, 0)); y(0, wr_i) = -atr*sinf(u(tr_i, 0));
	y(0, wb_i) = 0; y(0, tl_i) = -atl*u(wl_i, 0)*cosf(u(tl_i, 0));
	y(0, tr_i) = -atr*u(wr_i, 0)*cosf(u(tr_i, 0));

	y(2, wl_i) = -atl*cosf(u(tl_i, 0)); y(2, wr_i) = -atr*cosf(u(tr_i, 0));
	y(2, wb_i) = -atb; y(2, tl_i) = atl*u(wl_i, 0)*sinf(u(tl_i, 0));
	y(2, tr_i) = atr*u(wr_i, 0)*sinf(u(tr_i, 0));
	return y;
}

/*  Calculates the jacobian of thrust moments with respect to  the control vector*/
matrix::Matrix<float, N2, M> AttControl::thrustmoment_jac(matrix::Matrix<float,
	M, 1> u){

	matrix::Matrix<float, N2, M> y;
	y(0, wl_i) = l3*atl*cosf(u(tl_i, 0)) - sl*bnl*sinf(u(tl_i, 0));
	y(0, wr_i) = -l3*atr*cosf(u(tr_i, 0)) - sr*bnr*sinf(u(tr_i, 0));
	y(0, wb_i) = 0; 
	y(0, tl_i) = -l3*atl*u(wl_i, 0)*sinf(u(tl_i, 0)) -
		sl*bnl*u(wl_i, 0)*cosf(u(tl_i, 0));
	y(0, tr_i) = l3*atr*u(wr_i, 0)*sinf(u(tr_i, 0)) - 
		sr*bnr*u(wr_i, 0)*cosf(u(tr_i, 0));

	y(1, wl_i) = l1*atl*cosf(u(tl_i, 0)) - l4*atl*sinf(u(tl_i, 0));
	y(1, wr_i) = -l4*atr*sinf(u(tr_i, 0)) + l1*atr*cosf(u(tr_i, 0));
	y(1, wb_i) = -l2*atb;
	y(1, tl_i) = -l1*atl*u(wl_i, 0)*sinf(u(tl_i, 0)) -
		l4*atl*u(wl_i, 0)*cosf(u(tl_i, 0));
	y(1, tr_i) = -l4*atr*u(wr_i, 0)*cosf(u(tr_i, 0)) - 
		l1*atr*u(wr_i, 0)*sinf(u(tr_i, 0));

	y(2, wl_i) = -l3*atl*sinf(u(tl_i, 0)) - sl*bnl*cosf(u(tl_i, 0));
	y(2, wr_i) = l3*atr*sinf(u(tr_i, 0)) - sr*bnr*cosf(u(tr_i, 0));
	y(2, wb_i) = -sb*bnb;
	y(2, tl_i) = -l3*atl*u(wl_i, 0)*cosf(u(tl_i, 0)) +
		sl*bnl*u(wl_i, 0)*sinf(u(tl_i, 0));
	y(2, tr_i) = l3*atr*u(wr_i, 0)*cosf(u(tr_i, 0)) +
		sr*bnr*u(wr_i, 0)*sinf(u(tr_i, 0));

	return y;
}

#endif