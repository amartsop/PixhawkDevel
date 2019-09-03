#ifndef STRUCTS_MAIN_H
#define STRUCTS_MAIN_H

// Measurements struct
struct measurements_vec_s{
	
	// Translational velocity in body frame
	float u; float v; float w;
	
	// Rotational velocity in body frame
	float p; float q; float r;

	// Euler angles
	float phi_; float theta_; float psi_;

	// Translational acceleration in body frame
	float axb; float ayb; float azb;

	// Previous translational velocity in body frame
	float up; float vp; float wp;
	
	// Previous rotational velocity in body frame
	float pp; float qp; float rp;

	// Previous euler angles
	float phi_p; float theta_p; float psi_p;

	// Previous translational acceleration in body frame
	float axb_p; float ayb_p; float azb_p;

	// time (ms)
	uint64_t t;
	
	// time previous (ms)
	uint64_t t_p;
};

// State vector struct
struct state_vec_s{

	// Translational velocity
	float w;

	// Rotational velocity
	float q;

	// z displacement
	float z;

	// theta angle
	float theta;
};

// Setpoint struct
struct setpoint_vec_s{
	
	// Desired translation velocity
	float w;

	//Desired rotational velocity
	float theta;
};

struct control_vec_s{
	// Rotational velocity of the left motor
	float wl_t;

	// Rotational velocity of the right motor
	float wr_t;
	
};

struct testbed_control_subscription_data_s {

	struct battery_status_s bat_status;
	struct vehicle_attitude_s att;
	struct sensor_bias_s sen_bias;
	struct input_rc_s input_rc_signal;

	int battery_status_sub;
	int sensor_bias_sub;
	int att_sub;
	int input_rc_sub;
};

struct covariance_mat_s{
	
	// Covariance in w estimation
	float w;
	
	// Covariance in q estimation
	float q;

	// Covariance in z estimation
	float z;

	// Covariance in theta estimation
	float theta;

};

#endif

