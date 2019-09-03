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

#endif

