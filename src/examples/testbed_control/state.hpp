#ifndef STATE_H
#define STATE_H


#include <math.h>
#include <matrix/math.hpp>
#include "att_control.hpp"

class State{


    public:
        // Constructor
        State();

        // Main methods
        static void state_est(struct state_vec_s *state_vec, 
            struct measurements_vec_s *measurements_vec,
            struct control_vec_s *control_vec, struct covariance_mat_s *cov_mat); 

        static void state_init(struct state_vec_s *state_vec, 
            struct measurements_vec_s *measurements_vec,
            struct covariance_mat_s *cov_mat); 

    private:

        static matrix::Matrix<float, Ns, 1> system_model( 
            matrix::Matrix<float, Ns, 1> x_,  matrix::Matrix<float, M, 1> u_);
        
        static matrix::Matrix<float, Ns, Ns> system_jac(
            matrix::Matrix<float, Ns, 1> x_, matrix::Matrix<float, M, 1> u_);
};

State::State(){}

void State::state_est(struct state_vec_s *state_vec, 
    struct measurements_vec_s *measurements_vec,
    struct control_vec_s *control_vec, struct covariance_mat_s *cov_mat){
        
        /****************** Extended Kalman Filter ******************/

        // Previous state
        matrix::Matrix<float, Ns, 1> x_;
        x_(0, 0) = state_vec->w; x_(1, 0) = state_vec->q;
        x_(2, 0) = state_vec->z; x_(3, 0) = state_vec->theta;

        // Control vector
        matrix::Matrix<float, M, 1> u_;
        u_(0, 0) = control_vec->wl_t; u_(1, 0) = control_vec->wr_t;

        /****************** Prediction ******************/
        // System model Runge-Kutta 4 Integration
        float h_s = 1.e-6; // integration step
        matrix::Matrix<float, Ns, 1> f1 = system_model(x_, u_);
        matrix::Matrix<float, Ns, 1> f2 = system_model(x_ + (h_s/2)*f1, u_);
        matrix::Matrix<float, Ns, 1> f3 = system_model(x_ + (h_s/2)*f2, u_);
        matrix::Matrix<float, Ns, 1> f4 = system_model(x_ + h_s*f3, u_);
        x_ = x_ + (h_s / 6) * (f1 + (float) 2 * f2 + (float) 2 * f3 + f4);

        // System jacobian
        matrix::Matrix<float, Ns, Ns> A = system_jac(x_, u_);

        // Process noise matrix
        matrix::Matrix<float, Ns, Ns> Q;
        Q(AttControl::w_i, AttControl::w_i) = powf(0.1, 2);
        Q(AttControl::q_i, AttControl::q_i) = powf(0.1, 2);
        Q(AttControl::z_i, AttControl::z_i) = powf(0.1, 2);
        Q(AttControl::theta_i, AttControl::theta_i) = powf(0.1, 2);

        // Measurement noise matrix
        matrix::Matrix<float, P, P> R;
        R(0, 0) = powf(1, 2); R(1, 1) = powf(0.1, 2); R(2, 2) = powf(0.1, 2);

        // Covariance matrix
        matrix::Matrix<float, Ns, Ns> cov_m;
        cov_m(AttControl::w_i, AttControl::w_i) = cov_mat->w; 
        cov_m(AttControl::q_i, AttControl::q_i) = cov_mat->q;
        cov_m(AttControl::z_i, AttControl::z_i) = cov_mat->z;
        cov_m(AttControl::theta_i, AttControl::theta_i) = cov_mat->theta;
        cov_m = cov_m + h_s * (A * cov_m + cov_m * A.transpose() + Q);

        /****************** Correction ******************/
        // Output matrix
        matrix::Matrix<float, P, Ns> C;
        C(0, AttControl::w_i) = 1; C(1, AttControl::q_i) = 1;
        C(2, AttControl::theta_i) = 1;

        // Compute gain
        matrix::SquareMatrix<float, P> mat = 
            (matrix::SquareMatrix<float, P>) (R + C * cov_m * C.transpose());
        matrix::SquareMatrix<float, P> mat_inv = matrix::inv(mat);
        matrix::Matrix<float, Ns, P> L = cov_m * C.transpose() * mat_inv;

        // Eye matrix
        matrix::SquareMatrix<float, Ns> I = matrix::eye<float, Ns>(); 

        // Covariance matrix update
        cov_m = (I - L * C) * cov_m;
        cov_mat->w = cov_m(AttControl::w_i, AttControl::w_i);
        cov_mat->q = cov_m(AttControl::q_i, AttControl::q_i);
        cov_mat->z = cov_m(AttControl::z_i, AttControl::z_i);
        cov_mat->theta = cov_m(AttControl::theta_i, AttControl::theta_i);

        // Meazured Output
        matrix::Matrix<float, P, 1> z_;  
        z_(0, 0) =  measurements_vec->w - measurements_vec->q*
            AttControl::lc*sinf(measurements_vec->theta_);
        z_(1, 0) = state_vec->q; z_(2, 0) =  measurements_vec->theta_;

        // State estimate update
        x_ = x_ + L * (z_ - C * x_);
        state_vec->w = x_(AttControl::w_i, 0);
        state_vec->q = x_(AttControl::q_i, 0);
        state_vec->z = x_(AttControl::z_i, 0);
        state_vec->theta = x_(AttControl::theta_i, 0);
}

void State::state_init(struct state_vec_s *state_vec, 
    struct measurements_vec_s *measurements_vec,
    struct covariance_mat_s *cov_mat){

    /* State components */	

    // Height can't be measured or estimated
    state_vec->z = (float) 0;   
    
     // Pitch angle(rad)
    state_vec->theta = measurements_vec->theta_;   
    
    // Tranlational velocity (m/s)
    state_vec->w = measurements_vec->w - measurements_vec->q*
        AttControl::lc*sinf(measurements_vec->theta_);
    
    // Rotational velocity in y inertial axis( rad/sec)
    state_vec->q = measurements_vec->q;	


    /* Covariance matrix initialization */
    cov_mat->w = powf(0.1, 2);
    cov_mat->q = powf(0.1, 2);
    cov_mat->z = powf(0.5, 2);
    cov_mat->theta = powf(0.1, 2);  
}


matrix::Matrix<float, Ns, 1> State::system_model(matrix::Matrix<float, Ns, 1> x_,
    matrix::Matrix<float, M, 1> u_){

		 // matrix h
		matrix::Matrix<float, N, N>  h_ = AttControl::h_mat(x_);

		// matrix h inverse
		matrix::SquareMatrix<float, N> h_sqr = (matrix::SquareMatrix<float, N>) h_;
		matrix::SquareMatrix<float, N> h_inv = matrix::inv(h_sqr);

		// vector c
		matrix::Matrix<float, N, 1> c_ = AttControl::c_vec(x_);

		// matrix g
		matrix::Matrix<float, N, M> g_ = AttControl::g_mat(x_);

        // vector pdot
        matrix::Matrix<float, N, 1> pdot; pdot = h_inv * (g_*u_ - c_);

        // vector qdot
        matrix::Matrix<float, N, 1> qdot; qdot(0, 0) = x_(AttControl::w_i, 0);
        qdot(1, 0) = x_(AttControl::q_i, 0);

        // state dot
        matrix::Matrix<float, Ns, 1> y;
        y(AttControl::w_i, 0) = pdot(0, 0); y(AttControl::q_i, 0) = pdot(1, 0);
        y(AttControl::z_i, 0) = qdot(0, 0); y(AttControl::theta_i, 0) = qdot(1, 0);
        return y;
}

matrix::Matrix<float, Ns, Ns> State::system_jac(matrix::Matrix<float, Ns, 1> x_,
    matrix::Matrix<float, M, 1> u_){
        
    float h = 1.0e-4; matrix::Matrix<float, Ns, Ns> A; 

        for (int i = 0; i < Ns; i++){
            matrix::Matrix<float, Ns, 1> dx; dx(i, 0) = h;
            matrix::Matrix<float, Ns, 1> a_col;
            a_col = (system_model(x_ + dx, u_) - system_model(x_ - dx, u_))/(2*h);
            A.setCol(i, a_col);
        }
    return A;
}




#endif