#ifndef PRINT_DEBUG_H
#define PRINT_DEBUG_H

#include <stdio.h>

template <class M, class N>
class PrintDebug{

    public:
        // Constructor
        PrintDebug();

        //Main methods

        // Prints a float matrix
        static void print_matrix(M x, int n_rows, int n_cols);

        //Prints a float vector
        static void print_vector(M x, int n_rows);

};

template <class M, class N>
PrintDebug<M, N>::PrintDebug(){}

template <class M, class N>
void PrintDebug<M, N>::print_matrix(M x, int n_rows, int n_cols){
    /* 
    x:the matrix to be printed
    n_rows: matrix number of rows
    n_cols: matrix number of columns
    */

    for (int i = 0; i < n_rows; i++){

        for (int j = 0; j < n_cols; j ++){  

            if ((j + 1) == n_cols){
                printf("%8.4f\n", (N) x(i, j));
            }
            
            else{
                printf("%8.4f", (N) x(i, j));
            }
        }

        if ((i + 1) == n_rows){
            printf("\v");
        } 
    }
}

template <class M, class N>
void PrintDebug<M, N>::print_vector(M x, int n_rows){
    /* 
    x:the vector to be printed
    n_rows: vector's number of rows
    */

    for (int i = 0; i < n_rows; i++){

        if ((i + 1) == n_rows){
            printf("\v");
        } 
        else{
            printf("%8.4f\n", (N) x(i));
        }
    }
}


// Example code 
/*
    const int n_x = 8;
    const int n_y = 8;


    // NxM matrix
    matrix::Matrix<float, n_x, n_y> P;
    P.setZero(); 
    P(0, 0) = (float) 8.32;
    P(2, 0) = (float) 0.03;
    P(1, 1) = (float) 0.9;

    typedef matrix::Matrix<float, n_x, n_y> matrix_type;
    PrintDebug<matrix_type, double>::print_matrix(P, n_x, n_y);

    // Vector plot
    matrix::Vector<float, n_x> x;
    x(0) = 5; x(3) = 6; x(7) = 23;
    typedef matrix::Vector<float, n_x> vector_type;
    PrintDebug<vector_type, double>::print_vector(x, n_x);
    */




#endif