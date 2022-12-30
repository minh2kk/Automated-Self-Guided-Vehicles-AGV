#ifndef FILTER_H
#define FILTER_H

#include <stdbool.h>
#include <stdlib.h>

#include <math.h>
#include "stm32f4xx.h"
static float minh_x_hat[11], minh_P[11];


void multiply(float* A, float* B, unsigned char m, unsigned char p, unsigned char n, float* C);
void addition(float* A, float* B, unsigned char m, unsigned char n, float* C);
void subtraction(float* A, float* B, unsigned char m, unsigned char n, float* C);
void transpose(float* A, unsigned char m, unsigned char n, float* C);
void copy(int m, int n, float* A, float* B);
void getCofactor(float** A, int k, float** temp, int p, int q, int n);
float determinant(float** A, int k);
void adjoint(float** A, int k, float** adj);
void inversion(float** A, int k, float** inverse);

float LPF(float x, float CUTOFF, float SAMPLE_RATE);
float HPF(float x, float CUTOFF, float SAMPLE_RATE);
void kalman_signal(uint16_t* a, uint16_t* b, float measure_noice, float process_noice);
float kalman_signal1(float z, float measure_noice, float process_noice);
float kalman_signal2(float z, float measure_noice, float process_noice);
float kalman_signal3(float z, float measure_noice, float process_noice);
float kalman_signal4(float z, float measure_noice, float process_noice);
float kalman_signal5(float z, float measure_noice, float process_noice);
float kalman_signal6(float z, float measure_noice, float process_noice);
float kalman_signal7(float z, float measure_noice, float process_noice);
float kalman_signal8(float z, float measure_noice, float process_noice);
float kalman_signal9(float z, float measure_noice, float process_noice);
float kalman_signal10(float z, float measure_noice, float process_noice);
float kalman_signal11(float z, float measure_noice, float process_noice);
void kalman(float* in, float* out, float measure_noice, float process_noice);



#endif

