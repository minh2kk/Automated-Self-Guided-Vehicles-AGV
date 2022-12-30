#ifndef MATRIX_H
#define MATRIX_H

#include <stdbool.h>

void multiply(float* A, float* B, unsigned char m, unsigned char p, unsigned char n, float* C);
void additon(float* A, float* B, unsigned char m, unsigned char n, float* C);
void subtracton(float* A, float* B, unsigned char m, unsigned char n, float* C);
void transpose(float* A, unsigned char m, unsigned char n, float* C);
void copy(int m, int n, float* A, float* B);
void getCofactor(float** A, int k, float** temp, int p, int q, int n);
float determinant(float** A, int k);
void adjoint(float** A, int k, float** adj);
bool inversion(float** A, int k, float** inverse);

#endif

