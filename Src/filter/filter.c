#include "filter.h"


void multiply(float* A, float* B, unsigned char m, unsigned char p, unsigned char n, float* C)
{
	//Matrix Multiplication Routine
	//A = input matrix (m x p)
	//B = input matrix (p x n)
	//m = number of rows in A
	//p = number of columns in A = number of rows in B
	//n = number of columns in B
	//C = output matrix = A*B (m x n)
	int i, j, k;
	for(i=0;i<m;i++){
		for(j=0;j<n;j++){
			C[n*i+j]=0;
			for(k=0;k<p;k++){
				C[n*i+j]=C[n*i+j] + A[p*i+k]*B[n*k+j];
			}
		}
	}
}

void addition(float* A, float* B, unsigned char m, unsigned char n, float* C)
{
	int i, j;
	for(i=0;i<m;i++){
		for(j=0;j<n;j++){
			C[n*i+j]=A[n*i+j]+B[n*i+j];
		}
	}
}

void subtraction(float* A, float* B, unsigned char m, unsigned char n, float* C)
{
	int i, j;
	for(i=0;i<m;i++){
		for(j=0;j<n;j++){
			C[n*i+j]=A[n*i+j]-B[n*i+j];
		}
	}
}

void transpose(float* A, unsigned char m, unsigned char n, float* C){
	int i, j;
	for(i=0;i<m;i++){
		for(j=0;j<n;j++){
			C[m*j+i]=A[n*i+j];
		}
	}
}

void copy(int m, int n, float* A, float* B)
{
	int i, k;
	k=m*n;
	for(i=0;i<k;i++){
		B[i]=A[i];
	}
}


void getCofactor(float** A, int k, float** temp, int p, int q, int n)
{
    int i = 0, j = 0;
		int row,col;
    // Looping for each element of the matrix
    for (row = 0; row < n; row++) {
        for (col = 0; col < n; col++) {
            //  Copying into temporary matrix only those
            //  element which are not in given row and
            //  column
            if (row != p && col != q) {
                temp[i][j++] = A[row][col];
 
                // Row is filled, so increase row index and
                // reset col index
                if (j == n - 1) {
                    j = 0;
                    i++;
                }
            }
        }
    }
}

float determinant(float** A, int k)
{
    float D = 0; // Initialize result
		int sign = 1; // To store sign multiplier
		int f;
		float** temp; // To store cofactors
    //  Base case : if matrix contains single element
    if (k == 1)
        return A[0][0];
    // Iterate for each element of first row
    for (f = 0; f < k; f++) {
        // Getting Cofactor of A[0][f]
        getCofactor(A, k, temp, 0, f, k);
        D += sign * A[0][f] * determinant(temp, k - 1);
 
        // terms are to be added with alternate sign
        sign = -sign;
    }
    return D;
}

void adjoint(float** A, int k, float** adj)
{
		int sign = 1;
		float** temp;
		int i,j;
    if (k == 1) {
        adj[0][0] = 1;
    }else if(k!=1){
			// temp is used to store cofactors of A[][]

			for (i = 0; i < k; i++) {
					for (j = 0; j < k; j++) {
							// Get cofactor of A[i][j]
							getCofactor(A, k, temp, i, j, k);
	 
							// sign of adj[j][i] positive if sum of row
							// and column indexes is even.
							sign = ((i + j) % 2 == 0) ? 1 : -1;
	 
							// Interchanging rows and columns to get the
							// transpose of the cofactor matrix
							adj[j][i] = (sign) * (determinant(temp, k - 1));
					}
			}
		}
}

void inversion(float** A, int k, float** inverse)
{
		float** adj;
			int i,j;
    // Find determinant of A[][]
    float det = determinant(A, k);
//    if (det == 0) {
//        //cout << "Singular matrix, can't find its inverse";
//        return false;
//    }
		if (det != 0) {
			adjoint(A, k, adj);
	 
			// Find Inverse using formula "inverse(A) =
			// adj(A)/det(A)"

			for (i = 0; i < k; i++){
				for (j = 0; j < k; j++){
					inverse[i][j] = adj[i][j] / det;
				}
			}
        
    }
    // Find adjoint
}



float LPF(float x, float CUTOFF, float SAMPLE_RATE)
{
	float RC, dt, alpha, y;
	static float ylast = 0;
	RC = 1.0f/(CUTOFF*2*3.14f);
	dt = 1.0f/SAMPLE_RATE;
	alpha = dt/(RC+dt);
	y = ylast + alpha*(x-ylast);
	ylast=y;
	return y;
}

float HPF(float x, float CUTOFF, float SAMPLE_RATE)
{
	float RC = 1.0f/(CUTOFF*2*3.14f);
	float dt = 1.0f/SAMPLE_RATE;
	float alpha = RC/(RC+dt);
	float y;
	static float xlast = 0, ylast = 0;
	y = alpha*(ylast + x -xlast);
	ylast = y;
	xlast = x;
	return y;
}

float kalman_signal1(float z, float measure_noice, float process_noice)
{
	const float R=measure_noice*measure_noice;
	const float Q=process_noice*process_noice;
	static float x_hat1, P1;
	float P_, K;
	//Noi suy kalman
	P_ = P1+Q;								       	//P_ = A*P*A' + Q
	K = P_/(P_+R);										//K = P_*H'*inv(H*P_*H'+R)
	x_hat1 = x_hat1 + K*(z - x_hat1);	//x_hat = x_hat + K*(z-H*x_hat)
	P1 = (1 - K)*P_;									//P = (1-K*H)*P_
	return x_hat1;
}
float kalman_signal2(float z, float measure_noice, float process_noice)
{
	const float R=measure_noice*measure_noice;
	const float Q=process_noice*process_noice;
	static float x_hat2, P2;
	float P_, K;
	//Noi suy kalman
	P_ = P2+Q;								       	//P_ = A*P*A' + Q
	K = P_/(P_+R);										//K = P_*H'*inv(H*P_*H'+R)
	x_hat2 = x_hat2 + K*(z - x_hat2);	//x_hat = x_hat + K*(z-H*x_hat)
	P2 = (1 - K)*P_;									//P = (1-K*H)*P_
	return x_hat2;
}
float kalman_signal3(float z, float measure_noice, float process_noice)
{
	const float R=measure_noice*measure_noice;
	const float Q=process_noice*process_noice;
	static float x_hat3, P3;
	float P_, K;
	//Noi suy kalman
	P_ = P3+Q;								       	//P_ = A*P*A' + Q
	K = P_/(P_+R);										//K = P_*H'*inv(H*P_*H'+R)
	x_hat3 = x_hat3 + K*(z - x_hat3);	//x_hat = x_hat + K*(z-H*x_hat)
	P3 = (1 - K)*P_;									//P = (1-K*H)*P_
	return x_hat3;
}
float kalman_signal4(float z, float measure_noice, float process_noice)
{
	const float R=measure_noice*measure_noice;
	const float Q=process_noice*process_noice;
	static float x_hat4, P4;
	float P_, K;
	//Noi suy kalman
	P_ = P4+Q;								       	//P_ = A*P*A' + Q
	K = P_/(P_+R);										//K = P_*H'*inv(H*P_*H'+R)
	x_hat4 = x_hat4 + K*(z - x_hat4);	//x_hat = x_hat + K*(z-H*x_hat)
	P4 = (1 - K)*P_;									//P = (1-K*H)*P_
	return x_hat4;
}
float kalman_signal5(float z, float measure_noice, float process_noice)
{
	const float R=measure_noice*measure_noice;
	const float Q=process_noice*process_noice;
	static float x_hat5, P5;
	float P_, K;
	//Noi suy kalman
	P_ = P5+Q;								       	//P_ = A*P*A' + Q
	K = P_/(P_+R);										//K = P_*H'*inv(H*P_*H'+R)
	x_hat5 = x_hat5 + K*(z - x_hat5);	//x_hat = x_hat + K*(z-H*x_hat)
	P5 = (1 - K)*P_;									//P = (1-K*H)*P_
	return x_hat5;
}
float kalman_signal6(float z, float measure_noice, float process_noice)
{
	const float R=measure_noice*measure_noice;
	const float Q=process_noice*process_noice;
	static float x_hat6, P6;
	float P_, K;
	//Noi suy kalman
	P_ = P6+Q;								       	//P_ = A*P*A' + Q
	K = P_/(P_+R);										//K = P_*H'*inv(H*P_*H'+R)
	x_hat6 = x_hat6 + K*(z - x_hat6);	//x_hat = x_hat + K*(z-H*x_hat)
	P6 = (1 - K)*P_;									//P = (1-K*H)*P_
	return x_hat6;
}
float kalman_signal7(float z, float measure_noice, float process_noice)
{
	const float R=measure_noice*measure_noice;
	const float Q=process_noice*process_noice;
	static float x_hat7, P7;
	float P_, K;
	//Noi suy kalman
	P_ = P7+Q;								       	//P_ = A*P*A' + Q
	K = P_/(P_+R);										//K = P_*H'*inv(H*P_*H'+R)
	x_hat7 = x_hat7 + K*(z - x_hat7);	//x_hat = x_hat + K*(z-H*x_hat)
	P7 = (1 - K)*P_;									//P = (1-K*H)*P_
	return x_hat7;
}
float kalman_signal8(float z, float measure_noice, float process_noice)
{
	const float R=measure_noice*measure_noice;
	const float Q=process_noice*process_noice;
	static float x_hat8, P8;
	float P_, K;
	//Noi suy kalman
	P_ = P8+Q;								       	//P_ = A*P*A' + Q
	K = P_/(P_+R);										//K = P_*H'*inv(H*P_*H'+R)
	x_hat8 = x_hat8 + K*(z - x_hat8);	//x_hat = x_hat + K*(z-H*x_hat)
	P8 = (1 - K)*P_;									//P = (1-K*H)*P_
	return x_hat8;
}
float kalman_signal9(float z, float measure_noice, float process_noice)
{
	const float R=measure_noice*measure_noice;
	const float Q=process_noice*process_noice;
	static float x_hat9, P9;
	float P_, K;
	//Noi suy kalman
	P_ = P9+Q;								       	//P_ = A*P*A' + Q
	K = P_/(P_+R);										//K = P_*H'*inv(H*P_*H'+R)
	x_hat9 = x_hat9 + K*(z - x_hat9);	//x_hat = x_hat + K*(z-H*x_hat)
	P9 = (1 - K)*P_;									//P = (1-K*H)*P_
	return x_hat9;
}
float kalman_signal10(float z, float measure_noice, float process_noice)
{
	const float R=measure_noice*measure_noice;
	const float Q=process_noice*process_noice;
	static float x_hat10, P10;
	float P_, K;
	//Noi suy kalman
	P_ = P10+Q;								       	//P_ = A*P*A' + Q
	K = P_/(P_+R);										//K = P_*H'*inv(H*P_*H'+R)
	x_hat10 = x_hat10 + K*(z - x_hat10);	//x_hat = x_hat + K*(z-H*x_hat)
	P10 = (1 - K)*P_;									//P = (1-K*H)*P_
	return x_hat10;
}
float kalman_signal11(float z, float measure_noice, float process_noice)
{
	const float R=measure_noice*measure_noice;
	const float Q=process_noice*process_noice;
	static float x_hat11, P11;
	float P_, K;
	//Noi suy kalman
	P_ = P11+Q;								       	//P_ = A*P*A' + Q
	K = P_/(P_+R);										//K = P_*H'*inv(H*P_*H'+R)
	x_hat11 = x_hat11 + K*(z - x_hat11);	//x_hat = x_hat + K*(z-H*x_hat)
	P11 = (1 - K)*P_;									//P = (1-K*H)*P_
	return x_hat11;
}
void kalman_signal(uint16_t* a, uint16_t* b, float measure_noice, float process_noice){
	const float R=measure_noice*measure_noice;
	const float Q=process_noice*process_noice;
	float P_, K;
	int i = 0;
	for(i = 0; i < 11; i++){
		P_ = *(minh_P+i)+Q;
		K = P_/(P_+R);
		*(minh_x_hat+i) = *(minh_x_hat+i) + K*(*(a+i) - *(minh_x_hat+i));
		*(minh_P+i) = (1 - K)*P_;	
		*(b+i) = *(minh_x_hat+i);
//		//Noi suy kalman
//		P_ = minh_P[i]+Q;								       	//P_ = A*P*A' + Q
//		K = P_/(P_+R);										//K = P_*H'*inv(H*P_*H'+R)
//		minh_x_hat[i] = minh_x_hat[i] + K*(a[i] - minh_x_hat[i]);	//x_hat = x_hat + K*(z-H*x_hat)
//		minh_P[i] = (1 - K)*P_;									//P = (1-K*H)*P_
//		b[i] = minh_x_hat[i];
	}
}
void kalman(float* in, float* out, float measure_noice, float process_noice)
{
	float R[3][3]={0}, Q[3][3]={0};
	/*
		const float A[3][3] = {
														{1,0,0},
														{0,1,0},
														{0,0,1},
													};
		const float H[3][3]	=	{
														{1,0,0},
														{0,1,0},
														{0,0,1},
													};
	*/
	const float B[3][3] = {
													{1,0,0},
													{0,1,0},
													{0,0,1},
												};
	const float I[3][3]	=	{
													{1,0,0},
													{0,1,0},
													{0,0,1},
												};
	static float x_hat[3][1];
	static float P[3][3];
	float z[3][1];
	float x_hat_[3][1];
	float P_[3][3];
	float K[3][3];
	float P_R1[3][3], P_R2[3][3];
	float z_x_hat_[3][1], K_z_x_hat_[3][1];
	float I_K[3][3];
	R[0][0] = measure_noice*measure_noice;
	R[1][1] = measure_noice*measure_noice;
	R[2][2] = measure_noice*measure_noice;
	Q[0][0] = process_noice*process_noice;
	Q[1][1] = process_noice*process_noice;
	Q[2][2] = process_noice*process_noice;
												
	transpose((float*)in, 1, 3, (float*)z);
//	//x_hat_ = A*x_hat + B*u_    <=> x_hat_ = B*x_hat
	multiply((float*)B, (float*)x_hat,3,3,1,(float*)x_hat_);
//	//P_ =A*P*A' +Q <=> P_ =P+Q
	addition((float*)P, (float*)Q, 3,3,(float*)P_);
	
	//K= P_*H'*inv(H*P_*H' +R)	<=> K = P_*inv(P_ + R)
	addition((float*)P_, (float*)R, 3,3,(float*)P_R1);	//P_R1 = P_ + R
	inversion((float**)P_R1, 3,(float**)P_R2);	//P_R2 = inv(P_R1)
	multiply((float*)P_, (float*)P_R2, 3,3,3,(float*)K);
	
	//x_hat = x_hat_ + K*(z - H*x_hat_)	<=> x_hat = x_hat_ + K*(z - x_hat_)
	subtraction((float*)z, (float*)x_hat,3,1,(float*)z_x_hat_);		//z_x_hat_ = z -  x_hat_
	multiply((float*)K, (float*)z_x_hat_, 3,3,1, (float*)K_z_x_hat_);	//K_z_x_hat_ = K*z_x_hat_
	addition((float*)x_hat_, (float*)K_z_x_hat_,3,1,(float*)x_hat);	//x_hat = x_hat_ +K_z_x_hat_
	
	//P = (I - K*H)*P_  <=>  (I-K)*P_
	subtraction((float*)I, (float*)K, 3,3, (float*)I_K);	//I_K = I-K
	multiply((float*)I_K, (float*)P_,3,3,3,(float*)P);	//P = I_K*P_
	transpose((float*)x_hat,3,1,(float*)out);
}

