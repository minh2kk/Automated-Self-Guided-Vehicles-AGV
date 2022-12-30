#include "matrix.h"
#include <stdlib.h>
#include <math.h>

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

void additon(float* A, float* B, unsigned char m, unsigned char n, float* C)
{
	int i, j;
	for(i=0;i<m;i++){
		for(j=0;j<n;j++){
			C[n*i+j]=A[n*i+j]+B[n*i+j];
		}
	}
}

void subtracton(float* A, float* B, unsigned char m, unsigned char n, float* C)
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
        return;
    }
 
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

bool inversion(float** A, int k, float** inverse)
{
		float** adj;
			int i,j;
    // Find determinant of A[][]
    float det = determinant(A, k);
    if (det == 0) {
        //cout << "Singular matrix, can't find its inverse";
        return false;
    }
 
    // Find adjoint

    adjoint(A, k, adj);
 
    // Find Inverse using formula "inverse(A) =
    // adj(A)/det(A)"

    for (i = 0; i < k; i++){
			for (j = 0; j < k; j++){
				inverse[i][j] = adj[i][j] / det;
			}
		}
        
            
 
    return true;
}

