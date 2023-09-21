#ifndef __MATRIX_INVERSION_H_
#define __MATRIX_INVERSION_H_

#include "matrix_port.h"


int inverse_dynamic(MAT_TYPE** A, MAT_TYPE** inverse, unsigned int n);
int fast_inverse_dynamic(MAT_TYPE** A, MAT_TYPE** inverse, unsigned int n); // pointers must be not equal if you use this !!!

int inverse_static(MAT_TYPE (*A)[MAT_STATIC_SIZE], MAT_TYPE (*inverse)[MAT_STATIC_SIZE], unsigned int n);
int inverse_from_dynamic_to_static(MAT_TYPE** A, MAT_TYPE (*inverse)[MAT_STATIC_SIZE], unsigned int n);
int inverse_from_static_to_dynamic(MAT_TYPE (*A)[MAT_STATIC_SIZE], MAT_TYPE** inverse, unsigned int n);

void adjoint_static(MAT_TYPE (*A)[MAT_STATIC_SIZE], MAT_TYPE (*adj)[MAT_STATIC_SIZE], unsigned int n);
void adjoint_dynamic(MAT_TYPE** A, MAT_TYPE (*adj)[MAT_STATIC_SIZE], unsigned int n);
void invMat(MAT_TYPE** A, MAT_TYPE** dest, double invdet, unsigned int n); // Function to get invMat of A[N][N] if we know inverse determinant

float determinant_dynamic(MAT_TYPE** A, unsigned int n);
float determinant_static(MAT_TYPE (*A)[MAT_STATIC_SIZE], unsigned int n);

void getCofactor_dynamic(MAT_TYPE** A, MAT_TYPE (*temp)[MAT_STATIC_SIZE], unsigned int p, unsigned int q, unsigned int n);
void getCofactor_static(MAT_TYPE (*A)[MAT_STATIC_SIZE], MAT_TYPE (*temp)[MAT_STATIC_SIZE], unsigned int p, unsigned int q, unsigned int n);


#endif /* __MATRIX_INVERSION_H_ */
