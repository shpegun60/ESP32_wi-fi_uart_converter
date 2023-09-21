#ifndef __MATRIX_PRINT_H_
#define __MATRIX_PRINT_H_

#include "matrix_port.h"

void printMat_dynamic(MAT_TYPE** A, int r, int c,  char* name);
void printMat_static(MAT_TYPE (*A)[MAT_STATIC_SIZE], int r, int c,  char* name);

#endif /* __MATRIX_PRINT_H_ */
