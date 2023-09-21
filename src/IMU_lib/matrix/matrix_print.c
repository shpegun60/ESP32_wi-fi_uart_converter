#include <stdio.h>
#include <stdlib.h>
#include "matrix_print.h"
#include "smart_assert.h"

//--------------------------PRINT RESULT----------------------------------------------------------------------------
void printMat_dynamic(MAT_TYPE** A, int r, int c,  char* name)
{
    M_Assert_Break((A == NULL), "printMat_dynamic: NULL matrix", return);

    if((r == 0) || (c == 0)) {
        printf("[]");
        return;
    }

    printf("%s [%d:%d]\t\n", name, r, c);
    printf("[");
    for( int i = 0; i < r; i++){
        for (int j = 0; j < c; j++) {
            printf("\t%.4f", (float)A[i][j]);
        }

        if(i == (r - 1)) {
            printf("\t]\n");
        } else {
            printf("\n");
        }
    }
    printf("\n");
    fflush(stdout);
}

void printMat_static(MAT_TYPE (*A)[MAT_STATIC_SIZE], int r, int c,  char* name)
{
    M_Assert_Break((A == NULL), "printMat_static: NULL matrix", return);
    M_Assert_Break((MAT_STATIC_SIZE < c), "printMat_static: static matrix is less than input matrix, you must change #define MAT_STATIC_SIZE to input r", return);

    if((r == 0) || (c == 0)) {
        printf("[]");
        return;
    }

    printf("%s [%d:%d]\t\n", name, r, c);
    printf("[");
    for( int i = 0; i < r; i++){
        for (int j = 0; j < c; j++) {
            printf("\t%.4f", (float)A[i][j]);
        }

        if(i == (r - 1)) {
            printf("\t]\n");
        } else {
            printf("\n");
        }
    }
    printf("\n");
    fflush(stdout);
}
//----------------------------------------------------------------------------------------------------------



