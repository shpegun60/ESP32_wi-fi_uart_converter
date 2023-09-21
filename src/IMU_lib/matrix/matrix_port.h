#ifndef __MATRIX_PORT_H_
#define __MATRIX_PORT_H_

#define MAT_SUCC 1
#define MAT_FAIL -1

#ifndef NULL
    #define NULL ((void *)0)
#endif

/* current representation of a matrix in my mind  */

#define MAT_TYPE float

#define MAT_STATIC_SIZE 4 // maximum size of matrix [N x N] for calculation inverse and static matrix printf


#endif /* __MATRIX_PORT_H_ */
