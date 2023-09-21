#include "inversion_matrix.h"
#include "smart_assert.h"

//***************************************************************************************************************
// Function to calculate and store inverse, returns false if
// matrix is singular
//***************************************************************************************************************

int inverse_dynamic(MAT_TYPE** A, MAT_TYPE** inverse, unsigned int n)
{
    M_Assert_Break((A == NULL || inverse == NULL), "inverse_dynamic: NULL matrix", return MAT_FAIL);
    M_Assert_Break((n == 0), "inverse_dynamic: empty matrix", return MAT_FAIL);
    M_Assert_Break((MAT_STATIC_SIZE < n), "inverse_dynamic: static matrix is less than input matrix, you must change #define MAT_STATIC_SIZE to input n", return MAT_FAIL);


    // Find determinant of A[][]
    double det = determinant_dynamic(A, n);
    M_Assert_WarningSaveCheck((det == 0.0), "inverse_dynamic: Singular matrix, can't find its inverse", return MAT_FAIL);

    double invdet = (double)(1.0/det);

    // Find adjoint
    MAT_TYPE adj[MAT_STATIC_SIZE][MAT_STATIC_SIZE];
    adjoint_dynamic(A, adj, n);

    // Find Inverse using formula "inverse(A) = adj(A)/det(A)"
    for (unsigned int i = 0; i < n; ++i) {
        for (unsigned int j = 0; j < n; ++j) {
            inverse[i][j] = adj[i][j] * invdet;
        }
    }
    return MAT_SUCC;
}

int fast_inverse_dynamic(MAT_TYPE** A, MAT_TYPE** inverse, unsigned int n)
{
    M_Assert_Break((A == NULL || inverse == NULL), "fast_inverse_dynamic: NULL matrix", return MAT_FAIL);
    M_Assert_Break((A == inverse), "fast_inverse_dynamic: on this case not work, must differential adresses", return MAT_FAIL);
    M_Assert_Break((n == 0), "fast_inverse_dynamic: empty matrix", return MAT_FAIL);
    M_Assert_Break((MAT_STATIC_SIZE < n), "fast_inverse_dynamic: static matrix is less than input matrix, you must change #define MAT_STATIC_SIZE to input n", return MAT_FAIL);

    // Find determinant of A[][]
    double det = determinant_dynamic(A, n);
    M_Assert_WarningSaveCheck((det == 0.0), "fast_inverse_dynamic: Singular matrix, can't find its inverse", return MAT_FAIL);

    double invdet = (double)(1.0/det);

    // Find Inverse using formula "inverse(A) = adj(A)/det(A)"
    invMat(A, inverse, invdet, n);
    return MAT_SUCC;
}

int inverse_static(MAT_TYPE (*A)[MAT_STATIC_SIZE], MAT_TYPE (*inverse)[MAT_STATIC_SIZE], unsigned int n)
{
    M_Assert_Break((A == NULL || inverse == NULL), "inverse_static: NULL matrix", return MAT_FAIL);
    M_Assert_Break((n == 0), "inverse_static: empty matrix", return MAT_FAIL);
    M_Assert_Break((MAT_STATIC_SIZE < n), "inverse_static: static matrix is less than input matrix, you must change #define MAT_STATIC_SIZE to input n", return MAT_FAIL);

    // Find determinant of A[][]
    double det = determinant_static(A, n);
    M_Assert_WarningSaveCheck((det == 0.0), "inverse_static: Singular matrix, can't find its inverse", return MAT_FAIL);

    double invdet = (double)(1.0/det);

    // Find adjoint
    MAT_TYPE adj[MAT_STATIC_SIZE][MAT_STATIC_SIZE];
    adjoint_static(A, adj, n);

    // Find Inverse using formula "inverse(A) = adj(A)/det(A)"
    for (unsigned int i = 0; i < n; ++i) {
        for (unsigned int j = 0; j < n; ++j) {
            inverse[i][j] = adj[i][j] * invdet;
        }
    }
    return MAT_SUCC;
}

int inverse_from_static_to_dynamic(MAT_TYPE (*A)[MAT_STATIC_SIZE], MAT_TYPE** inverse, unsigned int n)
{
    M_Assert_Break((A == NULL || inverse == NULL), "inverse_from_static_to_dynamic: NULL matrix", return MAT_FAIL);
    M_Assert_Break((n == 0), "inverse_from_static_to_dynamic: empty matrix", return MAT_FAIL);
    M_Assert_Break((MAT_STATIC_SIZE < n), "inverse_from_static_to_dynamic: static matrix is less than input matrix, you must change #define MAT_STATIC_SIZE to input n", return MAT_FAIL);

    // Find determinant of A[][]
    double det = determinant_static(A, n);
    M_Assert_WarningSaveCheck((det == 0.0), "inverse_from_static_to_dynamic: Singular matrix, can't find its inverse", return MAT_FAIL);

    double invdet = (double)(1.0/det);

    // Find adjoint
    MAT_TYPE adj[MAT_STATIC_SIZE][MAT_STATIC_SIZE];
    adjoint_static(A, adj, n);

    // Find Inverse using formula "inverse(A) = adj(A)/det(A)"
    for (unsigned int i = 0; i < n; ++i) {
        for (unsigned int j = 0; j < n; ++j) {
            inverse[i][j] = adj[i][j] * invdet;
        }
    }
    return MAT_SUCC;
}

int inverse_from_dynamic_to_static(MAT_TYPE** A, MAT_TYPE (*inverse)[MAT_STATIC_SIZE], unsigned int n)
{
    M_Assert_Break((A == NULL || inverse == NULL), "inverse_from_dynamic_to_static: NULL matrix", return MAT_FAIL);
    M_Assert_Break((n == 0), "inverse_from_dynamic_to_static: empty matrix", return MAT_FAIL);
    M_Assert_Break((MAT_STATIC_SIZE < n), "inverse_from_dynamic_to_static: static matrix is less than input matrix, you must change #define MAT_STATIC_SIZE to input n", return MAT_FAIL);

    // Find determinant of A[][]
    double det = determinant_dynamic(A, n);
    M_Assert_WarningSaveCheck((det == 0.0), "inverse_from_dynamic_to_static: Singular matrix, can't find its inverse", return MAT_FAIL);

    double invdet = (double)(1.0/det);

    // Find adjoint
    MAT_TYPE adj[MAT_STATIC_SIZE][MAT_STATIC_SIZE];
    adjoint_dynamic(A, adj, n);

    // Find Inverse using formula "inverse(A) = adj(A)/det(A)"
    for (unsigned int i = 0; i < n; ++i) {
        for (unsigned int j = 0; j < n; ++j) {
            inverse[i][j] = adj[i][j] * invdet;
        }
    }
    return MAT_SUCC;
}



/*
 * ****************************************************************************************************************************
 *      ADJOINT
 * ****************************************************************************************************************************
 */

// Function to get adjoint of A[N][N] in adj[N][N].
void adjoint_static(MAT_TYPE (*A)[MAT_STATIC_SIZE], MAT_TYPE (*adj)[MAT_STATIC_SIZE], unsigned int n)
{
    M_Assert_Break((A == NULL || adj == NULL), "adjoint_static: NULL matrix", return);
    M_Assert_Break((n == 0), "adjoint_static: empty matrix", return);
    M_Assert_Break((MAT_STATIC_SIZE < n), "adjoint_static: static matrix is less than input matrix, you must change #define MAT_STATIC_SIZE to input n", return);

    if (n == 1) {
        adj[0][0] = (MAT_TYPE)1.0f;
        return;
    }

    // temp is used to store cofactors of A[][]
    int sign = 1;
    MAT_TYPE temp[MAT_STATIC_SIZE][MAT_STATIC_SIZE];

    for (unsigned int i = 0; i < n; ++i) {
        for (unsigned int j = 0; j < n; ++j) {
            // Get cofactor of A[i][j]
            getCofactor_static(A, temp, i, j, n);

            // sign of adj[j][i] positive if sum of row
            // and column indexes is even.
            sign = ((i + j) & 0x01) ? -1 : 1; //((i+j)%2==0)? 1: -1;

            // Interchanging rows and columns to get the
            // transpose of the cofactor matrix
            adj[j][i] = (sign)*(determinant_static(temp, n-1));
        }
    }
}


// Function to get adjoint of A[N][N] in adj[N][N].
void adjoint_dynamic(MAT_TYPE** A, MAT_TYPE (*adj)[MAT_STATIC_SIZE], unsigned int n)
{
    M_Assert_Break((A == NULL || adj == NULL), "adjoint_dynamic: NULL matrix", return);
    M_Assert_Break((n == 0), "adjoint_dynamic: empty matrix", return);
    M_Assert_Break((MAT_STATIC_SIZE < n), "adjoint_dynamic: static matrix is less than input matrix, you must change #define MAT_STATIC_SIZE to input n", return);

    if (n == 1) {
        adj[0][0] = (MAT_TYPE)1.0f;
        return;
    }

    // temp is used to store cofactors of A[][]
    int sign = 1;
    MAT_TYPE temp[MAT_STATIC_SIZE][MAT_STATIC_SIZE];

    for (unsigned int i = 0; i < n; ++i) {
        for (unsigned int j = 0; j < n; ++j) {
            // Get cofactor of A[i][j]
            getCofactor_dynamic(A, temp, i, j, n);

            // sign of adj[j][i] positive if sum of row
            // and column indexes is even.
            sign = ((i + j) & 0x01) ? -1 : 1; //((i+j)%2==0)? 1: -1;

            // Interchanging rows and columns to get the
            // transpose of the cofactor matrix
            adj[j][i] = (sign)*(determinant_static(temp, n-1));
        }
    }
}

// Function to get invMat of A[N][N] if we know inverse determinant (1.0/det)
void invMat(MAT_TYPE** A, MAT_TYPE** dest, double invdet, unsigned int n)
{
    M_Assert_Break((A == NULL || dest == NULL), "invMat: NULL matrix", return);
    M_Assert_Break((A == dest), "invMat: on this case not work, must differential adresses", return);
    M_Assert_Break((n == 0), "invMat: empty matrix", return);
    M_Assert_Break((MAT_STATIC_SIZE < n), "invMat: static matrix is less than input matrix, you must change #define MAT_STATIC_SIZE to input n", return);

    if (n == 1) {
        dest[0][0] = (MAT_TYPE)invdet;
        return;
    }

    // temp is used to store cofactors of A[][]
    int sign = 1;
    MAT_TYPE temp[MAT_STATIC_SIZE][MAT_STATIC_SIZE];

    for (unsigned int i = 0; i < n; ++i) {
        for (unsigned int j = 0; j < n; ++j) {
            // Get cofactor of A[i][j]
            getCofactor_dynamic(A, temp, i, j, n);

            // sign of dest[j][i] positive if sum of row
            // and column indexes is even.
            sign = ((i + j) & 0x01) ? -1 : 1; //((i+j)%2==0)? 1: -1;

            // Interchanging rows and columns to get the
            // transpose of the cofactor matrix
            dest[j][i] = (MAT_TYPE)(((sign)*(determinant_static(temp, n-1))) * invdet);
        }
    }
}


/*
 * ****************************************************************************************************************************
 *      DETERMINANT
 * ****************************************************************************************************************************
 */

/* Recursive function for finding determinant of matrix.
   n is current dimension of A[][]. */
float determinant_dynamic(MAT_TYPE** A, unsigned int n)
{
    M_Assert_Break((A == NULL), "determinant_dynamic: NULL matrix", return ((MAT_TYPE)0));
    M_Assert_Break((n == 0), "determinant_dynamic: empty matrix", return ((MAT_TYPE)0));
    M_Assert_Break((MAT_STATIC_SIZE < n), "determinant_dynamic: static matrix is less than input matrix, you must change #define MAT_STATIC_SIZE to input n", return ((MAT_TYPE)0));

    float D = 0.0f; // Initialize result

    //  Base case : if matrix contains single element
    if (n == 1) {
        return ((float)A[0][0]);
    }

    MAT_TYPE temp[MAT_STATIC_SIZE][MAT_STATIC_SIZE]; // To store cofactors

    int sign = 1;  // To store sign multiplier

    // Iterate for each element of first row
    for (unsigned int f = 0; f < n; ++f) {
        // Getting Cofactor of A[0][f]
        getCofactor_dynamic(A, temp, 0, f, n);
        D += (float)(sign * A[0][f] * determinant_static(temp, n - 1));

        // terms are to be added with alternate sign
        sign = -sign;
    }

    return D;
}

float determinant_static(MAT_TYPE (*A)[MAT_STATIC_SIZE], unsigned int n)
{
    M_Assert_Break((A == NULL), "determinant_static: NULL matrix", return ((MAT_TYPE)0));
    M_Assert_Break((n == 0), "determinant_static: empty matrix", return ((MAT_TYPE)0));
    M_Assert_Break((MAT_STATIC_SIZE < n), "determinant_static: static matrix is less than input matrix, you must change #define MAT_STATIC_SIZE to input n", return ((MAT_TYPE)0));

    float D = 0.0f; // Initialize result

    //  Base case : if matrix contains single element
    if (n == 1) {
        return ((float)A[0][0]);
    }

    MAT_TYPE temp[MAT_STATIC_SIZE][MAT_STATIC_SIZE]; // To store cofactors

    int sign = 1;  // To store sign multiplier

    // Iterate for each element of first row
    for (unsigned int f = 0; f < n; ++f) {
        // Getting Cofactor of A[0][f]
        getCofactor_static(A, temp, 0, f, n);
        D += (float)(sign * A[0][f] * determinant_static(temp, n - 1));

        // terms are to be added with alternate sign
        sign = -sign;
    }

    return D;
}

/*
 * ****************************************************************************************************************************
 *      COFACTOR
 * ****************************************************************************************************************************
 */

// Function to get cofactor of A[p][q] in temp[][]. n is current
// dimension of A[][]
void getCofactor_dynamic(MAT_TYPE** A, MAT_TYPE (*temp)[MAT_STATIC_SIZE], unsigned int p, unsigned int q, unsigned int n)
{
    M_Assert_Break((A == NULL || temp == NULL), "getCofactor_dynamic: NULL matrix", return);

    unsigned int i = 0, j = 0;

    // Looping for each element of the matrix
    for (unsigned int row = 0; row < n; ++row) {
        for (unsigned int col = 0; col < n; ++col) {
            //  Copying into temporary matrix only those element
            //  which are not in given row and column
            if (row != p && col != q) {
                temp[i][j++] = A[row][col];

                // Row is filled, so increase row index and
                // reset col index
                if (j == n - 1) {
                    j = 0;
                    ++i;
                }
            }
        }
    }
}


void getCofactor_static(MAT_TYPE (*A)[MAT_STATIC_SIZE], MAT_TYPE (*temp)[MAT_STATIC_SIZE], unsigned int p, unsigned int q, unsigned int n)
{
    M_Assert_Break((A == NULL || temp == NULL), "getCofactor_static: NULL matrix", return);

    unsigned int i = 0, j = 0;

    // Looping for each element of the matrix
    for (unsigned int row = 0; row < n; ++row) {
        for (unsigned int col = 0; col < n; ++col) {
            //  Copying into temporary matrix only those element
            //  which are not in given row and column
            if (row != p && col != q) {
                temp[i][j++] = A[row][col];

                // Row is filled, so increase row index and
                // reset col index
                if (j == n - 1) {
                    j = 0;
                    ++i;
                }
            }
        }
    }
}


