#ifndef __COMPLEX_M_C__H_
#define __COMPLEX_M_C__H_

#define COMPLEX_TYPE float
#define COMPLEX_FUSION_TYPE float

//   Complex number type, COMPLEX_TYPE
typedef struct
{
    COMPLEX_TYPE re;
    COMPLEX_TYPE im;
} TComplex;


TComplex* complex_constructor(COMPLEX_TYPE re, COMPLEX_TYPE im);
void complex_destructor(TComplex** out);

char *complex_ctoa(TComplex* num);

void complex_set(COMPLEX_TYPE re, COMPLEX_TYPE im, TComplex* out); // Change values of a complex function
void complex_copy(TComplex* A, TComplex* out);
void complex_copy_conj(TComplex* A, TComplex* out);

void complex_add(TComplex* num1, TComplex* num2, TComplex* out);
void complex_subs(TComplex* A, TComplex* B, TComplex* out);
void complex_divide(TComplex* A, TComplex* B, TComplex* out);
void complex_multiply(TComplex* A, TComplex* B, TComplex* out);
void complex_scalarMultiply(TComplex* A, COMPLEX_TYPE val, TComplex* out);
void complex_conjugate(TComplex* A, TComplex* out);
void complex_power(TComplex* A, COMPLEX_FUSION_TYPE power, TComplex* out);
void complex_sqrt(TComplex* A, TComplex* out); // Calculate sqrt of a complex number, with the re and the iminary results.

COMPLEX_FUSION_TYPE complex_radangle(TComplex* A);
COMPLEX_FUSION_TYPE complex_degangle(TComplex* A);

COMPLEX_FUSION_TYPE complex_abs(TComplex* A);


COMPLEX_FUSION_TYPE degtorad(COMPLEX_FUSION_TYPE deg);
COMPLEX_FUSION_TYPE radtodeg(COMPLEX_FUSION_TYPE rad);


#endif // __COMPLEX_M_C__H_
