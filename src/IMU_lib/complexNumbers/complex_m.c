#include "complex_m.h"
#include "fastmath.h"
#include "smart_assert.h"

#include <math.h>
#include <stdlib.h>
#include <stdio.h>
#include <limits.h>

char *complex_ctoa(TComplex* num)
{
    M_Assert_Break((num == NULL), "getResultRawSize: incorrect input values", return NULL);

    static char buf[(sizeof(char) * (CHAR_BIT * sizeof(COMPLEX_TYPE) + 5))];
    char sign = '+';
    if(fabs(num->im) != num->im) {
        sign = '-';
    }
    sprintf(buf, "%.8Lg%c%.8Lgi", num->re, sign, fabs(num->im)); // +3 to account for the ',' and 'i'
    return buf;
}

TComplex* complex_constructor(COMPLEX_TYPE re, COMPLEX_TYPE im)
{
    TComplex* m = (TComplex*) malloc(sizeof(TComplex));
    M_Assert_BreakSaveCheck((m == NULL), "complex_constructor: no memories for allocation data", return NULL);
    m->re = re;
    m->im = im;
    return m;
}

void complex_destructor(TComplex** out)
{
    M_Assert_Break((out == NULL || *out == NULL), "complex_destructor: incorrect input values", return);
    free(*out);
    *out = NULL;
}

void complex_set(COMPLEX_TYPE re, COMPLEX_TYPE im, TComplex* out) // Change values of a complex function
{
    M_Assert_Break((out == NULL ), "complex_set: incorrect input values", return);
    out->re = re;
    out->im = im;
}

void complex_copy(TComplex* A, TComplex* out)
{
    M_Assert_Break((out == NULL  || A == NULL), "complex_copy: incorrect input values", return);
    out->re = A->re;
    out->im = A->im;
}

void complex_copy_conj(TComplex* A, TComplex* out)
{
    M_Assert_Break((out == NULL  || A == NULL), "complex_copy: incorrect input values", return);
    out->re =  A->re;
    out->im = -A->im;
}

void complex_add(TComplex* A, TComplex* B, TComplex* out)
{
    M_Assert_Break((A == NULL || B == NULL || out == NULL), "complex_add: incorrect input values", return);
    out->re = A->re + B->re;
    out->im = A->im + B->im;
}

void complex_subs(TComplex* A, TComplex* B, TComplex* out)
{
    M_Assert_Break((A == NULL || B == NULL || out == NULL), "complex_subs: incorrect input values", return);
    out->re = A->re - B->re;
    out->im = A->im - B->im;
}

void complex_divide(TComplex* A, TComplex* B, TComplex* out)
{
    M_Assert_Break((A == NULL || B == NULL || out == NULL), "complex_divide: incorrect input values", return);
    TComplex result;
    COMPLEX_FUSION_TYPE denominator = B->re * B->re + B->im * B->im;
    result.re = (A->re * B->re + A->im * B->im) / denominator;
    result.im = (A->im * B->re - A->re * B->im) / denominator;
    *out = result;
}

void complex_multiply(TComplex* A, TComplex* B, TComplex* out)
{
    M_Assert_Break((A == NULL || B == NULL || out == NULL), "complex_multiply: incorrect input values", return);
    TComplex result;
    result.re = A->re * B->re - A->im * B->im;
    result.im = A->re * B->im + A->im * B->re;
    *out = result;
}

void complex_scalarMultiply(TComplex* A, COMPLEX_TYPE val, TComplex* out)
{
    M_Assert_Break((A == NULL || out == NULL), "complex_scalarMultiply: incorrect input values", return);
    out->re = val * A->re;
    out->im = val * A->im;
}

void complex_conjugate(TComplex* A, TComplex* out)
{
    M_Assert_Break((A == NULL || out == NULL), "complex_conjugate: incorrect input values", return);
    out->re =  A->re;
    out->im = -A->im;
}

void complex_power(TComplex* A, COMPLEX_FUSION_TYPE power, TComplex* out)
{
    M_Assert_Break((A == NULL || out == NULL), "complex_power: incorrect input values", return);
    out->re = fastpow(A->re, power);
    out->im = fastpow(A->im, power);
}

void complex_sqrt(TComplex* A, TComplex* out) // Calculate sqrt of a complex number, with the re and the iminary results.
{
    M_Assert_Break((A == NULL || out == NULL), "complex_sqrt: incorrect input values", return);
    COMPLEX_FUSION_TYPE a = A->re * A->re;
    COMPLEX_FUSION_TYPE b = A->im * A->im;

    COMPLEX_FUSION_TYPE rep = cos(0.5f * atan2(A->im, A->re));
    COMPLEX_FUSION_TYPE imp = sin(0.5f * atan2(A->im, A->re));

    out->re = fastpow(a + b, 0.25) * rep;
    out->im = fastpow(a + b, 0.25) * imp;
}


COMPLEX_FUSION_TYPE complex_radangle(TComplex* A)
{
    M_Assert_Break((A == NULL ), "complex_sqrt: incorrect input values", return (COMPLEX_FUSION_TYPE)0.0f);
    COMPLEX_FUSION_TYPE angle = atan2(A->im, A->re);
    return angle;
}

COMPLEX_FUSION_TYPE complex_degangle(TComplex* A)
{
    M_Assert_Break((A == NULL ), "complex_degangle: incorrect input values", return (COMPLEX_FUSION_TYPE)0.0f);
    COMPLEX_FUSION_TYPE angle = complex_radangle(A) * 180 / M_PI;
    return (COMPLEX_FUSION_TYPE)angle;
}

COMPLEX_FUSION_TYPE complex_abs(TComplex* A)
{
    M_Assert_Break((A == NULL ), "complex_abs: incorrect input values", return (COMPLEX_FUSION_TYPE)0.0f);
    COMPLEX_FUSION_TYPE d = fastSqrtf_math((A->re * A->re) + (A->im * A->im));
    return d;
}


COMPLEX_FUSION_TYPE degtorad(COMPLEX_FUSION_TYPE deg)
{
    COMPLEX_FUSION_TYPE rad = deg * M_PI / 180;
    return rad;
}
COMPLEX_FUSION_TYPE radtodeg(COMPLEX_FUSION_TYPE rad)
{
    COMPLEX_FUSION_TYPE deg = rad * 180 / M_PI;
    return deg;
}



