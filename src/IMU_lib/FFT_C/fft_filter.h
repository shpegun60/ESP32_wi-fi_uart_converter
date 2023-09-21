#ifndef _FFT_FILTER_H_
#define _FFT_FILTER_H_


#ifdef __cplusplus
extern "C" {
#endif


#include "complex_m.h"

typedef struct
{
    unsigned int N_coefs;
    float* filterCoefs;

    unsigned int N_points;
    unsigned int dataIterator;
    TComplex* MeasuredData;
    TComplex* transformData;
    TComplex* FiltersedData;
} FFTFilter_t;

FFTFilter_t* fftFilter_constructor(unsigned int N_coefs, float* coefs, unsigned int N_points);
void fftFilteringProceed(FFTFilter_t* m);

#ifdef __cplusplus
}
#endif

#endif
