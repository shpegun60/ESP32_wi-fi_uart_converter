#include "fft_filter.h"
#include "fft.h"
#include "smart_assert.h"

#include <stdlib.h>

FFTFilter_t* fftFilter_constructor(unsigned int N_coefs, float* coefs, unsigned int N_points)
{
    M_Assert_BreakSaveCheck((N_coefs < 1 ||  N_points < 1 || (N_points & (N_points - 1)) || coefs == NULL || N_coefs >= N_points), "fftFiletr_constructor: no valid input values", return NULL);

    FFTFilter_t* m = (FFTFilter_t*)malloc(sizeof(FFTFilter_t));
    M_Assert_BreakSaveCheck((m == NULL), "fftFiletr_constructor: no memories for allocation data", return NULL);

    m->filterCoefs = (float*)malloc(N_coefs * sizeof(float));
    M_Assert_BreakSaveCheck((m->filterCoefs == NULL), "fftFilter_constructor: no memories for allocation data", return NULL);
    for(unsigned int i = 0; i < N_coefs; ++i) {
        m->filterCoefs[i] = coefs[i];
    }
    m->N_coefs = N_coefs;

    m->MeasuredData = (TComplex*)calloc(N_points, sizeof(TComplex));
    M_Assert_BreakSaveCheck((m->MeasuredData == NULL), "fftFilter_constructor: no memories for allocation data", return NULL);

    m->transformData = (TComplex*)calloc(N_points, sizeof(TComplex));
    M_Assert_BreakSaveCheck((m->transformData == NULL), "fftFilter_constructor: no memories for allocation data", return NULL);

    m->FiltersedData = (TComplex*)calloc(N_points, sizeof(TComplex));
    M_Assert_BreakSaveCheck((m->FiltersedData == NULL), "fftFilter_constructor: no memories for allocation data", return NULL);

    m->N_points = N_points;
    m->dataIterator = 0;
    return m;
}


void fftFilteringProceed(FFTFilter_t* m)
{
   M_Assert_Break((m == NULL || m->MeasuredData == NULL || m->transformData == NULL || m->FiltersedData == NULL || m->filterCoefs == NULL), "fftFilteringProceed: no input valid data", return);

   m->dataIterator = (m->dataIterator + 1) & (m->N_points - 1);
   FFTForward_fromPos(m->MeasuredData, m->transformData, m->N_points, m->dataIterator);

   complex_scalarMultiply(&m->transformData[m->N_coefs], m->filterCoefs[0], &m->transformData[0]);
   for (unsigned int i = 1; i < m->N_coefs; ++i) {
       complex_scalarMultiply(&m->transformData[m->N_coefs], m->filterCoefs[i], &m->transformData[i]);
       complex_copy_conj(&m->transformData[i], &m->transformData[m->N_points - i]);
   }

   FFTInverse(m->transformData, m->FiltersedData, m->N_points, 1);
}
