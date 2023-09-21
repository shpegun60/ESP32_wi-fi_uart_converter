//   fft.h - declaration of
//   fast Fourier transform - FFT
//
//   The code is property of LIBROW
//   You can use it on your own
//   When utilizing credit LIBROW site

#ifndef _FFT_H_
#define _FFT_H_

//   C++ linking for mixed C++/C code
#ifdef __cplusplus
extern "C" {
#endif

#include "complex_m.h"


//   FORWARD FOURIER TRANSFORM
//     Input  - input data
//     Output - transform result
//     N      - length of both input data and result
extern unsigned int FFTForward(const TComplex *const Input, TComplex *const Output, const unsigned int N);

//   FORWARD FOURIER TRANSFORM, from input cycl buffer
//     Input  - input data
//     Output - transform result
//     N      - length of both input data and result
//     startPos - start position for read input
extern unsigned int FFTForward_fromPos(const TComplex *const Input, TComplex *const Output, const unsigned int N, const unsigned int startPos);


//   FORWARD FOURIER TRANSFORM, INPLACE VERSION
//     Data - both input data and output
//     N    - length of input data
extern unsigned int FFTForwardInplace(TComplex *const Data, const unsigned int N);

//   INVERSE FOURIER TRANSFORM
//     Input  - input data
//     Output - transform result
//     N      - length of both input data and result
//     Scale  - if to scale result: 1 - scale, 0 - do not scale
extern unsigned int FFTInverse(const TComplex *const Input, TComplex *const Output, const unsigned int N, const unsigned int Scale);

//   INVERSE FOURIER TRANSFORM, INPLACE VERSION
//     Data  - both input data and output
//     N     - length of both input data and result
//     Scale - if to scale result: 1 - scale, 0 - do not scale
extern unsigned int FFTInverseInplace(TComplex *const Data, const unsigned int N, const unsigned int Scale);

#ifdef __cplusplus
}
#endif

#endif
