#ifndef __FASTMATH__H_
#define __FASTMATH__H_

#define _USE_MATH_DEFINES
#include <math.h>

//   C++ linking for mixed C++/C code
#ifdef __cplusplus
extern "C" {
#endif

#ifndef MAX
    #define MAX(a, b) ((a) > (b) ? (a) : (b))
#endif // MAX

#ifndef SORT_4
    #define SORT_4(a, b, c, d) (MAX(MAX((a), (b)), MAX((c), (d))))
#endif // SORT_4

#ifndef FABS
    #define FABS(x) ((x) < 0.0f ? -(x) : (x))
#endif // FABS

#ifndef TO_RAD
    #define TO_RAD(x) (((x) / 180.0f) * M_PI)
#endif // TO_RAD

#ifndef TO_DEG
    #define TO_DEG(x) (((x) * 180.0f) / M_PI)
#endif // TO_DEG

#ifndef INV_SQRT_2
    #define INV_SQRT_2 0.70710678118654752440f
#endif // INV_SQRT_2





/** --------------------------------------------------------------------
    ---------- random
    --------------------------------------------------------------------
    Description : returns pseudo random number between mini and maxi
    Parameters  :
    Returns     : integer between mini and maxi
    Nota        : random number generator must be initialize with randomSeed function
    ------------------------------------------------------------------*/

int fastrandomi(int mini, int maxi);

/** --------------------------------------------------------------------
    ---------- bounds
    ------------------------------------------------------------------*/

long fastboundsu(long x, long _min, long _max);

/** --------------------------------------------------------------------
    ---------- Map
    --------------------------------------------------------------------
    Description : map(value, fromLow, fromHigh, toLow, toHigh)
    Parameters  :
    Returns     :
    ------------------------------------------------------------------*/

int fastmapi(int x, int in_min, int in_max, int out_min, int out_max);

/*
 ****************************************************************************************
 * Min & max function
 ****************************************************************************************
*/
float fastminf(float x, float y);
float fastmaxf(float x, float y);

/*
 ****************************************************************************************
 * abs function
 ****************************************************************************************
*/
float fastabsf(float x);
double fastabs(double x);
int absi_simple(int v);
float fabs_simple(float v);
/*
 ****************************************************************************************
 * sqrt function
 ****************************************************************************************
*/

// This algorithm is dependant on IEEE representation and only works for 32 bits
float fastsqrtf(float x);
// fast find sqrt(x)
float fastSqrtf_math(float n);
float fastSqrtf_game1(float n);
float fastSqrtf_game2(float n);



/*
 ****************************************************************************************
 * (1 / sqrt) function
 ****************************************************************************************
*/
// fast find 1 / sqrt(x) (float). The following code is the fast inverse square root implementation from Quake III Arena
float fastinvsqrtf(float number);
// fast find 1 / sqrt(x) (double)
double fastinvsqrt(double x);

/*
 ****************************************************************************************
 * pow2, exp function
 ****************************************************************************************
*/

// Underflow of exponential is common practice in numerical routines,
// so handle it here.

float fastpow2(float p);
float fastexp(float p);
float fasterpow2(float p);
float fasterexp(float p);

/*
 ****************************************************************************************
 * log2, pow,ln,log function
 ****************************************************************************************
*/

float fastlog2(float x);
float fastpow(float x, float p);
float fastln(float x);
float fastlog(float x);
float fasterlog2(float x);
float fasterpow(float x, float p);
float fasterln(float x);
float fasterlog(float x);
/*
 ****************************************************************************************
 * ERF function
 ****************************************************************************************
*/


// fasterfc: not actually faster than erfcf(3) on newer machines!
// ... although vectorized version is interesting
//     and fastererfc is very fast

float fasterfc (float x);
float fastererfc (float x);

// fasterf: not actually faster than erff(3) on newer machines! 
// ... although vectorized version is interesting
//     and fastererf is very fast

float fasterf (float x);
float fastererf (float x);
float fastinverseerf (float x);
float fasterinverseerf (float x);
/*
 ****************************************************************************************
 *  gamma/digamma function
 ****************************************************************************************
*/
/* gamma/digamma functions only work for positive inputs */

float fastlgamma (float x);
float fasterlgamma (float x);
float fastdigamma (float x);
float fasterdigamma (float x);
/*
 ****************************************************************************************
 *   HYPERBOLIC function
 ****************************************************************************************
*/
float fastsinh (float p);
float fastersinh (float p);
float fastcosh (float p);
float fastercosh (float p);
float fasttanh (float p);
float fastertanh (float p);

/*
 ****************************************************************************************
 *   ARC_HYPERBOLIC function
 ****************************************************************************************
*/
float fastasin(float x);
float fastacos(float x);
float fastatan2(float y, float x);


/*
 ****************************************************************************************
 *   LAMBERT_W function
 ****************************************************************************************
*/

// these functions compute the upper branch aka W_0

float fastlambertw (float x);
float fasterlambertw (float x);
float fastlambertwexpx (float x);
float fasterlambertwexpx (float x);
/*
 ****************************************************************************************
 *   SIGMOID function
 ****************************************************************************************
*/
float fastsigmoid (float x);
float fastersigmoid (float x);
/*
 ****************************************************************************************
 *   TRIGONOMETRIC function
 ****************************************************************************************
*/
// http://www.devmaster.net/forums/showthread.php?t=5784
// fast sine variants are for x \in [ -\pi, pi ]
// fast cosine variants are for x \in [ -\pi, pi ]
// fast tangent variants are for x \in [ -\pi / 2, pi / 2 ]
// "full" versions of functions handle the entire range of inputs
// although the range reduction technique used here will be hopelessly
// inaccurate for |x| >> 1000
//
// WARNING: fastsinfull, fastcosfull, and fasttanfull can be slower than
// libc calls on older machines (!) and on newer machines are only 
// slighly faster.  however:
//   * vectorized versions are competitive
//   * faster full versions are competitive

float fastsin (float x);
float fastersin (float x);
float fastsinfull (float x);
float fastersinfull (float x);
float fastcos (float x);
float fastercos (float x);

float fastcosfull (float x);
float fastercosfull (float x);

float fasttan (float x);
float fastertan (float x);

float fasttanfull (float x);

float fastertanfull (float x);

#ifdef __cplusplus
}
#endif


#endif // __FASTMATH__H_
