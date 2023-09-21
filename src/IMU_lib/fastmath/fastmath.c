#include "fastmath.h"
#include <stdint.h>
#include <stdlib.h>


typedef unsigned int u32;

typedef union
{
    float f;
    unsigned int i;
} fasthelperf;

typedef union
{
    double d;
    unsigned long long i;
} fasthelper;

#ifndef __CAST
    #define cast_u32 (u32)
#endif // __CAST


/** --------------------------------------------------------------------
    ---------- random
    --------------------------------------------------------------------
    Description : returns pseudo random number between mini and maxi
    Parameters  :
    Returns     : integer between mini and maxi
    Nota        : random number generator must be initialize with randomSeed function
    ------------------------------------------------------------------*/

int fastrandomi(int mini, int maxi)
{
    return ( (rand() % maxi ) - mini );
}

/** --------------------------------------------------------------------
    ---------- bounds
    ------------------------------------------------------------------*/

long fastboundsu(long x, long _min, long _max)
{
    if (_max < _min) {
        long temp = _max;
        _max = _min;
        _min = temp;
    }

    if (x > _max) return _max;
    if (x < _min) return _min;
    return x;
}

/** --------------------------------------------------------------------
    ---------- Map
    --------------------------------------------------------------------
    Description : map(value, fromLow, fromHigh, toLow, toHigh)
    Parameters  :
    Returns     :
    ------------------------------------------------------------------*/
int fasmapi(int x, int in_min, int in_max, int out_min, int out_max)
{
    return (x - in_min) * (out_max - out_min) / (in_max - in_min) + out_min;
}



/*
 ****************************************************************************************
 * Min & max function
 ****************************************************************************************
*/
float fastminf(float x, float y)
{
    return (x < y) ? x : y;
}

float fastmaxf(float x, float y)
{
    return (x > y) ? x : y;
}

/*
 ****************************************************************************************
 * abs function
 ****************************************************************************************
*/
float fastabsf(float x)
{
    // copy and re-interpret as 32 bit integer
    int casted = *(int*) &x;
    // clear highest bit
    casted &= 0x7FFFFFFF;

    // re-interpret as float
    return *(float*)&casted;
}

double fastabs(double x)
{
    // copy and re-interpret as 64 bit integer
    long long casted = *(long long*) &x;
    // clear highest bit
    casted &= 0x7FFFFFFFFFFFFFFFLL;

    // re-interpret as double
    return *(double*)&casted;
}

int absi_simple(int v)
{
    return (v < 0) ? (int)(-(unsigned)v) : v;
}

float fabs_simple(float v)
{
    return (v < 0.0f) ? -(float)v : v;
}


/*
 ****************************************************************************************
 * sqrt function
 ****************************************************************************************
*/

// This algorithm is dependant on IEEE representation and only works for 32 bits
float fastsqrtf(float x)
{
    unsigned int i = *(unsigned int*) &x; 
    // adjust bias
    i  += 127 << 23;
    // approximation of square root
    i >>= 1; 
    return *(float*) &i;
}

// fast find sqrt(x)
float fastSqrtf_math(float n)
{
    /*
     * *********************************************************************************************
     * 1) HIGH precision for math.
     *  -Advantages:
     *  * The convergence of the function is high enough to be used in applications other than games.
     *
     *  -Disadvantages:
     *  * Computing times are much larger.
     *  * The square root of “0” is a number very close to “0” but never “0”.
     *  * The division operation is the bottleneck in this function. because the division operation is more expensive than the other arithmetic operations of Arithmetic Logic Units (ALU)
     **********************************************************************************************
    */
    fasthelperf u;
    u.i = 0x1FB5AD00 + (*(int*)&n >> 1);
    u.f = n / u.f + u.f;
    return n / u.f + u.f * 0.25f;
}

float fastSqrtf_game1(float n)
{
    /*
     ********************************************************************************************
     * 2) The fastest algorithm but not accurate. (less then origin square)
     * -Advantages:
     *      * The convergence of the function is greater than that of the first method.
     *      * Generates times equal to or greater than the first method.
     *
     * -Disadvantages:
     *      * The square root of “0” is a number very close to “0” but never “0”.
     *******************************************************************************************
    */
    fasthelperf u;
    u.i = 0x5F375A86 - (*(int*)&n >> 1);
    return ((int)3 - n * u.f * u.f) * n * u.f * 0.5f;
}

float fastSqrtf_game2(float n)
{
    /*
     * ******************************************************************************************
     * 3) LOW precision for games. (more than origin square)
     *  -Advantages:
     *      * When Root of 0 is calculated the function returns 0.
     *      * The convergence of the function is acceptable enough for games.
     *      * It generates very good times.
     *      * The Reciprocal of the root can be calculated by removing the second “n” from the third line. According to the property of: 1 / sqrt (n) * n = sqrt (n).
     *
     *  -Disadvantages:
     *      * Convergence decreases when the root to be calculated is very large.
     * *****************************************************************************************
     */
	
	fasthelperf u;
    u.i = 0x2035AD0C + (*(int*)&n >> 1);
    return n / u.f + u.f * 0.25f;
}



/*
 ****************************************************************************************
 * (1 / sqrt) function
 ****************************************************************************************
*/
// fast find 1 / sqrt(x) (float). The following code is the fast inverse square root implementation from Quake III Arena
float fastinvsqrtf(float number)
{
    long i;
    float x2, y;
    const float threehalfs = 1.5F;

    x2 = number * 0.5F;
    y  = number;
    i  = * ( long * ) &y;                       // evil floating point bit level hacking
    i  = 0x5f3759df - ( i >> 1 );               // what the fuck? 
    y  = * ( float * ) &i;
    y  = y * ( threehalfs - ( x2 * y * y ) );   // 1st iteration
    y  = y * ( threehalfs - ( x2 * y * y ) );   // 2nd iteration, this can be removed

    return y;
}

// fast find 1 / sqrt(x) (double)
double fastinvsqrt(double x)
{
    double y = x;
    double xhalf = ( double )0.5 * y;
    long long i = *( long long* )( &y );
    i = 0x5fe6ec85e7de30daLL - ( i >> 1 ); //LL suffix for (long long) type for GCC
    y = *( double* )( &i );
    y = y * ( ( double )1.5 - xhalf * y * y );
    return y;
}


/*
 ****************************************************************************************
 * pow2, exp function
 ****************************************************************************************
*/

// Underflow of exponential is common practice in numerical routines,
// so handle it here.

float fastpow2(float p)
{
  float offset = (p < 0) ? 1.0f : 0.0f;
  float clipp = (p < -126) ? -126.0f : p;
  int w = (int)clipp;
  float z = clipp - w + offset;
  fasthelperf v;
  v.f = cast_u32 ( (1 << 23) * (clipp + 121.2740575f + 27.7280233f / (4.84252568f - z) - 1.49012907f * z) );
  return v.f;
}

float fastexp(float p)
{
  return fastpow2(1.442695040f * p);
}

float fasterpow2(float p)
{
  float clipp = (p < -126) ? -126.0f : p;
  fasthelperf v;
  v.f = cast_u32 ( (1 << 23) * (clipp + 126.94269504f) );
  return v.f;
}

float fasterexp(float p)
{
  return fasterpow2(1.442695040f * p);
}


/*
 ****************************************************************************************
 * log2, pow,ln,log function
 ****************************************************************************************
*/

float fastlog2(float x)
{
  float y;
  fasthelperf vx, mx;
  vx.f = x;
  mx.i = (vx.i & 0x007FFFFF) | 0x3f000000;
  y = (float)vx.i;
  y *= 1.1920928955078125e-7f;

  return y - 124.22551499f
           - 1.498030302f * mx.f 
           - 1.72587999f / (0.3520887068f + mx.f);
}

float fastpow(float x, float p)
{
    return fastpow2(p * fastlog2(x));
}

float fastln(float x)
{
    return 0.69314718f * fastlog2(x);
}

float fastlog(float x)
{
    return 0.30102999f * fastlog2(x);
}

float fasterlog2(float x)
{
    float y;
    fasthelperf vx;
    vx.f = x;
    y = vx.i;
    y *= 1.1920928955078125e-7f;
    return y - 126.94269504f;
}

float fasterpow(float x, float p)
{
    return fasterpow2(p * fasterlog2(x));
}

float fasterln(float x)
{
    //  return 0.69314718f * fasterlog2(x);

    float y;
    fasthelperf vx;
    vx.f = x;
    y = vx.i;
    y *= 8.2629582881927490e-8f;
    return y - 87.989971088f;
}

float fasterlog(float x)
{
    return 0.30102999f * fasterlog2(x);
}

/*
 ****************************************************************************************
 * ERF function
 ****************************************************************************************
*/


// fasterfc: not actually faster than erfcf(3) on newer machines!
// ... although vectorized version is interesting
//     and fastererfc is very fast

float fasterfc (float x)
{
  static const float k = 3.3509633149424609f;
  static const float a = 0.07219054755431126f;
  static const float b = 15.418191568719577f;
  static const float c = 5.609846028328545f;
  float xsq = x * x;
  float xquad = xsq * xsq;
  fasthelperf vc;
  
  vc.f = c * x;
  vc.i |= 0x80000000;

  return 2.0f / (1.0f + fastpow2 (k * x)) - a * x * (b * xquad - 1.0f) * fasterpow2 (vc.f);
}

float fastererfc (float x)
{
  static const float k = 3.3509633149424609f;

  return 2.0f / (1.0f + fasterpow2 (k * x));
}

// fasterf: not actually faster than erff(3) on newer machines! 
// ... although vectorized version is interesting
//     and fastererf is very fast

float fasterf (float x)
{
  return 1.0f - fasterfc (x);
}

float fastererf (float x)
{
  return 1.0f - fastererfc (x);
}

float fastinverseerf (float x)
{
  static const float invk = 0.30004578719350504f;
  static const float a = 0.020287853348211326f;
  static const float b = 0.07236892874789555f;
  static const float c = 0.9913030456864257f;
  static const float d = 0.8059775923760193f;

  float xsq = x * x;

  return invk * fastlog2 ((1.0f + x) / (1.0f - x)) 
       + x * (a - b * xsq) / (c - d * xsq);
}

float fasterinverseerf (float x)
{
  static const float invk = 0.30004578719350504f;

  return invk * fasterlog2 ((1.0f + x) / (1.0f - x));
}
/*
 ****************************************************************************************
 *  gamma/digamma function
 ****************************************************************************************
*/
/* gamma/digamma functions only work for positive inputs */

float fastlgamma (float x)
{
  float logterm = fastlog (x * (1.0f + x) * (2.0f + x));
  float xp3 = 3.0f + x;

  return - 2.081061466f 
         - x 
         + 0.0833333f / xp3 
         - logterm 
         + (2.5f + x) * fastlog (xp3);
}

float fasterlgamma (float x)
{
  return - 0.0810614667f 
         - x
         - fasterlog (x)
         + (0.5f + x) * fasterlog (1.0f + x);
}

float fastdigamma (float x)
{
  float twopx = 2.0f + x;
  float logterm = fastlog (twopx);

  return (-48.0f + x * (-157.0f + x * (-127.0f - 30.0f * x))) /
         (12.0f * x * (1.0f + x) * twopx * twopx)
         + logterm;
}

float fasterdigamma (float x)
{
  float onepx = 1.0f + x;

  return -1.0f / x - 1.0f / (2 * onepx) + fasterlog (onepx);
}

/*
 ****************************************************************************************
 *   HYPERBOLIC function
 ****************************************************************************************
*/
float fastsinh (float p)
{
  return 0.5f * (fastexp (p) - fastexp (-p));
}

float fastersinh (float p)
{
  return 0.5f * (fasterexp (p) - fasterexp (-p));
}

float fastcosh (float p)
{
  return 0.5f * (fastexp (p) + fastexp (-p));
}

float fastercosh (float p)
{
  return 0.5f * (fasterexp (p) + fasterexp (-p));
}

float fasttanh (float p)
{
  return -1.0f + 2.0f / (1.0f + fastexp (-2.0f * p));
}

float fastertanh (float p)
{
  return -1.0f + 2.0f / (1.0f + fasterexp (-2.0f * p));
}


/*
 ****************************************************************************************
 *   ARC_HYPERBOLIC function
 ****************************************************************************************
*/
float fastasin(float x)
{
    const float halfpi = 1.5707963267948966f;
    const float a0 = 1.5707288;
    const float a1 = -0.2121144;
    const float a2 = 0.0742610;
    const float a3 = -0.0187293;

    //float xx = abs(x);

    return (halfpi - fastsqrtf(1-x) * (a0 + a1*x + a2*x*x + a3*x*x*x));
}

float fastacos(float x)
{
    float negate = (float)(x < 0);
    float ret = -0.0187293;
    x = fastabsf(x);
    ret = ret * x;
    ret = ret + 0.0742610;
    ret = ret * x;
    ret = ret - 0.2121144;
    ret = ret * x;
    ret = ret + 1.5707288;
    ret = ret * fastsqrtf(1.0-x);
    ret = ret - 2 * negate * ret;
    return negate * 3.14159265358979 + ret;
}

float fastatan2(float y, float x)
{
    //http://pubs.opengroup.org/onlinepubs/009695399/functions/atan2.html
    //Volkan SALMA

    const float pi = 3.1415926535897932384626433832795;
    const float ONEQTR_PI = pi / 4.0;
    const float THRQTR_PI = 3.0 * pi / 4.0;
    float r, angle;
    float abs_y = fastabsf(y) + 1e-10f;      // kludge to prevent 0/0 condition
    if ( x < 0.0f )
    {
        r = (x + abs_y) / (abs_y - x);
        angle = THRQTR_PI;
    }
    else
    {
        r = (x - abs_y) / (x + abs_y);
        angle = ONEQTR_PI;
    }
    angle += (0.1963f * r * r - 0.9817f) * r;
    if ( y < 0.0f )
        return( -angle );     // negate if in quad III or IV
    else
        return( angle );
}


/*
 ****************************************************************************************
 *   LAMBERT_W function
 ****************************************************************************************
*/

// these functions compute the upper branch aka W_0

float fastlambertw (float x)
{
  static const float threshold = 2.26445f;

  float c = (x < threshold) ? 1.546865557f : 1.0f;
  float d = (x < threshold) ? 2.250366841f : 0.0f;
  float a = (x < threshold) ? -0.737769969f : 0.0f;

  float logterm = fastlog (c * x + d);
  float loglogterm = fastlog (logterm);

  float minusw = -a - logterm + loglogterm - loglogterm / logterm;
  float expminusw = fastexp (minusw);
  float xexpminusw = x * expminusw;
  float pexpminusw = xexpminusw - minusw;

  return (2.0f * xexpminusw - minusw * (4.0f * xexpminusw - minusw * pexpminusw)) /
         (2.0f + pexpminusw * (2.0f - minusw));
}

float fasterlambertw (float x)
{
  static const float threshold = 2.26445f;

  float c = (x < threshold) ? 1.546865557f : 1.0f;
  float d = (x < threshold) ? 2.250366841f : 0.0f;
  float a = (x < threshold) ? -0.737769969f : 0.0f;

  float logterm = fasterlog (c * x + d);
  float loglogterm = fasterlog (logterm);

  float w = a + logterm - loglogterm + loglogterm / logterm;
  float expw = fasterexp (-w);

  return (w * w + expw * x) / (1.0f + w);
}

float fastlambertwexpx (float x)
{
  static const float k = 1.1765631309f;
  static const float a = 0.94537622168f;

  float logarg = fastmaxf(x, k);
  float powarg = (x < k) ? a * (x - k) : 0;

  float logterm = fastlog (logarg);
  float powterm = fasterpow2 (powarg);  // don't need accuracy here

  float w = powterm * (logarg - logterm + logterm / logarg);
  float logw = fastlog (w);
  float p = x - logw;

  return w * (2.0f + p + w * (3.0f + 2.0f * p)) /
         (2.0f - p + w * (5.0f + 2.0f * w));
}

float fasterlambertwexpx (float x)
{
  static const float k = 1.1765631309f;
  static const float a = 0.94537622168f;

  float logarg = fastmaxf(x, k);
  float powarg = (x < k) ? a * (x - k) : 0;

  float logterm = fasterlog (logarg);
  float powterm = fasterpow2 (powarg);

  float w = powterm * (logarg - logterm + logterm / logarg);
  float logw = fasterlog (w);

  return w * (1.0f + x - logw) / (1.0f + w);
}


/*
 ****************************************************************************************
 *   SIGMOID function
 ****************************************************************************************
*/
float fastsigmoid (float x)
{
  return 1.0f / (1.0f + fastexp (-x));
}

float fastersigmoid (float x)
{
  return 1.0f / (1.0f + fasterexp (-x));
}

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

float fastsin (float x)
{
  static const float fouroverpi = 1.2732395447351627f;
  static const float fouroverpisq = 0.40528473456935109f;
  static const float q = 0.78444488374548933f;
  fasthelperf p, r, s, vx;
  u32 sign;
  
  p.f = 0.20363937680730309f;
  r.f = 0.015124940802184233f;
  s.f = -0.0032225901625579573f;

  vx.f = x;
  sign = vx.i & 0x80000000;
  vx.i = vx.i & 0x7FFFFFFF;

  float qpprox = fouroverpi * x - fouroverpisq * x * vx.f;
  float qpproxsq = qpprox * qpprox;

  p.i |= sign;
  r.i |= sign;
  s.i ^= sign;

  return q * qpprox + qpproxsq * (p.f + qpproxsq * (r.f + qpproxsq * s.f));
}

float fastersin (float x)
{
  static const float fouroverpi = 1.2732395447351627f;
  static const float fouroverpisq = 0.40528473456935109f;
  static const float q = 0.77633023248007499f;
  fasthelperf p, vx;
  u32 sign;
  
  p.f = 0.22308510060189463f;
  vx.f = x;
  sign = vx.i & 0x80000000;
  vx.i &= 0x7FFFFFFF;

  float qpprox = fouroverpi * x - fouroverpisq * x * vx.f;

  p.i |= sign;

  return qpprox * (q + p.f * qpprox);
}

float fastsinfull (float x)
{
  static const float twopi = 6.2831853071795865f;
  static const float invtwopi = 0.15915494309189534f;

  //int k = x * invtwopi;
  float k = x * invtwopi;
  float half = (x < 0) ? -0.5f : 0.5f;
  return fastsin ((half + k) * twopi - x);
}

float fastersinfull (float x)
{
  static const float twopi = 6.2831853071795865f;
  static const float invtwopi = 0.15915494309189534f;

  //int k = x * invtwopi;
  float k = x * invtwopi;
  float half = (x < 0) ? -0.5f : 0.5f;
  return fastersin ((half + k) * twopi - x);
}

float fastcos (float x)
{
  static const float halfpi = 1.5707963267948966f;
  static const float halfpiminustwopi = -4.7123889803846899f;
  float offset = (x > halfpi) ? halfpiminustwopi : halfpi;
  return fastsin (x + offset);
}

float fastercos (float x)
{
  static const float twooverpi = 0.63661977236758134f;
  static const float p = 0.54641335845679634f;
  fasthelperf vx;
  vx.f = x;
  vx.i &= 0x7FFFFFFF;

  float qpprox = 1.0f - twooverpi * vx.f;

  return qpprox + p * qpprox * (1.0f - qpprox * qpprox);
}

float fastcosfull (float x)
{
  static const float halfpi = 1.5707963267948966f;
  return fastsinfull (x + halfpi);
}

float fastercosfull (float x)
{
  static const float halfpi = 1.5707963267948966f;
  return fastersinfull (x + halfpi);
}

float fasttan (float x)
{
  static const float halfpi = 1.5707963267948966f;
  return fastsin (x) / fastsin (x + halfpi);
}

float fastertan (float x)
{
  return fastersin (x) / fastercos (x);
}

float fasttanfull (float x)
{
  static const float twopi = 6.2831853071795865f;
  static const float invtwopi = 0.15915494309189534f;

  //int k = x * invtwopi;
  float k = x * invtwopi;
  float half = (x < 0) ? -0.5f : 0.5f;
  float xnew = x - (half + k) * twopi;

  return fastsin (xnew) / fastcos (xnew);
}

float fastertanfull (float x)
{
  static const float twopi = 6.2831853071795865f;
  static const float invtwopi = 0.15915494309189534f;

  //int k = x * invtwopi;
  float k = x * invtwopi;
  float half = (x < 0) ? -0.5f : 0.5f;
  float xnew = x - (half + k) * twopi;

  return fastersin (xnew) / fastercos (xnew);
}


