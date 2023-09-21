//   fft.cpp - impelementation of
//   fast Fourier transform - FFT
//
//   The code is property of LIBROW
//   You can use it on your own
//   When utilizing credit LIBROW site

//   Include declaration file
#include "fft.h"
//   Include math library
#include <math.h>
#include <assert.h>

#define chooseSinFoo(x, value) _Generic(x, float: sinf(value), default: sin(value))

//   Rearrange function
static void FFTRearrange(const TComplex *const Input, TComplex *const Output, const unsigned int N)
{
    //   Position counter
    unsigned int Position;
    //   Bit mask
    unsigned int Mask;
    //   Data entry position
    unsigned int Target = 0;
    //   Process all positions of input signal
    for (Position = 0; Position < N; ++Position) {
        //  Set data entry
        Output[Target] = Input[Position];
        //   Bit mask
        Mask = N;
        //   While bit is set
        while (Target & (Mask >>= 1)) {
            //   Drop bit
            Target &= ~Mask;
        }

        //   The current bit is 0 - set it
        Target |= Mask;
    }
}

static void FFTRearrange_fromPos(const TComplex *const Input, TComplex *const Output, const unsigned int N, const unsigned int startPos)
{
    //   Position counter
    unsigned int Position;
    //   Bit mask
    unsigned int MaskAll = (N - 1);
    unsigned int Mask;
    //   Data entry position
    unsigned int Target = 0;
    //   Process all positions of input signal
    for (Position = 0; Position < N; ++Position) {
        //  Set data entry
        Output[Target] = Input[(Position + startPos) & MaskAll];
        //   Bit mask
        Mask = N;
        //   While bit is set
        while (Target & (Mask >>= 1)) {
            //   Drop bit
            Target &= ~Mask;
        }

        //   The current bit is 0 - set it
        Target |= Mask;
    }
}

//   Inplace version of rearrange function
static void FFTRearrangeInplace(TComplex *const Data, const unsigned int N)
{
    //   Position counter;
    unsigned int Position;
    //   Bit mask
    unsigned int Mask;
    //   Swap position
    unsigned int Target = 0;
    //   Process all positions of input signal
    for (Position = 0; Position < N; ++Position) {
        //   Only for not yet swapped entries
        if (Target > Position) {
            //   Swap entries
            const TComplex Temp = Data[Target];
            Data[Target] = Data[Position];
            Data[Position] = Temp;
        }
        //   Bit mask
        Mask = N;
        //   While bit is set
        while (Target & (Mask >>= 1)) {
            //   Drop bit
            Target &= ~Mask;
        }

        //   The current bit is 0 - set it
        Target |= Mask;
    }
}


//   FFT implementation
static void FFTPerform(TComplex *const Data, const unsigned int N, const unsigned int Inverse)
{
    //   Pi constant
    const COMPLEX_FUSION_TYPE pi = Inverse ? 3.14159265358979323846f : -3.14159265358979323846f;
    //   Cycle counters
    unsigned int Step, Group, Pair;
    //   Jump to the next entry of the same transform factor
    unsigned int Jump;
    //   Angle increment
    COMPLEX_FUSION_TYPE delta;
    //   Auxiliary sin(delta / 2)
    COMPLEX_FUSION_TYPE Sine;
    //   Multiplier for trigonometric recurrence
    TComplex Multiplier;
    //   Transform factor
    TComplex Factor;
    //   Temporary storage for factor real part
    COMPLEX_FUSION_TYPE FactorRe;
    //   Match position
    unsigned int Match;
    //   Second term of two-point transform
    TComplex Product;
    //   Iteration through dyads, quadruples, octads and so on...
    for (Step = 1; Step < N; Step <<= 1) {
        //   Jump to the next entry of the same transform factor
        Jump = Step << 1;
        //   Angle increment
        delta = pi / (COMPLEX_FUSION_TYPE)Step;
        //   Auxiliary sin(delta / 2)

        Sine = chooseSinFoo(Multiplier.re, (delta * .5f)); //sinf(delta * .5f);
        //   Multiplier for trigonometric recurrence
        Multiplier.re = -2.f * Sine * Sine;
        Multiplier.im = chooseSinFoo(Multiplier.im, delta);//sinf(delta);
        //   Start value for transform factor, fi = 0
        Factor.re = 1.f;
        Factor.im = 0.f;
        //   Iteration through groups of different transform factor
        for (Group = 0; Group < Step; ++Group) {
            //   Iteration within group
            for (Pair = Group; Pair < N; Pair += Jump) {
                //   Match position
                Match = Pair + Step;
                //   Second term of two-point transform
                Product.re = Factor.re * Data[Match].re - Factor.im * Data[Match].im;
                Product.im = Factor.re * Data[Match].im + Factor.im * Data[Match].re;
                //   Transform for fi + pi
                Data[Match].re = Data[Pair].re - Product.re;
                Data[Match].im = Data[Pair].im - Product.im;
                //   Transform for fi
                Data[Pair].re += Product.re;
                Data[Pair].im += Product.im;
            }
            //   Successive transform factor via trigonometric recurrence
            FactorRe = Factor.re;
            Factor.re = Multiplier.re * Factor.re - Multiplier.im * Factor.im + Factor.re;
            Factor.im = Multiplier.re * Factor.im + Multiplier.im * FactorRe + Factor.im;
        }
    }
}

//   Scaling of inverse FFT result
static void FFTScale(TComplex *const Data, const unsigned int N)
{
    //   Position counter
    unsigned int Position;
    //   Scale factor
    const COMPLEX_FUSION_TYPE Factor = 1.f / (COMPLEX_FUSION_TYPE)N;
    //   Scale all data entries
    for (Position = 0; Position < N; ++Position) {
        Data[Position].re *= Factor;
        Data[Position].im *= Factor;
    }
}

//   FORWARD FOURIER TRANSFORM
//     Input  - input data
//     Output - transform result
//     N      - length of both input data and result
unsigned int FFTForward(const TComplex *const Input, TComplex *const Output, const unsigned int N)
{
    //   Check input parameters
    if (!Input || !Output || N < 1 || N & (N - 1)) {
        return 0;
    }

    //   Initialize data
    FFTRearrange(Input, Output, N);
    //   Call FFT implementation
    FFTPerform(Output, N, 0);
    //   Succeeded
    return 1;
}

//   FORWARD FOURIER TRANSFORM, from cycl buffer
//     Input  - input data
//     Output - transform result
//     N      - length of both input data and result
//     startPos - start position for read input
unsigned int FFTForward_fromPos(const TComplex *const Input, TComplex *const Output, const unsigned int N, const unsigned int startPos)
{
    //   Check input parameters
    if (!Input || !Output || N < 1 || N & (N - 1) ||  (startPos >= N)) {
        return 0;
    }

    //   Initialize data
    FFTRearrange_fromPos(Input, Output, N, startPos);
    //   Call FFT implementation
    FFTPerform(Output, N, 0);
    //   Succeeded
    return 1;
}

//   FORWARD FOURIER TRANSFORM, INPLACE VERSION
//     Data - both input data and output
//     N    - length of input data
unsigned int FFTForwardInplace(TComplex *const Data, const unsigned int N)
{
    //   Check input parameters
    if (!Data || N < 1 || N & (N - 1)) {
        return 0;
    }

    //   Rearrange
    FFTRearrangeInplace(Data, N);
    //   Call FFT implementation
    FFTPerform(Data, N, 0);
    //   Succeeded
    return 1;
}

//   INVERSE FOURIER TRANSFORM
//     Input  - input data
//     Output - transform result
//     N      - length of both input data and result
//     Scale  - if to scale result: 1 - scale, 0 - do not scale
unsigned int FFTInverse(const TComplex *const Input, TComplex *const Output, const unsigned int N, const unsigned int Scale)
{
    //   Check input parameters
    if (!Input || !Output || N < 1 || N & (N - 1)) {
        return 0;
    }

    //   Initialize data
    FFTRearrange(Input, Output, N);
    //   Call FFT implementation
    FFTPerform(Output, N, 1);
    //   Scale if necessary
    if (Scale) {
        FFTScale(Output, N);
    }

    //   Succeeded
    return 1;
}

//   INVERSE FOURIER TRANSFORM, INPLACE VERSION
//     Data  - both input data and output
//     N     - length of both input data and result
//     Scale - if to scale result: 1 - scale, 0 - do not scale
unsigned int FFTInverseInplace(TComplex *const Data, const unsigned int N, const unsigned int Scale)
{
    //   Check input parameters
    if (!Data || N < 1 || N & (N - 1)) {
        return 0;
    }

    //   Rearrange
    FFTRearrangeInplace(Data, N);
    //   Call FFT implementation
    FFTPerform(Data, N, 1);
    //   Scale if necessary
    if (Scale) {
        FFTScale(Data, N);
    }

    //   Succeeded
    return 1;
}
