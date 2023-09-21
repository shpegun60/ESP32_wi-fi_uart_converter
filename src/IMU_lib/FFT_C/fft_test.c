//   LIBROW FFT sample
//   Demonstrates how to use the FFT
//
//   The code is property of LIBROW
//   You can use it on your own
//   When utilizing credit LIBROW site

//   Include FFT header
#include "fft.h"
#include "fft_test.h"

//   Output
#include <stdio.h>

//   Application entry point
void fft_test(void)
{

    /*
     *  re: 28.000000 -4.000000 -4.000000 -4.000000 -4.000000 -4.000000 -4.000000 -4.000000
     *  im: 0.000000 9.656854 4.000000 1.656854 0.000000 -1.656854 -4.000000 -9.656854
     */


	//   Output counter
	unsigned int i;

	//   Input signal {0, 1, 2, 3, 4, 5, 6, 7}
    TComplex Signal[8] = {{0., 0.}, {1., 0.}, {2., 0.}, {3., 0.}, {4., 0.}, {5., 0.}, {6., 0.}, {7., 0}};
    TComplex Signal1[8] = {{7., 0}, {0., 0.}, {1., 0.}, {2., 0.}, {3., 0.}, {4., 0.}, {5., 0.}, {6., 0.}};
    TComplex ToSignal[8];

	//   Process the signal with FFT
    FFTForwardInplace(Signal, 8);
    //FFTInverseInplace(Signal, 8, 1); //inverse fft
    FFTForward_fromPos(Signal1, ToSignal, 8, 1);

	//   Output the result
	//   Real part
	printf("re: ");
    for (i = 0; i < 8; ++i) {
		printf("%f ", Signal[i].re);
    }

	//   Imaginary part
	printf("\nim: ");
    for (i = 0; i < 8; ++i) {
        printf("%f ", Signal[i].im);
    }

	printf("\n\n");

    printf("re: ");
    for (i = 0; i < 8; ++i) {
        printf("%f ", ToSignal[i].re);
    }

    //   Imaginary part
    printf("\nim: ");
    for (i = 0; i < 8; ++i) {
        printf("%f ", ToSignal[i].im);
    }

    printf("\n\n");


	//   Done
    fflush(stdout);
}
