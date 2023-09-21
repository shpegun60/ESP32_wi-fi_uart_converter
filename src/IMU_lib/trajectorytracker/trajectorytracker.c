#include "trajectorytracker.h"
#include "smart_assert.h"
#include "fastmath.h"

#include <stdlib.h>

TrajTrack_t* tralectoryTrackerConstructor(void)
{
    TrajTrack_t* m = (TrajTrack_t*)calloc(1, sizeof(TrajTrack_t));
    M_Assert_BreakSaveCheck((m == NULL), "tralectoryTrackerConstructor: no memories for allocation data", return NULL);

#define coefsFilteringFFT 1U
#define discreteTimesFFT  4096U

    float arr[coefsFilteringFFT] = {0.00f, };
    m->fftFilter = fftFilter_constructor(coefsFilteringFFT, arr, discreteTimesFFT);

#undef coefsFilteringFFT
#undef discreteTimesFFT

    trajectoryTrackerReset(m);
    return m;
}
void trajectoryTrackerReset(TrajTrack_t *self)
{
    // position data --------------------------------------
    for(int i = 0; i < 3; ++i) {
        self->a_dataGlobal[i] = 0.0;
    }

    for(int i = 0; i < 2; ++i) {
        self->accelX[i] = 0.0;
        self->accelY[i] = 0.0;
        self->velocityX[i] = 0.0;
        self->velocityY[i] = 0.0;
        self->posX[i] = 0.0;
        self->posY[i] = 0.0;
    }

    self->countx = 0;
    self->county = 0;

    Quaternion_setIdentity(&self->tmp);
}

static void accelCoordintareToGlobal(TrajTrack_t *self, TrajTrackInput_t *data)
{
    //coordinate Local To Global ---------------------------------------------------------
    Quaternion_conjugate(data->q, &self->tmp);
    Quaternion_rotate(&self->tmp, data->a, self->a_dataGlobal);
}

static void movement_end_check(TrajTrack_t *self)
{
    if (self->accelX[1]==0) //we count the number of acceleration samples that equals cero
    { self->countx++;}
    else { self->countx = 0;}

    if (self->countx >= 30) //if this number exceeds 25, we can assume that velocity is cero
    {
        self->velocityX[1]=0;
        self->velocityX[0]=0;
    }

    if (self->accelY[1]==0) //we do the same for the Y axis
    { self->county++;}
    else { self->county =0;}

    if (self->county>=30)
    {
        self->velocityY[1]=0;
        self->velocityY[0]=0;
    }
}


int proceedTrajectorySimple(TrajTrack_t *self, TrajTrackInput_t *data)
{
    M_Assert_Break((self == NULL), "proceedTrajectorySimple: invalid input", return 0);
    M_Assert_Break((data == NULL || data->a == NULL || data->q == NULL), "proceedTrajectorySimple:  invalid input", return 0);

    //coordinate Local To Global ---------------------------------------------------------
    accelCoordintareToGlobal(self, data);

    if(fastabsf(self->a_dataGlobal[0]) < 0.4) {
        self->accelX[1] = 0.0f;
    } else {
        self->accelX[1] = self->a_dataGlobal[0];
    }

    if(fastabsf(self->a_dataGlobal[1]) < 0.4) {
        self->accelY[1] = 0.0f;
    } else {
        self->accelY[1] = self->a_dataGlobal[1];
    }

    self->velocityX[1]= self->velocityX[0] + self->accelX[0] + (((self->accelX[1] - self->accelX[0]) * 0.5) * data->dt_sec);
    self->velocityY[1]= self->velocityY[0] + self->accelY[0] + (((self->accelY[1] - self->accelY[0]) * 0.5) * data->dt_sec);

    self->posX[1]= self->posX[0] + self->velocityX[0] + (((self->velocityX[1] - self->velocityX[0]) * 0.5) * data->dt_sec);
    self->posY[1]= self->posY[0] + self->velocityY[0] + (((self->velocityY[1] - self->velocityY[0]) * 0.5) * data->dt_sec);

    self->accelX[0] = self->accelX[1]; //The current acceleration value must be sent
    //to the previous acceleration
    self->accelY[0] = self->accelY[1]; //variable in order to introduce the new
    //acceleration value.

    self->velocityX[0] = self->velocityX[1]; //Same done for the velocity variable
    self->velocityY[0] = self->velocityY[1];

    movement_end_check(self);

    self->posX[0] = self->posX[1]; //actual position data must be sent to the
    self->posY[0] = self->posY[1]; //previous position

    return 1;
}


int proceedTrajectoryFFT(TrajTrack_t *self, TrajTrackInput_t *data)
{
    M_Assert_Break((self == NULL), "proceedTrajectoryFFT: invalid input", return 0);
    M_Assert_Break((self->fftFilter == NULL), "proceedTrajectoryFFT: invalid fft pointer", return 0);
    M_Assert_Break((data == NULL || data->a == NULL || data->q == NULL), "proceedTrajectoryFFT:  invalid input", return 0);

    //coordinate Local To Global ---------------------------------------------------------
    accelCoordintareToGlobal(self, data);

    // spectrum----------------------------------------------------------------------------
    self->fftFilter->MeasuredData[self->fftFilter->dataIterator].re = self->a_dataGlobal[0];
    self->fftFilter->MeasuredData[self->fftFilter->dataIterator].im = 0.0f;
    fftFilteringProceed(self->fftFilter);

    if(fastabsf(self->fftFilter->FiltersedData[self->fftFilter->N_points - 1].re) < 0.05) {
        self->accelX[1] = 0.0f;
    } else {
        self->accelX[1] = self->fftFilter->FiltersedData[self->fftFilter->N_points - 1].re;
    }

    self->velocityX[1] = self->velocityX[0] + self->accelX[0] + (((self->accelX[1] - self->accelX[0]) * 0.5) * data->dt_sec);

    self->posX[1] = self->posX[0] + self->velocityX[0] + (((self->velocityX[1] - self->velocityX[0]) * 0.5) * data->dt_sec);

    self->accelX[0] = self->accelX[1]; //The current acceleration value must be sent
    //acceleration value.

    self->velocityX[0] = self->velocityX[1]; //Same done for the velocity variable

    movement_end_check(self);

    self->posX[0] = self->posX[1]; //actual position data must be sent to the
    return 1;
}

























