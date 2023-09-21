/**
 * @file    __TRAJECTORY_TRACKER.h
 * @brief
 * @date
 */

#ifndef __TRAJECTORY_TRACKER_H_
#define __TRAJECTORY_TRACKER_H_

#include "fft.h"
#include "fft_filter.h"
#include "quaternion.h"


typedef struct TrajTrackInput
{
    float dt_sec;
    float *a; // ==> x,y,z
    Quaternion *q;
} TrajTrackInput_t;

typedef struct TrajTrack {
    // position data --------------------------------------
    float a_dataGlobal[3];

    float accelX[2];
    float accelY[2];

    float velocityX[2];
    float velocityY[2];

    float posX[2];
    float posY[2];

    unsigned int countx;
    unsigned int county;


    FFTFilter_t* fftFilter;
    Quaternion tmp;
} TrajTrack_t;

typedef int (*TrajTrack_polymorph_t) (TrajTrack_t *, TrajTrackInput_t *);

TrajTrack_t* tralectoryTrackerConstructor(void);
void trajectoryTrackerReset(TrajTrack_t *self);

// polymorph function`s
int proceedTrajectorySimple(TrajTrack_t *self, TrajTrackInput_t *data);
int proceedTrajectoryFFT(TrajTrack_t *self, TrajTrackInput_t *data);


#endif /* __TRAJECTORY_TRACKER_H_ */


