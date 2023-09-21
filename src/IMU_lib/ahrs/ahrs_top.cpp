#include "ahrs_top.h"
#include "smart_assert.h"

AHRS_Worker::AHRS_Worker(AHRSInit* init, AHRS_polymorph_t proceedFoo)
{
    M_Assert_BreakSaveCheck((proceedFoo == (AHRS_polymorph_t)(NULL)), "AHRS_Worker constructor: proceedFoo is not exists", return);
    M_Assert_BreakSaveCheck((init == NULL), "AHRS_Worker constructor: init is not exists", return);
    ahrs = ahrsCreate(init);
    ahrsFusion = proceedFoo;
}

AHRS_Worker::AHRS_Worker()
{

}

int AHRS_Worker::AHRS_Reinit(AHRSInit *init, AHRS_polymorph_t proceedFoo)
{
    M_Assert_BreakSaveCheck((proceedFoo == (AHRS_polymorph_t)(NULL)), "AHRS_Reinit: proceedFoo is not exists", return 0);
    M_Assert_BreakSaveCheck((init == NULL), "AHRS_Reinit: init is not exists", return 0);
    
    if(ahrs == NULL) {
        ahrs = ahrsCreate(init);
    }
    ahrsFusion = proceedFoo;
    return 1;
}

void AHRS_Worker::AHRS_reset()
{
    ahrsReset(ahrs);
}


// function for update --------------------------------------------------------------------------------------------------------------------------
int AHRS_Worker::AhrsProceed(AHRSinput *data)
{
    M_Assert_Break((data == NULL), "AhrsProceed: m is not exists", return 0);
    M_Assert_Break((ahrsFusion == (AHRS_polymorph_t)(NULL)), "AhrsProceed: proceedFoo is not exists", return 0);
    return ahrsFusion(ahrs, data);
}

int AHRS_Worker::AhrsProceed(AHRSinput data)
{
    M_Assert_Break((ahrsFusion == (AHRS_polymorph_t)(NULL)), "AhrsProceed: proceedFoo is not exists", return 0);
    return ahrsFusion(ahrs, &data);
}

int AHRS_Worker::AhrsProceed(float dt, float a[3], float g[3], float m[3])
{
    M_Assert_Break((a == NULL || g == NULL || m == NULL), "AhrsProceed: data is not exists", return 0);
    M_Assert_Break((ahrsFusion == (AHRS_polymorph_t)(NULL)), "AhrsProceed: proceedFoo is not exists", return 0);

    AHRSinput data;
    data.dt_sec = dt;
    for(int i = 0; i < 3; ++i) {
        data.a[i] = a[i];
        data.g[i] = g[i];
        data.m[i] = m[i];
    }
    return ahrsFusion(ahrs, &data);
}


int AHRS_Worker::AhrsProceed(float dt, float ax, float ay, float az, float gx, float gy, float gz, float mx, float my, float mz)
{
    M_Assert_Break((ahrsFusion == (AHRS_polymorph_t)(NULL)), "AhrsProceed: proceedFoo is not exists", return 0);
    AHRSinput data;

    data.dt_sec = dt;
    data.a[0] = ax;
    data.a[1] = ay;
    data.a[2] = az;

    data.g[0] = gx;
    data.g[1] = gy;
    data.g[2] = gz;

    data.m[0] = mx;
    data.m[1] = my;
    data.m[2] = mz;

    return ahrsFusion(ahrs, &data);
}

//function for starting calculate quaternion ---------------------------------------------------------------------------------------------------------------------------------------------
int AHRS_Worker::AhrsInitQuat(AHRSinput *data, int meanIterations, int iterationsStart)
{
    M_Assert_Break((data == NULL), "AhrsInitQuat: data is not exists", return 0);
    M_Assert_Break((ahrsFusion == (AHRS_polymorph_t)(NULL)), "AhrsInitQuat: proceedFoo is not exists", return 0);
    return ahrsInitStartQuat(ahrs, data, meanIterations, iterationsStart, ahrsFusion);
}

int AHRS_Worker::AhrsInitQuat(AHRSinput data, int meanIterations, int iterationsStart)
{
    M_Assert_Break((ahrsFusion == (AHRS_polymorph_t)(NULL)), "AhrsInitQuat: proceedFoo is not exists", return 0);
    return ahrsInitStartQuat(ahrs, &data, meanIterations, iterationsStart, ahrsFusion);
}


int AHRS_Worker::AhrsInitQuat(float dt, float a[3], float g[3], float m[3], int meanIterations, int iterationsStart)
{
    M_Assert_Break((ahrsFusion == (AHRS_polymorph_t)(NULL)), "AhrsInitQuat: proceedFoo is not exists", return 0);
    AHRSinput data;
    data.dt_sec = dt;
    for(int i = 0; i < 3; ++i) {
        data.a[i] = a[i];
        data.g[i] = g[i];
        data.m[i] = m[i];
    }

    return ahrsInitStartQuat(ahrs, &data, meanIterations, iterationsStart, ahrsFusion);
}

int AHRS_Worker::AhrsInitQuat(float dt, float ax, float ay, float az, float gx, float gy, float gz, float mx, float my, float mz, int meanIterations, int iterationsStart)
{
    M_Assert_Break((ahrsFusion == (AHRS_polymorph_t)(NULL)), "AhrsInitQuat: proceedFoo is not exists", return 0);
    AHRSinput data;

    data.dt_sec = dt;
    data.a[0] = ax;
    data.a[1] = ay;
    data.a[2] = az;

    data.g[0] = gx;
    data.g[1] = gy;
    data.g[2] = gz;

    data.m[0] = mx;
    data.m[1] = my;
    data.m[2] = mz;

    return ahrsInitStartQuat(ahrs, &data, meanIterations, iterationsStart, ahrsFusion);
}





//------------------------------------------------------------------------------------------------------------------------------------------------------------
Mat *AHRS_Worker::getResultData()
{
    return ahrs->kalman->X_est;
}

Quaternion AHRS_Worker::getQuaternion()
{
    return ahrs->RES;
}

Quaternion* AHRS_Worker::getQuaternion_ptr()
{
    return &ahrs->RES;
}

float *AHRS_Worker::getGravity()
{
    return ahrs->grav;
}

Mat * AHRS_Worker::getLinearAcceleration()
{
    return ahrs->kalman->U;
}

Mat * AHRS_Worker::getSystem()
{
    return ahrs->kalman->X_pred;
}

void AHRS_Worker::setPolymorphAHRS(AHRS_polymorph_t proceedFoo)
{
    M_Assert_BreakSaveCheck((proceedFoo == (AHRS_polymorph_t)(NULL)), "setPolymorph: proceedFoo is not exists", return);
    ahrsFusion = proceedFoo;
}


void AHRS_Worker::printKalmanTop()
{
    printkalman(ahrs->kalman);
}

