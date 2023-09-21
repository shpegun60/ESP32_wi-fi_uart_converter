#include "coordinate_ahrs.h"
#include "smart_assert.h"
#include "fastmath.h"

#define ERROR_VALUE 0.00001
/*
 * **************************************************************************************************************************
 * Complementary filter coordinate transition functions (conjugate quaternion OUT)
 * **************************************************************************************************************************
 */

// accelerometer delta quaternion finder for all coordinates --------------------------------------------
void ComplementaryAccDeltaQuaterFinder_NED(AHRS_t* ahrs, AHRSinput * data) // NED gravity vector G = [0, 0, -1]
{
    //-g system CONJUGATE
    register float gx = ahrs->rotateAxisErrAcc[0];
    register float gy = ahrs->rotateAxisErrAcc[1];
    register float gz = ahrs->rotateAxisErrAcc[2];

    // if (ahrs->rotateAxisErrAcc[2] > 0.0f) {
    //     ahrs->q_ae.w    = ((double) ((-gz + 1.0) * ahrs->q_ae.w) + gy * ahrs->q_ae.v[0] - gx * ahrs->q_ae.v[1]) * 0.5;
    //     ahrs->q_ae.v[0] = (gy * (double) ahrs->q_ae.w + (double) (gz + 1.0f) * ahrs->q_ae.v[0] - gx * ahrs->q_ae.v[2]) * 0.5;
    //     ahrs->q_ae.v[1] =  0.0f;
    //     ahrs->q_ae.v[2] = (-gx * ahrs->q_ae.v[0] - gy * ahrs->q_ae.v[1] + (double) (-gz + 1.0f) * ahrs->q_ae.v[2]) * 0.5;
    // } else {
        ahrs->q_ae.w = ((double) ((-gz + 1.0) * ahrs->q_ae.w) + gy * ahrs->q_ae.v[0] - gx * ahrs->q_ae.v[1]) * 0.5;
        ahrs->q_ae.v[0] = (gy * (double) ahrs->q_ae.w + (double) (gz + 1.0) * ahrs->q_ae.v[0] - gx * ahrs->q_ae.v[2]) * 0.5;
        ahrs->q_ae.v[1] = (-gx * (double) ahrs->q_ae.w + (double) (gz + 1.0) * ahrs->q_ae.v[1] - gy * ahrs->q_ae.v[2]) * 0.5;
        ahrs->q_ae.v[2] =  0.0f;
    // }

    ahrs->q_ae.w = ahrs->q_ae.w > 0.0 ? (ahrs->q_ae.w + ERROR_VALUE) : (ahrs->q_ae.w - ERROR_VALUE);
    (void)data;
}

void ComplementaryAccDeltaQuaterFinder_ENU(AHRS_t* ahrs, AHRSinput * data) // ENU gravity vector G = [0, 0,  1]
{
    // you must add this
    (void)ahrs;
    (void)data;
}

// magnetometer delta quaternion finder for all coordinates --------------------------------------------
void ComplementaryMagDeltaQuaterFinder_NED(AHRS_t* ahrs, AHRSinput * data) // NED geomagnetic field vector R = [mN, 0, mD]
{
    register float mn = fastSqrtf_math(ahrs->rotateAxisErrMag[0]*ahrs->rotateAxisErrMag[0] + ahrs->rotateAxisErrMag[1]*ahrs->rotateAxisErrMag[1]);
    register float md = ahrs->rotateAxisErrMag[2];

    register float mx = ahrs->rotateAxisErrMag[0];
    register float my = ahrs->rotateAxisErrMag[1];
    register float mz = ahrs->rotateAxisErrMag[2];

    ahrs->q_me.w    = ((double)((md * mz + mn * mx + 1.0f) * ahrs->q_me.w)  + (double)(mn * my * ahrs->q_me.v[2])) * 0.5f;
    ahrs->q_me.v[0] = 0.0f;
    ahrs->q_me.v[1] = 0.0f;
    ahrs->q_me.v[2] = ((double)(mn * my * ahrs->q_me.w) + (double)((md * mz - mn * mx + 1.0f) * ahrs->q_me.v[2])) * 0.5f;

    ahrs->refAxisMag[0] = mn;
    ahrs->refAxisMag[1] = 0.0f;
    ahrs->refAxisMag[2] = md;
    (void)data;
}

void ComplementaryMagDeltaQuaterFinder_ENU(AHRS_t* ahrs, AHRSinput * data) // ENU geomagnetic field vector R = [0, mN, -mD]
{
    // you must add this
    (void)ahrs;
    (void)data;
}

/*
 * **************************************************************************************************************************
 * Madgwick filter coordinate transition functions (original quaternion OUT)
 * **************************************************************************************************************************
 */

// accelerometer delta quaternion finder for all coordinates --------------------------------------------
void MadgwickAccDeltaQuaterFinder_NED(AHRS_t* ahrs, AHRSinput * data) // NED gravity vector G = [0, 0, -1]
{
    // Auxiliary variables to avoid repeated arithmetic
    float _2q0q0 = 2.0f * ahrs->q_Buf.w    * ahrs->q_Buf.w;
    float _2q1q1 = 2.0f * ahrs->q_Buf.v[0] * ahrs->q_Buf.v[0];
    float _2q2q2 = 2.0f * ahrs->q_Buf.v[1] * ahrs->q_Buf.v[1];
    float _2q3q3 = 2.0f * ahrs->q_Buf.v[2] * ahrs->q_Buf.v[2];
    float _2ax = 2.0f * data->a[0];
    float _2ay = 2.0f * data->a[1];
    float _2az = 2.0f * data->a[2];
    float sumQuat = (2.0f * _2q0q0 + 2.0f * _2q1q1 + 2.0f * _2q2q2 + 2.0f * _2q3q3);

    float q0 = ahrs->q_Buf.w   ;
    float q1 = ahrs->q_Buf.v[0];
    float q2 = ahrs->q_Buf.v[1];
    float q3 = ahrs->q_Buf.v[2];

    // Gradient decent algorithm corrective step
    ahrs->q_ae.w    = (_2az + sumQuat) * q0 + _2ay * q1 - _2ax * q2;
    ahrs->q_ae.v[0] = (-_2az + sumQuat) * q1 + _2ay * q0 + _2ax * q3;
    ahrs->q_ae.v[1] = (-_2az + sumQuat) * q2 - _2ax * q0 + _2ay * q3;
    ahrs->q_ae.v[2] = (_2az + sumQuat) * q3 + _2ax * q1 + _2ay * q2;
}

void MadgwickAccDeltaQuaterFinder_ENU(AHRS_t* ahrs, AHRSinput * data) // ENU gravity vector G = [0, 0,  1]
{
    // Auxiliary variables to avoid repeated arithmetic
    float _2q0q0 = 2.0f * ahrs->q_Buf.w    * ahrs->q_Buf.w;
    float _2q1q1 = 2.0f * ahrs->q_Buf.v[0] * ahrs->q_Buf.v[0];
    float _2q2q2 = 2.0f * ahrs->q_Buf.v[1] * ahrs->q_Buf.v[1];
    float _2q3q3 = 2.0f * ahrs->q_Buf.v[2] * ahrs->q_Buf.v[2];
    float _2ax = 2.0f * data->a[0];
    float _2ay = 2.0f * data->a[1];
    float _2az = 2.0f * data->a[2];
    float sumQuat = (2.0f * _2q0q0 + 2.0f * _2q1q1 + 2.0f * _2q2q2 + 2.0f * _2q3q3);

    float q0 = ahrs->q_Buf.w   ;
    float q1 = ahrs->q_Buf.v[0];
    float q2 = ahrs->q_Buf.v[1];
    float q3 = ahrs->q_Buf.v[2];

    ahrs->q_ae.w    = (-_2az + sumQuat) * q0 - _2ay * q1 + _2ax * q2;
    ahrs->q_ae.v[0] = (_2az + sumQuat) * q1 - _2ay * q0 - _2ax * q3;
    ahrs->q_ae.v[1] = (_2az + sumQuat) * q2 + _2ax * q0 - _2ay * q3;
    ahrs->q_ae.v[2] = (-_2az + sumQuat) * q3 - _2ax * q1 - _2ay * q2;
}

// magnetometer delta quaternion finder for all coordinates --------------------------------------------
void MadgwickAccMagDeltaQuaterFinder_NED(AHRS_t* ahrs, AHRSinput * data) // NED geomagnetic field vector R = [mN, 0, mD], G = [0, 0,  -1]
{
    float _2q0q0 = 2.0f * ahrs->q_Buf.w    * ahrs->q_Buf.w;
    float _2q1q1 = 2.0f * ahrs->q_Buf.v[0] * ahrs->q_Buf.v[0];
    float _2q2q2 = 2.0f * ahrs->q_Buf.v[1] * ahrs->q_Buf.v[1];
    float _2q3q3 = 2.0f * ahrs->q_Buf.v[2] * ahrs->q_Buf.v[2];
    float _2mx = 2.0f * data->m[0];
    float _2my = 2.0f * data->m[1];
    float _2mz = 2.0f * data->m[2];
    float _2ax = 2.0f * data->a[0];
    float _2ay = 2.0f * data->a[1];
    float _2az = 2.0f * data->a[2];
    float sumQuat = (2.0f * _2q0q0 + 2.0f * _2q1q1 + 2.0f * _2q2q2 + 2.0f * _2q3q3);

    float mn = fastSqrtf_math(ahrs->rotateAxisErrMag[0]*ahrs->rotateAxisErrMag[0] + ahrs->rotateAxisErrMag[1]*ahrs->rotateAxisErrMag[1]);
    float md = ahrs->rotateAxisErrMag[2];

    float q0 = ahrs->q_Buf.w   ;
    float q1 = ahrs->q_Buf.v[0];
    float q2 = ahrs->q_Buf.v[1];
    float q3 = ahrs->q_Buf.v[2];

    ahrs->q_me.w    = (md * md * sumQuat + mn * mn * sumQuat - _2mx * mn - _2mz * md + _2az + sumQuat) * q0 + (_2mx * q2 - _2my * q1) * md + (_2my * q3 - _2mz * q2) * mn + q1 * _2ay - q2 * _2ax;
    ahrs->q_me.v[0] = (md * md * sumQuat + mn * mn * sumQuat - _2mx * mn + _2mz * md - _2az + sumQuat) * q1 + (-_2mx * q3 - _2my * q0) * md + (-_2my * q2 - _2mz * q3) * mn + q0 * _2ay + q3 * _2ax;
    ahrs->q_me.v[1] = (md * md * sumQuat + mn * mn * sumQuat + _2mx * mn + _2mz * md - _2az + sumQuat) * q2 + (_2mx * q0 - _2my * q3) * md + (-_2my * q1 - _2mz * q0) * mn - q0 * _2ax + q3 * _2ay;
    ahrs->q_me.v[2] = (md * md * sumQuat + mn * mn * sumQuat + _2mx * mn - _2mz * md + _2az + sumQuat) * q3 + (-_2mx * q1 - _2my * q2) * md + (_2my * q0 - _2mz * q1) * mn + q1 * _2ax + q2 * _2ay;
}

void MadgwickAccMagDeltaQuaterFinder_ENU(AHRS_t* ahrs, AHRSinput * data) // ENU geomagnetic field vector R = [0, mN, -mD], G = [0, 0,  1]
{
    float _2q0q0 = 2.0f * ahrs->q_Buf.w    * ahrs->q_Buf.w;
    float _2q1q1 = 2.0f * ahrs->q_Buf.v[0] * ahrs->q_Buf.v[0];
    float _2q2q2 = 2.0f * ahrs->q_Buf.v[1] * ahrs->q_Buf.v[1];
    float _2q3q3 = 2.0f * ahrs->q_Buf.v[2] * ahrs->q_Buf.v[2];
    float _2mx = 2.0f * data->m[0];
    float _2my = 2.0f * data->m[1];
    float _2mz = 2.0f * data->m[2];
    float _2ax = 2.0f * data->a[0];
    float _2ay = 2.0f * data->a[1];
    float _2az = 2.0f * data->a[2];
    float sumQuat = (2.0f * _2q0q0 + 2.0f * _2q1q1 + 2.0f * _2q2q2 + 2.0f * _2q3q3);

    float mn = fastSqrtf_math(ahrs->rotateAxisErrMag[0]*ahrs->rotateAxisErrMag[0] + ahrs->rotateAxisErrMag[1]*ahrs->rotateAxisErrMag[1]);
    float md = ahrs->rotateAxisErrMag[2];

    float q0 = ahrs->q_Buf.w   ;
    float q1 = ahrs->q_Buf.v[0];
    float q2 = ahrs->q_Buf.v[1];
    float q3 = ahrs->q_Buf.v[2];

    ahrs->q_me.w    = (md * md * sumQuat + mn * mn * sumQuat - _2my * mn + _2mz * md - _2az + sumQuat) * q0 + (-_2mx * q2 + _2my * q1) * md + (-_2mx * q3 + _2mz * q1) * mn - q1 * _2ay + q2 * _2ax;
    ahrs->q_me.v[0] = (md * md * sumQuat + mn * mn * sumQuat + _2my * mn - _2mz * md + _2az + sumQuat) * q1 + (_2mx * q3 + _2my * q0) * md + (-_2mx * q2 + _2mz * q0) * mn - q0 * _2ay - q3 * _2ax;
    ahrs->q_me.v[1] = (md * md * sumQuat + mn * mn * sumQuat - _2my * mn - _2mz * md + _2az + sumQuat) * q2 + (-_2mx * q0 + _2my * q3) * md + (-_2mx * q1 - _2mz * q3) * mn + q0 * _2ax - q3 * _2ay;
    ahrs->q_me.v[2] = (md * md * sumQuat + mn * mn * sumQuat + _2my * mn + _2mz * md - _2az + sumQuat) * q3 + (_2mx * q1 + _2my * q2) * md + (-_2mx * q0 - _2mz * q2) * mn - q1 * _2ax - q2 * _2ay;
}








































































































