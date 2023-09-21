#include "updater_ahrs.h"
#include "quaternion.h"
#include "smart_assert.h"
#include "fastmath.h"

#include <math.h>

//---------------------------------------------------------------------------------------------------------------------------------------
float accelCalcPitchAHRS(float x, float y, float z)
{
    return atan2(y, fastSqrtf_math(x * x + z * z));
}

float accelCalcRollAHRS(float x, float y, float z)
{
    return -atan2(x, fastSqrtf_math(y * y + z * z));
}
//-----------------------------------------------------------------------------------------------------------------------------------------


void computeGravVectorAHRS(const float q0, const float q1, const float q2, const float q3, float* const grav, const float* const constant)
{
    M_Assert_Break((grav == NULL || constant == NULL), "computeGravVector: grav vector is null pointer", return);
// original quaternion
//    grav[0] = 2.0f * ((q1 * q3) - (q0 * q2)) * constant[0];
//    grav[1] = 2.0f * ((q0 * q1) + (q2 * q3)) * constant[1];
//    grav[2] = ((q0 * q0) - (q1 * q1) - (q2 *q2) + (q3 * q3)) * constant[2];

    // conjugate quaternion
    grav[0] = 2.0f * ((q0 * q2) + (q1 * q3)) * constant[0];
    grav[1] = 2.0f * ((q2 * q3) - (q0 * q1)) * constant[1];
    grav[2] = ((q0 * q0) - (q1 * q1) - (q2 * q2) + (q3 * q3)) * constant[2];
}

void magOffsetDeleteAHRS(float* iBpx, float* iBpy, float* iBpz, float iV[3], float InvW[3][3])
{
    M_Assert_Break((iBpx == NULL || iBpy==NULL || iBpz == NULL || InvW == NULL || *InvW == NULL), "magOffsetDelete: incorrect input values", return);

    /* 32 bit scratch */
    float iSumx, iSumy, iSumz;
    /* subtract the hard iron offset */
    (*iBpx) -= iV[0];
    (*iBpy) -= iV[1];
    (*iBpz) -= iV[2];
    /* multiply by the inverse soft iron offset */
    iSumx = (InvW[0][0] * (*iBpx)) + (InvW[0][1] * (*iBpy)) + (InvW[0][2] * (*iBpz));
    iSumy = (InvW[1][0] * (*iBpx)) + (InvW[1][1] * (*iBpy)) + (InvW[1][2] * (*iBpz));
    iSumz = (InvW[2][0] * (*iBpx)) + (InvW[2][1] * (*iBpy)) + (InvW[2][2] * (*iBpz));
    /* return the resulting vector invW*(Bp-V) */
    (*iBpx) = iSumx;
    (*iBpy) = iSumy;
    (*iBpz) = iSumz;
}



void updateSysAHRS(float * g, float dt, float** cg, float** cg0) // cg ==> F, cg0 ==> F_T, cg1 ==> G
{
    M_Assert_Break((g == NULL || cg==NULL || cg0 == NULL), "getResultRawSize: incorrect input values", return);
    register float halfT = 0.5f * dt;
    register float Tx = halfT * g[0];
    register float Ty = halfT * g[1];
    register float Tz = halfT * g[2];

    // F ----------------------------------------------------------------------------
    cg[0][0] = 1;
    cg[0][1] = Tx;
    cg[0][2] = Ty;
    cg[0][3] = Tz;
    cg[1][0] = -Tx;
    cg[1][1] = 1;
    cg[1][2] = Tz;
    cg[1][3] = -Ty;
    cg[2][0] = -Ty;
    cg[2][1] = -Tz;
    cg[2][2] = 1;
    cg[2][3] = Tx;
    cg[3][0] = -Tz;
    cg[3][1] = Ty;
    cg[3][2] = -Tx;
    cg[3][3] = 1;

    // F_T ----------------------------------------------------------------------------
    cg0[0][0] = 1;
    cg0[0][1] = -Tx;
    cg0[0][2] = -Ty;
    cg0[0][3] = -Tz;
    cg0[1][0] = Tx;
    cg0[1][1] = 1;
    cg0[1][2] = -Tz;
    cg0[1][3] = Ty;
    cg0[2][0] = Ty;
    cg0[2][1] = Tz;
    cg0[2][2] = 1;
    cg0[2][3] = -Tx;
    cg0[3][0] = Tz;
    cg0[3][1] = -Ty;
    cg0[3][2] = Tx;
    cg0[3][3] = 1;
}

void updatePredictionNoiseAHRS(float gx, float gy, float gz, float q0, float q1, float q2,float q3, float dt, float** cg) // cg ==> Q
{
    M_Assert_Break((cg == NULL), "updateSysNoise: incorrect input value", return);

    register float ddT = dt * dt;
    register float dT4 = 0.25f * ddT;

    cg[0][0] = (q1 * q1 * gx + q2 * q2 * gy + q3 * q3 * gz) * dT4;
    cg[0][1] = -dT4 * (q3 * (gy - gz) * q2 + q1 * gx * q0);
    cg[0][2] = (q3 * (gx - gz) * q1 - q2 * gy * q0) * dT4;
    cg[0][3] = -dT4 * (q2 * (gx - gy) * q1 + q3 * gz * q0);
    cg[1][0] = -dT4 * (q3 * (gy - gz) * q2 + q1 * gx * q0);
    cg[1][1] = (q0 * q0 * gx + q3 * q3 * gy + q2 * q2 * gz) * dT4;
    cg[1][2] = -dT4 * (q3 * (gx - gy) * q0 + q2 * gz * q1);
    cg[1][3] = dT4 * (q2 * (gx - gz) * q0 - q3 * gy * q1);
    cg[2][0] = (q3 * (gx - gz) * q1 - q2 * gy * q0) * dT4;
    cg[2][1] = -dT4 * (q3 * (gx - gy) * q0 + q2 * gz * q1);
    cg[2][2] = (q3 * q3 * gx + q0 * q0 * gy + q1 * q1 * gz) * dT4;
    cg[2][3] = -dT4 * (q1 * (gy - gz) * q0 + q3 * gx * q2);
    cg[3][0] = -dT4 * (q2 * (gx - gy) * q1 + q3 * gz * q0);
    cg[3][1] = dT4 * (q2 * (gx - gz) * q0 - q3 * gy * q1);
    cg[3][2] = -dT4 * (q1 * (gy - gz) * q0 + q3 * gx * q2);
    cg[3][3] = (q2 * q2 * gx + q1 * q1 * gy + q0 * q0 * gz) * dT4;
}

void updateMeasurmentJacobianMagAHRS_NED(float mag[3], float magRef[3], float acc[3], float q0, float q1, float q2,float q3, float** cg, float** cgt) // cg ==> J, cgt ==> J_T
{
    M_Assert_Break((cg == NULL || mag == NULL || acc == NULL || magRef == NULL), "updateMeasurmentJacobianMag: incorrect input value", return);

    float mx = mag[0];
    float my = mag[1];
    float mz = mag[2];

    float mn = magRef[0];
    float md = magRef[2];

    float ax = acc[0];
    float ay = acc[1];
    float az = acc[2];

    if(az > 0.0f) {
        cgt[0][0] = cg[0][0] = (mz * q3 * md - mx * q3 * mn + mn * my * q0 + q3) * q1;
        cgt[1][0] = cg[0][1] = q1 * (md * mz * q0 + mn * mx * q0 + mn * my * q3 + q0);
        cgt[2][0] = cg[0][2] = mz * (-2 * q0 * q0 - q1 * q1 - q2 * q2 + 1) * md + mx * (q1 * q1 + q2 * q2 - 1) * mn - 2 * q0 * q0 - q1 * q1 - q2 * q2 + 1;
        cgt[3][0] = cg[0][3] = -(ax * q1 * q3 - ay * q0 * q1 - az * q1 * q1 - az * q2 * q2 + q1 * q1 + q2 * q2 + az - 1) * mn;
        cgt[4][0] = cg[0][4] = (ax * q0 + ay * q3) * mn * q1;
        cgt[5][0] = cg[0][5] = ((-2 * q0 * q0 - q1 * q1 - q2 * q2 + 1) * az + q1 * q1 + (ax * q3 + ay * q0) * q1 + 2 * q0 * q0 + q2 * q2 - 1) * md;
        cgt[0][1] = cg[1][0] = ((q0 * q0 + q1 * q1 + q2 * q2 - 1) * my - mx * q0 * q3) * mn - (mz * md + 1) * q3 * q0;
        cgt[1][1] = cg[1][1] = q0 * (md * mz * q0 + mn * mx * q0 + mn * my * q3 + q0);
        cgt[2][1] = cg[1][2] = q1 * (md * mz * q0 + mn * mx * q0 + mn * my * q3 + q0);
        cgt[3][1] = cg[1][3] = -(-ay * q0 + (-az - 1) * q1 + ax * q3) * mn * q0;
        cgt[4][1] = cg[1][4] = mn * ((q0 * q0 + q1 * q1 + q2 * q2 - 1) * ax + ((az + 1) * q1 + ay * q0) * q3);
        cgt[5][1] = cg[1][5] = -(-ay * q0 + (-az - 1) * q1 + ax * q3) * md * q0;
        cgt[0][2] = cg[2][0] = ((q0 * q0 + q1 * q1 + q2 * q2 - 1) * mx + q3 * my * q0) * mn - (q0 * q0 + q1 * q1 + q2 * q2 - 1) * (mz * md + 1);
        cgt[1][2] = cg[2][1] = -(mz * q3 * md - mx * q3 * mn + mn * my * q0 + q3) * q0;
        cgt[2][2] = cg[2][2] = -(mz * q3 * md - mx * q3 * mn + mn * my * q0 + q3) * q1;
        cgt[3][2] = cg[2][3] = mn * ((q0 * q0 + q1 * q1 + q2 * q2 - 1) * ax + ((az + 1) * q1 + ay * q0) * q3);
        cgt[4][2] = cg[2][4] = (-ay * q0 + (-az - 1) * q1 + ax * q3) * mn * q0;
        cgt[5][2] = cg[2][5] = -md * ((q0 * q0 + q1 * q1 + q2 * q2 - 1) * ax + ((az + 1) * q1 + ay * q0) * q3);
        cgt[0][3] = cg[3][0] = -q1 * (md * mz * q0 + mn * mx * q0 + mn * my * q3 + q0);
        cgt[1][3] = cg[3][1] = (mz * q3 * md - mx * q3 * mn + mn * my * q0 + q3) * q1;
        cgt[2][3] = cg[3][2] = my * mn * (q1 * q1 + q2 * q2 - 1) - 2 * (mz * md + 1) * q3 * q0;
        cgt[3][3] = cg[3][3] = -(ax * q0 + ay * q3) * mn * q1;
        cgt[4][3] = cg[3][4] = -(ax * q1 * q3 - ay * q0 * q1 - az * q1 * q1 - az * q2 * q2 + q1 * q1 + q2 * q2 + az - 1) * mn;
        cgt[5][3] = cg[3][5] = -md * (((2 * az - 2) * q3 + ax * q1) * q0 - ay * q1 * q3);
    } else {
        cgt[0][0] = cg[0][0] = -(double) q2 * (double) (md * mz * q0 + mn * mx * q0 + mn * my * q3 + q0);
        cgt[1][0] = cg[0][1] = (double) q1 * (double) (md * mz * q0 + mn * mx * q0 + mn * my * q3 + q0);
        cgt[2][0] = cg[0][2] = -q0 * (md * mz * q0 + mn * mx * q0 + mn * my * q3 + q0);
        cgt[3][0] = cg[0][3] = -((az - 1) * q0 + ax * q2 - ay * q1) * mn * q0;
        cgt[4][0] = cg[0][4] = -((az - 1) * q0 + ax * q2 - ay * q1) * mn * q3;
        cgt[5][0] = cg[0][5] = -((az - 1) * q0 + ax * q2 - ay * q1) * md * q0;
        cgt[0][1] = cg[1][0] = -(mz * q3 * md - mx * q3 * mn + mn * my * q0 + q3) * q0;
        cgt[1][1] = cg[1][1] = q0 * (md * mz * q0 + mn * mx * q0 + mn * my * q3 + q0);
        cgt[2][1] = cg[1][2] = ((mx * q1 + my * q2) * q0 - q3 * (mx * q2 - my * q1)) * mn + (q0 * q1 + q2 * q3) * (mz * md + 1);
        cgt[3][1] = cg[1][3] = mn * (ay * q0 * q0 + (ax * q3 + (az + 1) * q1) * q0 - q2 * (az + 1) * q3);
        cgt[4][1] = cg[1][4] = -mn * (ax * q0 * q0 + (-ay * q3 - (az + 1) * q2) * q0 - q1 * q3 * (az + 1));
        cgt[5][1] = cg[1][5] = -(-ay * q0 * q0 + (ax * q3 - (az + 1) * q1) * q0 - q2 * (az + 1) * q3) * md;
        cgt[0][2] = cg[2][0] = -q0 * (md * mz * q0 + mn * mx * q0 + mn * my * q3 + q0);
        cgt[1][2] = cg[2][1] = -(mz * q3 * md - mx * q3 * mn + mn * my * q0 + q3) * q0;
        cgt[2][2] = cg[2][2] = ((mx * q2 - my * q1) * q0 + q3 * (mx * q1 + my * q2)) * mn + (q0 * q2 - q1 * q3) * (mz * md + 1);
        cgt[3][2] = cg[2][3] = -mn * (ax * q0 * q0 + (-ay * q3 - (az + 1) * q2) * q0 - q1 * q3 * (az + 1));
        cgt[4][2] = cg[2][4] = -mn * (ay * q0 * q0 + (ax * q3 + (az + 1) * q1) * q0 - q2 * (az + 1) * q3);
        cgt[5][2] = cg[2][5] = -md * (ax * q0 * q0 + (ay * q3 - (az + 1) * q2) * q0 + q1 * q3 * (az + 1));
        cgt[0][3] = cg[3][0] = -(mz * q3 * md - mx * q3 * mn + mn * my * q0 + q3) * q2;
        cgt[1][3] = cg[3][1] = (mz * q3 * md - mx * q3 * mn + mn * my * q0 + q3) * q1;
        cgt[2][3] = cg[3][2] = -(mz * q3 * md - mx * q3 * mn + mn * my * q0 + q3) * q0;
        cgt[3][3] = cg[3][3] = ((az - 1) * q0 + ax * q2 - ay * q1) * mn * q3;
        cgt[4][3] = cg[3][4] = -((az - 1) * q0 + ax * q2 - ay * q1) * mn * q0;
        cgt[5][3] = cg[3][5] = -md * q3 * ((az - 1) * q0 + ax * q2 - ay * q1);
    }
}


void updateMeasurmentJacobianAHRS_NED(float acc[3], float q0, float q1, float q2,float q3, float** cg, float** cgt) // cg ==> J, cgt ==> J_T
{
    M_Assert_Break((cg == NULL  || acc == NULL ), "updateMeasurmentJacobianAHRS: incorrect input value", return);

    (void)acc;

    cgt[0][0] = cg[0][0] = -q2;
    cgt[1][0] = cg[0][1] = q1;
    cgt[2][0] = cg[0][2] = -q0;
    cgt[3][0] = cg[0][3] = 0;
    cgt[4][0] = cg[0][4] = 0;
    cgt[5][0] = cg[0][5] = 0;
    cgt[0][1] = cg[1][0] = -q3;
    cgt[1][1] = cg[1][1] = q0;
    cgt[2][1] = cg[1][2] = q1;
    cgt[3][1] = cg[1][3] = 0;
    cgt[4][1] = cg[1][4] = 0;
    cgt[5][1] = cg[1][5] = 0;
    cgt[0][2] = cg[2][0] = -q0;
    cgt[1][2] = cg[2][1] = -q3;
    cgt[2][2] = cg[2][2] = q2;
    cgt[3][2] = cg[2][3] = 0;
    cgt[4][2] = cg[2][4] = 0;
    cgt[5][2] = cg[2][5] = 0;
    cgt[0][3] = cg[3][0] = -q1;
    cgt[1][3] = cg[3][1] = -q2;
    cgt[2][3] = cg[3][2] = -q3;
    cgt[3][3] = cg[3][3] = 0;
    cgt[4][3] = cg[3][4] = 0;
    cgt[5][3] = cg[3][5] = 0;
}


void updateMeasurmentJacobianMagAHRS_ENU(float mag[3], float magRef[3], float acc[3], float q0, float q1, float q2,float q3, float** cg, float** cgt) // cg ==> J, cgt ==> J_T
{
    // no released
}
void updateMeasurmentJacobianAHRS_ENU(float acc[3], float q0, float q1, float q2,float q3, float** cg, float** cgt) // cg ==> J, cgt ==> J_T
{
    // no released
}










































































































