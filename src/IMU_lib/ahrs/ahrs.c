#include "ahrs.h"
#include "smart_assert.h"
#include "updater_ahrs.h"
#include "coordinate_ahrs.h"
#include "fastmath.h"

#include <stdlib.h>
#include <math.h>


#define EPSILON_QUATER 0.9f
static_assert (__builtin_types_compatible_p(MAT_TYPE, float), "ahrs: MATH_TYPE must be float or rewrite /void Quaternion_multiply_to_arrayLN(Quaternion* q1, Quaternion* q2, float** output)/ in quaternion lib to /void Quaternion_multiply_to_arrayLN(Quaternion* q1, Quaternion* q2, USER_TYPE** output)/ and rewrite this assert to USER_TYPE");

#define KALx 4  // x_k system state
#define KALz 4   // z_k measurment

#define QuatSize 4      // quaternion size is 4
#define MagGyroSize 6   // magnetometer + gyroscope variance size is 6

AHRS_t* ahrsCreate(AHRSInit* init)
{
    M_Assert_BreakSaveCheck((init == NULL), "ahrsCreate:INIT is nullptr", return NULL);
    //M_Assert_BreakSaveCheck((init->accConst_u < 0.0f || init->accConst_u > 1.0f), "ahrsCreate: value of u out of range", return NULL);
    AHRS_t* ahrs = (AHRS_t*)malloc(sizeof(AHRS_t));
    M_Assert_BreakSaveCheck((ahrs == NULL), "ahrsCreate: no memory for allocation structure", return NULL);

    if(init->Type == Kalman) {
        ahrs->kalman = kalmanCreate(NULL, KALx, KALz, 0, 1);

        // R update matrix--------------------------------------------------------------------------
        ahrs->J = matrixCreate(QuatSize, MagGyroSize);
        ahrs->J_t = createResultTransMatrix(ahrs->J);
        ahrs->Noise_measurment = matrixCreate(MagGyroSize, MagGyroSize);
        ahrs->NOISE_R_RES = createResultMulMatrix(ahrs->Noise_measurment, ahrs->J_t);
    } else {
        ahrs->kalman = NULL;
        // R
        ahrs->J = NULL;
        ahrs->J_t = NULL;
        ahrs->Noise_measurment = NULL;
        ahrs->NOISE_R_RES = NULL;
    }


    /*
     * ******************************************************************************************
     * ahrs user init functions
     *******************************************************************************************
     *
    */

    ahrs->init = init;
    if(init->coordinateType == NED) {
        ahrs->accDeltaQuaterFinder[AHRS_COMPLEMENTARY_FILTER] = ComplementaryAccDeltaQuaterFinder_NED;
        ahrs->magDeltaQuaterFinder[AHRS_COMPLEMENTARY_FILTER] = ComplementaryMagDeltaQuaterFinder_NED;

        ahrs->accDeltaQuaterFinder[AHRS_MADGWICK_FILTER] = MadgwickAccDeltaQuaterFinder_NED;
        ahrs->magDeltaQuaterFinder[AHRS_MADGWICK_FILTER] = MadgwickAccMagDeltaQuaterFinder_NED;
    } else {
        ahrs->accDeltaQuaterFinder[AHRS_COMPLEMENTARY_FILTER] = ComplementaryAccDeltaQuaterFinder_ENU;
        ahrs->magDeltaQuaterFinder[AHRS_COMPLEMENTARY_FILTER] = ComplementaryMagDeltaQuaterFinder_ENU;

        ahrs->accDeltaQuaterFinder[AHRS_MADGWICK_FILTER] = MadgwickAccDeltaQuaterFinder_ENU;
        ahrs->magDeltaQuaterFinder[AHRS_MADGWICK_FILTER] = MadgwickAccMagDeltaQuaterFinder_ENU;
    }

    for(unsigned i = 0; i < 3; ++i) {
        if(init->coordinateType == NED) {
            init->gravityConstVect[i] = -init->gravityConstVect[i];
        }
    }

    ahrsReset(ahrs);
    return ahrs;
}


void ahrsReset(AHRS_t* ahrs)
{
    if(ahrs->init->Type == Kalman) {
        /*
        * ******************************************************************************************
        * KALMAN MATRIX user init functions
        * ******************************************************************************************
        */

        // F, F_t init ------------------------------------------------------------------------
        float F_init [KALx][KALx] =
        {
            {1,  0,  0,  0},
            {0,  1,  0,  0},
            {0,  0,  1,  0},
            {0,  0,  0,  1}
        };
        matrixInitFromArr(ahrs->kalman->F, F_init[0]);
        matrixInitFromArr_T(ahrs->kalman->F_t, F_init[0]);

        // X_est init ------------------------------------------------------------------------
        float X_est_init[KALx][1] =
        {
            {1},
            {0},
            {0},
            {0}
        };
        matrixInitFromArr(ahrs->kalman->X_est, X_est_init[0]);

        // P_est init ------------------------------------------------------------------------
        float P_init[KALx][KALx] =
        {
            {500,  0,  0,  0},
            {0,  500,  0,  0},
            {0,  0,  500,  0},
            {0,  0,  0,  500}
        };
        matrixInitFromArr(ahrs->kalman->P_est, P_init[0]);

        // Q init ------------------------------------------------------------------------
        float sQR = 1e-6;

        float Q_init[KALx][KALx] =
        {
            {sQR,  0,  0,  0},
            {0,  sQR,  0,  0},
            {0,  0,  sQR,  0},
            {0,  0,  0,  sQR}
        };
        matrixInitFromArr(ahrs->kalman->Q, Q_init[0]);

        // R init ------------------------------------------------------------------------
        float sR = 0.000015f;
        float R_init[KALz][KALz] =
        {
            {sR, 0,  0,  0},
            {0, sR,  0,  0},
            {0, 0,  sR,  0},
            {0, 0,  0,  sR}
        };
        matrixInitFromArr(ahrs->kalman->R, R_init[0]);

        // R update matrix--------------------------------------------------------------------------
        float smx = ahrs->init->magVarianceVect[0];
        float smy = ahrs->init->magVarianceVect[1];
        float smz = ahrs->init->magVarianceVect[2];
        float sax = ahrs->init->accVarianceVect[0];
        float say = ahrs->init->accVarianceVect[1];
        float saz = ahrs->init->accVarianceVect[2];

        float noiseMeasureInit[MagGyroSize][MagGyroSize] =
        {
            {sax, 0,   0,    0,   0,  0 },
            {0,  say,  0,    0,   0,  0 },
            {0,  0,   saz,   0,   0,  0 },
            {0,  0,   0,    smx,  0,  0 },
            {0,  0,   0,    0,   smy, 0 },
            {0,  0,   0,    0,   0,  smz}
        };
        matrixInitFromArr(ahrs->Noise_measurment, noiseMeasureInit[0]);
    }

    for(unsigned i = 0; i < 3; ++i) {
        ahrs->grav[i] = 0.0f;
        ahrs->rotateAxisErrAcc[i] = 0.0f;
        ahrs->rotateAxisErrMag[i] = 0.0f;
        ahrs->refAxisMag[i] = 0.0f;

        ahrs->grav_norm[i] = 0.0f;
    }

    Quaternion_setIdentity(&ahrs->q_ae);
    Quaternion_setIdentity(&ahrs->q_me);
    Quaternion_setIdentity(&ahrs->q_a);
    Quaternion_setIdentity(&ahrs->q_i);
    Quaternion_setIdentity(&ahrs->RES);
    Quaternion_setIdentity(&ahrs->diff_q);
    Quaternion_setIdentity(&ahrs->q_Buf);// temp quaternion

    ahrs->recipNorm = 0.0f;
}


#undef QuatSize
#undef MagGyroSize

/*
 * **********************************************************************************************************************************************************************
 * Complementary filter based algorithms
 * **********************************************************************************************************************************************************************
 */

int ahrsProceedKalmanComplementaryGyroAccMag(AHRS_t* ahrs, AHRSinput * data) // original kalman fusion with magnetometer
{
    M_Assert_Break((ahrs == NULL || data == NULL), "ahrsProceed: vectors is null ptr", return KALMAN_ERR);
    M_Assert_Break((ahrs->init == NULL), "ahrsProceed:INIT is nullptr", return KALMAN_ERR);
    M_Assert_Break((ahrs->accDeltaQuaterFinder == NULL || ahrs->magDeltaQuaterFinder == NULL), "ahrsProceed: NULL delta quater functions", return KALMAN_ERR);
    M_Assert_Break((ahrs->init->Type != Kalman), "ahrsProceed: for Kalman fusion is not allocation some matrix, please choose ahrs->init->Type = Kalman", return KALMAN_ERR);

    /*
     * ***************************************************
     * Predict step 1
     * ***************************************************
     */
    // update F, F_t, G
    updateSysAHRS(data->g, data->dt_sec, ahrs->kalman->F->data, ahrs->kalman->F_t->data); // cg ==> F, cg0 ==> F_T (function generated from maple)
    kalmanPredict_withoutDrive(ahrs->kalman);

    Quaternion_normalize_vect(&ahrs->kalman->X_pred->data[0][0], &ahrs->kalman->X_pred->data[1][0], &ahrs->kalman->X_pred->data[2][0], &ahrs->kalman->X_pred->data[3][0]);
    Quaternion_set(ahrs->kalman->X_pred->data[0][0], ahrs->kalman->X_pred->data[1][0], ahrs->kalman->X_pred->data[2][0], ahrs->kalman->X_pred->data[3][0], &ahrs->q_a);

    updatePredictionNoiseAHRS(ahrs->init->gyroVarianceVect[0], ahrs->init->gyroVarianceVect[1], ahrs->init->gyroVarianceVect[2],
            ahrs->q_a.w, ahrs->q_a.v[0], ahrs->q_a.v[1],ahrs->q_a.v[2], data->dt_sec, ahrs->kalman->Q->data); // cg ==> Q (function generated from maple)
    //matrixCopy(ahrs->kalman->X_pred, ahrs->kalman->X_est);


    /*
     * ***************************************************
     * Update Step 2
     * ***************************************************
     */

    ////// accelerometer quaternion --------------------------------------------------------------------

    // normalize accel
    ahrs->recipNorm = fastinvsqrtf(data->a[0] * data->a[0] + data->a[1] * data->a[1] + data->a[2] * data->a[2]);
    data->a[0] *= ahrs->recipNorm;
    data->a[1] *= ahrs->recipNorm;
    data->a[2] *= ahrs->recipNorm;

    Quaternion_set(ahrs->q_a.w, -ahrs->q_a.v[0], -ahrs->q_a.v[1], -ahrs->q_a.v[2], &ahrs->q_Buf);
    Quaternion_rotate(&ahrs->q_Buf, data->a, ahrs->rotateAxisErrAcc);
    ahrs->accDeltaQuaterFinder[AHRS_COMPLEMENTARY_FILTER](ahrs, data);

    if(ahrs->q_ae.w > EPSILON_QUATER) {
        //LERP
        Quaternion_lerp(&ahrs->q_i, &ahrs->q_ae, ahrs->init->accConst_u, &ahrs->q_ae);
    } else {
        //SLERP
        Quaternion_slerp(&ahrs->q_i, &ahrs->q_ae, ahrs->init->accConst_u, &ahrs->q_ae);
    }

    Quaternion_normalize(&ahrs->q_ae, &ahrs->q_ae);
    Quaternion_multiply(&ahrs->q_a, &ahrs->q_ae, &ahrs->q_a);

    // ////// magnetometer quaternion --------------------------------------------------------------------

    magOffsetDeleteAHRS(&data->m[0], &data->m[1], &data->m[2], ahrs->init->magHardIronVect, ahrs->init->magSoftIronInvMat);

    // // normalize magnetometer
    ahrs->recipNorm = fastinvsqrtf(data->m[0] * data->m[0] + data->m[1] * data->m[1] + data->m[2] * data->m[2]);
    data->m[0] *= ahrs->recipNorm;
    data->m[1] *= ahrs->recipNorm;
    data->m[2] *= ahrs->recipNorm;

    Quaternion_set(ahrs->q_a.w, -ahrs->q_a.v[0], -ahrs->q_a.v[1], -ahrs->q_a.v[2], &ahrs->q_Buf);
    Quaternion_rotate(&ahrs->q_Buf, data->m, ahrs->rotateAxisErrMag);
    ahrs->magDeltaQuaterFinder[AHRS_COMPLEMENTARY_FILTER](ahrs, data);

    if(ahrs->q_me.w > EPSILON_QUATER) {
        //LERP
        Quaternion_lerp(&ahrs->q_i, &ahrs->q_me, ahrs->init->magConst_u, &ahrs->q_me);
    } else {
        //SLERP
        Quaternion_slerp(&ahrs->q_i, &ahrs->q_me, ahrs->init->magConst_u, &ahrs->q_me);
    }

    Quaternion_normalize(&ahrs->q_me, &ahrs->q_me);
    Quaternion_multiply_to_arrayLN(&ahrs->q_a, &ahrs->q_me, ahrs->kalman->Z->data);

    // update measurment R matrix ------------------------------------------------------------------------------------------------------------------------------
    updateMeasurmentJacobianMagAHRS_NED(data->m, ahrs->refAxisMag, data->a, ahrs->kalman->Z->data[0][0], ahrs->kalman->Z->data[1][0], ahrs->kalman->Z->data[2][0], ahrs->kalman->Z->data[3][0], ahrs->J->data,  ahrs->J_t->data); // cg ==> J, cgt ==> J_T (function generated from maple)
    multiply(ahrs->Noise_measurment, ahrs->J_t, ahrs->NOISE_R_RES);
    multiply(ahrs->J, ahrs->NOISE_R_RES, ahrs->kalman->R);
    //------------------------------------------------------------------------------------------------------------------------------------------------

    kalmanUpdate_withoutH(ahrs->kalman);
    Quaternion_normalize_vect(&ahrs->kalman->X_est->data[0][0], &ahrs->kalman->X_est->data[1][0], &ahrs->kalman->X_est->data[2][0], &ahrs->kalman->X_est->data[3][0]);


    //-----------------------------------------------------------------------------------------------------
    Quaternion_set(ahrs->kalman->X_est->data[0][0], ahrs->kalman->X_est->data[1][0], ahrs->kalman->X_est->data[2][0], ahrs->kalman->X_est->data[3][0], &ahrs->q_a); // qk
    //Quaternion_copy(&ahrs->q_a, &ahrs->RES);
    Quaternion_multiply(&ahrs->q_a, &ahrs->diff_q, &ahrs->RES); // correct result quaternion

    computeGravVectorAHRS(ahrs->q_a.w, ahrs->q_a.v[0], ahrs->q_a.v[1], ahrs->q_a.v[2], ahrs->grav, ahrs->init->gravityConstVect); // g calculate
    return KALMAN_OK;
}

int ahrsProceedKalmanComplementaryGyroAcc(AHRS_t* ahrs, AHRSinput * data) // original kalman fusion
{
    M_Assert_Break((ahrs == NULL || data == NULL), "ahrsProceed: vectors is null ptr", return KALMAN_ERR);
    M_Assert_Break((ahrs->init == NULL), "ahrsProceed:INIT is nullptr", return KALMAN_ERR);
    M_Assert_Break((ahrs->accDeltaQuaterFinder == NULL || ahrs->magDeltaQuaterFinder == NULL), "ahrsProceed: NULL delta quater functions", return KALMAN_ERR);
    M_Assert_Break((ahrs->init->Type != Kalman), "ahrsProceed: for Kalman fusion is not allocation some matrix, please choose ahrs->init->Type = Kalman", return KALMAN_ERR);

    /*
     * ***************************************************
     * Predict step 1
     * ***************************************************
     */
    // update F, F_t, G
    updateSysAHRS(data->g, data->dt_sec, ahrs->kalman->F->data, ahrs->kalman->F_t->data); // cg ==> F, cg0 ==> F_T (function generated from maple)
    kalmanPredict_withoutDrive(ahrs->kalman);

    Quaternion_normalize_vect(&ahrs->kalman->X_pred->data[0][0], &ahrs->kalman->X_pred->data[1][0], &ahrs->kalman->X_pred->data[2][0], &ahrs->kalman->X_pred->data[3][0]);
    Quaternion_set(ahrs->kalman->X_pred->data[0][0], ahrs->kalman->X_pred->data[1][0], ahrs->kalman->X_pred->data[2][0], ahrs->kalman->X_pred->data[3][0], &ahrs->q_a);

    updatePredictionNoiseAHRS(ahrs->init->gyroVarianceVect[0], ahrs->init->gyroVarianceVect[1], ahrs->init->gyroVarianceVect[2],
            ahrs->q_a.w, ahrs->q_a.v[0], ahrs->q_a.v[1],ahrs->q_a.v[2], data->dt_sec, ahrs->kalman->Q->data); // cg ==> Q (function generated from maple)
    //matrixCopy(ahrs->kalman->X_pred, ahrs->kalman->X_est);

    /*
     * ***************************************************
     * Update Step 2
     * ***************************************************
     */

    ////// accelerometer quaternion --------------------------------------------------------------------

    // normalize accel
    ahrs->recipNorm = fastinvsqrtf(data->a[0] * data->a[0] + data->a[1] * data->a[1] + data->a[2] * data->a[2]);
    data->a[0] *= ahrs->recipNorm;
    data->a[1] *= ahrs->recipNorm;
    data->a[2] *= ahrs->recipNorm;

    Quaternion_set(ahrs->q_a.w, -ahrs->q_a.v[0], -ahrs->q_a.v[1], -ahrs->q_a.v[2], &ahrs->q_Buf);
    Quaternion_rotate(&ahrs->q_Buf, data->a, ahrs->rotateAxisErrAcc);
    ahrs->accDeltaQuaterFinder[AHRS_COMPLEMENTARY_FILTER](ahrs, data);
    

    if(ahrs->q_ae.w > EPSILON_QUATER) {
        //LERP
        Quaternion_lerp(&ahrs->q_i, &ahrs->q_ae, ahrs->init->accConst_u, &ahrs->q_ae);
    } else {
        //SLERP
        Quaternion_slerp(&ahrs->q_i, &ahrs->q_ae, ahrs->init->accConst_u, &ahrs->q_ae);
    }

    Quaternion_normalize(&ahrs->q_ae, &ahrs->q_ae);
    Quaternion_multiply_to_arrayLN(&ahrs->q_a, &ahrs->q_ae, ahrs->kalman->Z->data);

    // update measurment R matrix ------------------------------------------------------------------------------------------------------------------------------
    updateMeasurmentJacobianAHRS_NED(data->a, ahrs->kalman->Z->data[0][0], ahrs->kalman->Z->data[1][0], ahrs->kalman->Z->data[2][0], ahrs->kalman->Z->data[3][0], ahrs->J->data,  ahrs->J_t->data); // cg ==> J, cgt ==> J_T (function generated from maple)
    multiply(ahrs->Noise_measurment, ahrs->J_t, ahrs->NOISE_R_RES);
    multiply(ahrs->J, ahrs->NOISE_R_RES, ahrs->kalman->R);
    //------------------------------------------------------------------------------------------------------------------------------------------------

    kalmanUpdate_withoutH(ahrs->kalman);
    Quaternion_normalize_vect(&ahrs->kalman->X_est->data[0][0], &ahrs->kalman->X_est->data[1][0], &ahrs->kalman->X_est->data[2][0], &ahrs->kalman->X_est->data[3][0]);


    //-----------------------------------------------------------------------------------------------------
    Quaternion_set(ahrs->kalman->X_est->data[0][0], ahrs->kalman->X_est->data[1][0], ahrs->kalman->X_est->data[2][0], ahrs->kalman->X_est->data[3][0], &ahrs->q_a); // qk
    //Quaternion_copy(&ahrs->q_a, &ahrs->RES);
    Quaternion_multiply(&ahrs->q_a, &ahrs->diff_q, &ahrs->RES); // correct result quaternion

    computeGravVectorAHRS(ahrs->q_a.w, ahrs->q_a.v[0], ahrs->q_a.v[1], ahrs->q_a.v[2], ahrs->grav, ahrs->init->gravityConstVect); // g calculate
    return KALMAN_OK;
}



int ahrsComplementaryFilterGyroAccMag(AHRS_t* ahrs, AHRSinput * data)
{
    M_Assert_Break((ahrs == NULL || data == NULL), "ahrsComplementaryFilterGyroAccMag: vectors is null ptr", return KALMAN_ERR);
    M_Assert_Break((ahrs->init == NULL), "ahrsComplementaryFilterGyroAccMag:INIT is nullptr", return KALMAN_ERR);
    M_Assert_Break((ahrs->accDeltaQuaterFinder == NULL || ahrs->magDeltaQuaterFinder == NULL), "ahrsComplementaryFilterGyroAccMag: NULL delta quater functions", return KALMAN_ERR);

    ////// gyroscope quaternion --------------------------------------------------------------------
    float halfT = -0.5f * data->dt_sec;
    Quaternion_set(0.0f, data->g[0], data->g[1], data->g[2], &ahrs->q_Buf);
    Quaternion_multiply(&ahrs->q_Buf, &ahrs->q_a, &ahrs->q_Buf);
    Quaternion_scalar_multiplication(&ahrs->q_Buf, halfT, &ahrs->q_Buf);
    Quaternion_add(&ahrs->q_a, &ahrs->q_Buf, &ahrs->q_a);
    Quaternion_normalize(&ahrs->q_a, &ahrs->q_a);

    ////// accelerometer quaternion --------------------------------------------------------------------

    // normalize accel
    ahrs->recipNorm = fastinvsqrtf(data->a[0] * data->a[0] + data->a[1] * data->a[1] + data->a[2] * data->a[2]);
    data->a[0] *= ahrs->recipNorm;
    data->a[1] *= ahrs->recipNorm;
    data->a[2] *= ahrs->recipNorm;

    Quaternion_set(ahrs->q_a.w, -ahrs->q_a.v[0], -ahrs->q_a.v[1], -ahrs->q_a.v[2], &ahrs->q_Buf);
    Quaternion_rotate(&ahrs->q_Buf, data->a, ahrs->rotateAxisErrAcc);
    ahrs->accDeltaQuaterFinder[AHRS_COMPLEMENTARY_FILTER](ahrs, data);

    if(ahrs->q_ae.w > EPSILON_QUATER) {
        //LERP
        Quaternion_lerp(&ahrs->q_i, &ahrs->q_ae, ahrs->init->accConst_u, &ahrs->q_ae);
    } else {
        //SLERP
        Quaternion_slerp(&ahrs->q_i, &ahrs->q_ae, ahrs->init->accConst_u, &ahrs->q_ae);
    }

    Quaternion_normalize(&ahrs->q_ae, &ahrs->q_ae);
    Quaternion_multiply(&ahrs->q_a, &ahrs->q_ae, &ahrs->q_a);

    ////// magnetometer quaternion --------------------------------------------------------------------

    magOffsetDeleteAHRS(&data->m[0], &data->m[1], &data->m[2], ahrs->init->magHardIronVect, ahrs->init->magSoftIronInvMat);

    // normalize magnetometer
    ahrs->recipNorm = fastinvsqrtf(data->m[0] * data->m[0] + data->m[1] * data->m[1] + data->m[2] * data->m[2]);
    data->m[0] *= ahrs->recipNorm;
    data->m[1] *= ahrs->recipNorm;
    data->m[2] *= ahrs->recipNorm;

    Quaternion_set(ahrs->q_a.w, -ahrs->q_a.v[0], -ahrs->q_a.v[1], -ahrs->q_a.v[2], &ahrs->q_Buf);
    Quaternion_rotate(&ahrs->q_Buf, data->m, ahrs->rotateAxisErrMag);
    ahrs->magDeltaQuaterFinder[AHRS_COMPLEMENTARY_FILTER](ahrs, data);

    if(ahrs->q_me.w > EPSILON_QUATER) {
        //LERP
        Quaternion_lerp(&ahrs->q_i, &ahrs->q_me, ahrs->init->magConst_u, &ahrs->q_me);
    } else {
        //SLERP
        Quaternion_slerp(&ahrs->q_i, &ahrs->q_me, ahrs->init->magConst_u, &ahrs->q_me);
    }

    Quaternion_normalize(&ahrs->q_me, &ahrs->q_me);
    Quaternion_multiply(&ahrs->q_a, &ahrs->q_me, &ahrs->q_a);

    //-----------------------------------------------------------------------------------------------------
    Quaternion_multiply(&ahrs->q_a, &ahrs->diff_q, &ahrs->RES); // correct result quaternion

    computeGravVectorAHRS(ahrs->q_a.w, ahrs->q_a.v[0], ahrs->q_a.v[1], ahrs->q_a.v[2], ahrs->grav, ahrs->init->gravityConstVect); // g calculate
    return KALMAN_OK;
}

int ahrsComplementaryFilterGyroAcc(AHRS_t* ahrs, AHRSinput * data)
{
    M_Assert_Break((ahrs == NULL || data == NULL), "ahrsComplementaryFilterGyroAcc: vectors is null ptr", return KALMAN_ERR);
    M_Assert_Break((ahrs->init == NULL), "ahrsComplementaryFilterGyroAcc:INIT is nullptr", return KALMAN_ERR);
    M_Assert_Break((ahrs->accDeltaQuaterFinder == NULL || ahrs->magDeltaQuaterFinder == NULL), "ahrsComplementaryFilterGyroAcc: NULL delta quater functions", return KALMAN_ERR);

    ////// gyroscope quaternion --------------------------------------------------------------------
    float halfT = -0.5f * data->dt_sec;
    Quaternion_set(0.0f, data->g[0], data->g[1], data->g[2], &ahrs->q_Buf);
    Quaternion_multiply(&ahrs->q_Buf, &ahrs->q_a, &ahrs->q_Buf);
    Quaternion_scalar_multiplication(&ahrs->q_Buf, halfT, &ahrs->q_Buf);
    Quaternion_add(&ahrs->q_a, &ahrs->q_Buf, &ahrs->q_a);
    Quaternion_normalize(&ahrs->q_a, &ahrs->q_a);

    ////// accelerometer quaternion --------------------------------------------------------------------

    // normalize accel
    ahrs->recipNorm = fastinvsqrtf(data->a[0] * data->a[0] + data->a[1] * data->a[1] + data->a[2] * data->a[2]);
    data->a[0] *= ahrs->recipNorm;
    data->a[1] *= ahrs->recipNorm;
    data->a[2] *= ahrs->recipNorm;

    Quaternion_set(ahrs->q_a.w, -ahrs->q_a.v[0], -ahrs->q_a.v[1], -ahrs->q_a.v[2], &ahrs->q_Buf);
    Quaternion_rotate(&ahrs->q_Buf, data->a, ahrs->rotateAxisErrAcc);
    ahrs->accDeltaQuaterFinder[AHRS_COMPLEMENTARY_FILTER](ahrs, data);

    if(ahrs->q_ae.w > EPSILON_QUATER) {
        //LERP
        Quaternion_lerp(&ahrs->q_i, &ahrs->q_ae, ahrs->init->accConst_u, &ahrs->q_ae);
    } else {
        //SLERP
        Quaternion_slerp(&ahrs->q_i, &ahrs->q_ae, ahrs->init->accConst_u, &ahrs->q_ae);
    }

    Quaternion_normalize(&ahrs->q_ae, &ahrs->q_ae);
    Quaternion_multiply(&ahrs->q_a, &ahrs->q_ae, &ahrs->q_a);

    //-----------------------------------------------------------------------------------------------------
    Quaternion_multiply(&ahrs->q_a, &ahrs->diff_q, &ahrs->RES); // correct result quaternion
    computeGravVectorAHRS(ahrs->q_a.w, ahrs->q_a.v[0], ahrs->q_a.v[1], ahrs->q_a.v[2], ahrs->grav, ahrs->init->gravityConstVect); // g calculate
    return KALMAN_OK;
}


/*
 * **********************************************************************************************************************************************************************
 * orientation from some one sensor
 * **********************************************************************************************************************************************************************
 */

int ahrsOnlyGyro(AHRS_t* ahrs, AHRSinput * data)
{
    M_Assert_Break((ahrs == NULL || data == NULL), "ahrsOnlyGyro: vectors is null ptr", return KALMAN_ERR);
    M_Assert_Break((ahrs->init == NULL), "ahrsOnlyGyro:INIT is nullptr", return KALMAN_ERR);
    M_Assert_Break((ahrs->accDeltaQuaterFinder == NULL || ahrs->magDeltaQuaterFinder == NULL), "ahrsOnlyGyro: NULL delta quater functions", return KALMAN_ERR);

    ////// gyroscope quaternion --------------------------------------------------------------------
    float halfT = -0.5f * data->dt_sec;
    Quaternion_set(0.0f, data->g[0], data->g[1], data->g[2], &ahrs->q_Buf);
    Quaternion_multiply(&ahrs->q_Buf, &ahrs->q_a, &ahrs->q_Buf);
    Quaternion_scalar_multiplication(&ahrs->q_Buf, halfT, &ahrs->q_Buf);
    Quaternion_add(&ahrs->q_a, &ahrs->q_Buf, &ahrs->q_a);
    Quaternion_normalize(&ahrs->q_a, &ahrs->q_a);

    //-----------------------------------------------------------------------------------------------------
    Quaternion_multiply(&ahrs->q_a, &ahrs->diff_q, &ahrs->RES); // correct result quaternion
    computeGravVectorAHRS(ahrs->q_a.w, ahrs->q_a.v[0], ahrs->q_a.v[1], ahrs->q_a.v[2], ahrs->grav, ahrs->init->gravityConstVect); // g calculate
    return KALMAN_OK;
}

int ahrsOnlyAcc(AHRS_t* ahrs, AHRSinput * data)
{
    M_Assert_Break((ahrs == NULL || data == NULL), "ahrsOnlyAcc: vectors is null ptr", return KALMAN_ERR);
    M_Assert_Break((ahrs->init == NULL), "ahrsOnlyAcc:INIT is nullptr", return KALMAN_ERR);
    M_Assert_Break((ahrs->accDeltaQuaterFinder == NULL || ahrs->magDeltaQuaterFinder == NULL), "ahrsOnlyAcc: NULL delta quater functions", return KALMAN_ERR);

    ////// accelerometer quaternion --------------------------------------------------------------------

    // normalize accel
    ahrs->recipNorm = fastinvsqrtf(data->a[0] * data->a[0] + data->a[1] * data->a[1] + data->a[2] * data->a[2]);
    data->a[0] *= ahrs->recipNorm;
    data->a[1] *= ahrs->recipNorm;
    data->a[2] *= ahrs->recipNorm;

    ahrs->rotateAxisErrAcc[0] = data->a[0];
    ahrs->rotateAxisErrAcc[1] = data->a[1];
    ahrs->rotateAxisErrAcc[2] = data->a[2];

    ahrs->accDeltaQuaterFinder[AHRS_COMPLEMENTARY_FILTER](ahrs, data);

    Quaternion_normalize(&ahrs->q_ae, &ahrs->q_a);

    //-----------------------------------------------------------------------------------------------------
    Quaternion_multiply(&ahrs->q_a, &ahrs->diff_q, &ahrs->RES); // correct result quaternion
    computeGravVectorAHRS(ahrs->q_a.w, ahrs->q_a.v[0], ahrs->q_a.v[1], ahrs->q_a.v[2], ahrs->grav, ahrs->init->gravityConstVect); // g calculate
    return KALMAN_OK;
}

int ahrsOnlyMag(AHRS_t* ahrs, AHRSinput * data)
{
    M_Assert_Break((ahrs == NULL || data == NULL), "ahrsOnlyMag: vectors is null ptr", return KALMAN_ERR);
    M_Assert_Break((ahrs->init == NULL), "ahrsOnlyMag:INIT is nullptr", return KALMAN_ERR);
    M_Assert_Break((ahrs->accDeltaQuaterFinder == NULL || ahrs->magDeltaQuaterFinder == NULL), "ahrsOnlyMag: NULL delta quater functions", return KALMAN_ERR);

    ////// magnetometer quaternion --------------------------------------------------------------------

    magOffsetDeleteAHRS(&data->m[0], &data->m[1], &data->m[2], ahrs->init->magHardIronVect, ahrs->init->magSoftIronInvMat);

    // normalize magnetometer
    ahrs->recipNorm = fastinvsqrtf(data->m[0] * data->m[0] + data->m[1] * data->m[1] + data->m[2] * data->m[2]);
    data->m[0] *= ahrs->recipNorm;
    data->m[1] *= ahrs->recipNorm;
    data->m[2] *= ahrs->recipNorm;

    ahrs->rotateAxisErrMag[0] = data->m[0];
    ahrs->rotateAxisErrMag[1] = data->m[1];
    ahrs->rotateAxisErrMag[2] = data->m[2];

    ahrs->magDeltaQuaterFinder[AHRS_COMPLEMENTARY_FILTER](ahrs, data);

    Quaternion_normalize(&ahrs->q_me, &ahrs->q_a);

    //-----------------------------------------------------------------------------------------------------
    Quaternion_multiply(&ahrs->q_a, &ahrs->diff_q, &ahrs->RES); // correct result quaternion
    computeGravVectorAHRS(ahrs->q_a.w, ahrs->q_a.v[0], ahrs->q_a.v[1], ahrs->q_a.v[2], ahrs->grav, ahrs->init->gravityConstVect); // g calculate
    return KALMAN_OK;
}

/*
 * **********************************************************************************************************************************************************************
 * Madgwick filter (from adafruit AHRS)
 * **********************************************************************************************************************************************************************
 */

int ahrsProceedKalmanMadgwickGyroAcc(AHRS_t* ahrs, AHRSinput * data) // madgwick + kalman fusion
{
    M_Assert_Break((ahrs == NULL || data == NULL), "ahrsProceedKalmanMadgwick: vectors is null ptr", return KALMAN_ERR);
    M_Assert_Break((ahrs->init == NULL), "ahrsProceedKalmanMadgwick:INIT is nullptr", return KALMAN_ERR);
    M_Assert_Break((ahrs->accDeltaQuaterFinder == NULL || ahrs->magDeltaQuaterFinder == NULL || ahrs->accDeltaQuaterFinder[AHRS_MADGWICK_FILTER] == NULL || ahrs->magDeltaQuaterFinder[AHRS_MADGWICK_FILTER] == NULL), "ahrsProceedKalmanMadgwick: NULL delta quater functions", return KALMAN_ERR);
    M_Assert_Break((ahrs->init->Type != Kalman), "ahrsProceedKalmanMadgwick: for Kalman fusion is not allocation some matrix, please choose ahrs->init->Type = Kalman", return KALMAN_ERR);

    /*
     * ***************************************************
     * Predict step 1
     * ***************************************************
     */

    //    for (unsigned int i  = 0; i < 3; ++i) {
    //        data->a[i] += ahrs->init->accelBiasVect[i];
    //        data->g[i] += ahrs->init->gyroBiasVect[i];
    //    }

    // update F, F_t, G
    updateSysAHRS(data->g, data->dt_sec, ahrs->kalman->F->data, ahrs->kalman->F_t->data); // cg ==> F, cg0 ==> F_T(function generated from maple)
    kalmanPredict_withoutDrive(ahrs->kalman);

    Quaternion_normalize_vect(&ahrs->kalman->X_pred->data[0][0], &ahrs->kalman->X_pred->data[1][0], &ahrs->kalman->X_pred->data[2][0], &ahrs->kalman->X_pred->data[3][0]);
    Quaternion_set(ahrs->kalman->X_pred->data[0][0], ahrs->kalman->X_pred->data[1][0], ahrs->kalman->X_pred->data[2][0], ahrs->kalman->X_pred->data[3][0], &ahrs->q_ae);

    //    updatePredictionNoise(ahrs->init->gyroVarianceVect[0], ahrs->init->gyroVarianceVect[1], ahrs->init->gyroVarianceVect[2],
    //            ahrs->q_ae.w, ahrs->q_ae.v[0], ahrs->q_ae.v[1],ahrs->q_ae.v[2], data->dt_sec, ahrs->kalman->Q->data); // cg ==> Q (function generated from maple)

    /*
     * ***************************************************
     * Update Step 2
     * ***************************************************
     */

    ahrsMadgwickGyroAcc(ahrs, data);

    ahrs->kalman->Z->data[0][0] = ahrs->q_a.w;
    ahrs->kalman->Z->data[1][0] = ahrs->q_a.v[0];
    ahrs->kalman->Z->data[2][0] = ahrs->q_a.v[1];
    ahrs->kalman->Z->data[3][0] = ahrs->q_a.v[2];

    // update measurment R matrix ------------------------------------------------------------------------------------------------------------------------------
    //    updateMeasurmentJacobianMag(data->m, ahrs->refAxisMag, data->a, ahrs->kalman->Z->data[0][0], ahrs->kalman->Z->data[1][0], ahrs->kalman->Z->data[2][0], ahrs->kalman->Z->data[3][0], ahrs->J->data,  ahrs->J_t->data); // cg ==> J, cgt ==> J_T (function generated from maple)
    //    multiply(ahrs->Noise_measurment, ahrs->J_t, ahrs->NOISE_R_RES);
    //    multiply(ahrs->J, ahrs->NOISE_R_RES, ahrs->kalman->R);
    //------------------------------------------------------------------------------------------------------------------------------------------------

    kalmanUpdate_withoutH(ahrs->kalman);
    Quaternion_normalize_vect(&ahrs->kalman->X_est->data[0][0], &ahrs->kalman->X_est->data[1][0], &ahrs->kalman->X_est->data[2][0], &ahrs->kalman->X_est->data[3][0]);

    //-----------------------------------------------------------------------------------------------------
    Quaternion_set(ahrs->kalman->X_est->data[0][0], ahrs->kalman->X_est->data[1][0], ahrs->kalman->X_est->data[2][0], ahrs->kalman->X_est->data[3][0], &ahrs->q_ae); // qk
    Quaternion_multiply(&ahrs->q_ae, &ahrs->diff_q, &ahrs->RES); // correct result quaternion

    computeGravVectorAHRS(ahrs->q_ae.w, ahrs->q_ae.v[0], ahrs->q_ae.v[1], ahrs->q_ae.v[2], ahrs->grav, ahrs->init->gravityConstVect); // g calculate
    return KALMAN_OK;
}

int ahrsMadgwickGyroAccMag(AHRS_t* ahrs, AHRSinput * data)
{
    M_Assert_Break((ahrs == NULL || data == NULL), "ahrsMadgwickGyroAccMag: vectors is null ptr", return KALMAN_ERR);
    M_Assert_Break((ahrs->init == NULL), "ahrsMadgwickGyroAccMag:INIT is nullptr", return KALMAN_ERR);
    M_Assert_Break((ahrs->accDeltaQuaterFinder == NULL || ahrs->magDeltaQuaterFinder == NULL), "ahrsMadgwickGyroAccMag: NULL delta quater functions", return KALMAN_ERR);


    //  // Use ahrs algorithm if magnetometer measurement invalid (avoids NaN in
    //  // magnetometer normalisation)
    //  if ((data->m[0] == 0.0f) && (data->m[1] == 0.0f) && (data->m[2] == 0.0f)) {
    //    updateahrs(data->g[0], data->g[1], data->g[2], data->a[0], data->a[1], data->a[2], data->dt_sec);
    //    return 0;
    //  }

    // Rate of change of quaternion from gyroscope
    float qDot1 = 0.5f * (-ahrs->q_Buf.v[0] * data->g[0]   - ahrs->q_Buf.v[1] * data->g[1] - ahrs->q_Buf.v[2] * data->g[2]);
    float qDot2 = 0.5f * (ahrs->q_Buf.w * data->g[0]       + ahrs->q_Buf.v[1] * data->g[2] - ahrs->q_Buf.v[2] * data->g[1]);
    float qDot3 = 0.5f * (ahrs->q_Buf.w * data->g[1]       - ahrs->q_Buf.v[0] * data->g[2] + ahrs->q_Buf.v[2] * data->g[0]);
    float qDot4 = 0.5f * (ahrs->q_Buf.w * data->g[2]       + ahrs->q_Buf.v[0] * data->g[1] - ahrs->q_Buf.v[1] * data->g[0]);

    // Compute feedback only if accelerometer measurement valid (avoids NaN in
    // accelerometer normalisation)
    if (!((data->a[0] == 0.0f) && (data->a[1] == 0.0f) && (data->a[2] == 0.0f))) {

        // Normalise accelerometer measurement
        ahrs->recipNorm = fastinvsqrtf(data->a[0] * data->a[0] + data->a[1] * data->a[1] + data->a[2] * data->a[2]);
        data->a[0] *= ahrs->recipNorm;
        data->a[1] *= ahrs->recipNorm;
        data->a[2] *= ahrs->recipNorm;

        // Normalise magnetometer measurement
        magOffsetDeleteAHRS(&data->m[0], &data->m[1], &data->m[2], ahrs->init->magHardIronVect, ahrs->init->magSoftIronInvMat);
        ahrs->recipNorm = fastinvsqrtf(data->m[0] * data->m[0] + data->m[1] * data->m[1] + data->m[2] * data->m[2]);
        data->m[0] *= ahrs->recipNorm;
        data->m[1] *= ahrs->recipNorm;
        data->m[2] *= ahrs->recipNorm;

        Quaternion_rotate(&ahrs->q_Buf, data->m, ahrs->rotateAxisErrMag);
        ahrs->magDeltaQuaterFinder[AHRS_MADGWICK_FILTER](ahrs, data);
        Quaternion_normalize(&ahrs->q_me, &ahrs->q_me); // normalise step magnitude

        // Apply feedback step
        qDot1 -= ahrs->init->accConst_u * ahrs->q_me.w   ;
        qDot2 -= ahrs->init->accConst_u * ahrs->q_me.v[0];
        qDot3 -= ahrs->init->accConst_u * ahrs->q_me.v[1];
        qDot4 -= ahrs->init->accConst_u * ahrs->q_me.v[2];
    }

    // Integrate rate of change of quaternion to yield quaternion
    ahrs->q_Buf.w    += qDot1 * data->dt_sec;
    ahrs->q_Buf.v[0] += qDot2 * data->dt_sec;
    ahrs->q_Buf.v[1] += qDot3 * data->dt_sec;
    ahrs->q_Buf.v[2] += qDot4 * data->dt_sec;

    // Normalise quaternion
    Quaternion_normalize(&ahrs->q_Buf, &ahrs->q_Buf);

    Quaternion_set(ahrs->q_Buf.w, -ahrs->q_Buf.v[0], -ahrs->q_Buf.v[1], -ahrs->q_Buf.v[2], &ahrs->q_a);
    Quaternion_multiply(&ahrs->q_a, &ahrs->diff_q, &ahrs->RES);
    computeGravVectorAHRS(ahrs->q_a.w, ahrs->q_a.v[0], ahrs->q_a.v[1], ahrs->q_a.v[2], ahrs->grav, ahrs->init->gravityConstVect); // g calculate
    return KALMAN_OK;
}


int ahrsMadgwickGyroAcc(AHRS_t* ahrs, AHRSinput * data)
{
    M_Assert_Break((ahrs == NULL || data == NULL), "ahrsMadgwickGyroAcc: vectors is null ptr", return KALMAN_ERR);
    M_Assert_Break((ahrs->init == NULL), "ahrsMadgwickGyroAcc:INIT is nullptr", return KALMAN_ERR);
    M_Assert_Break((ahrs->accDeltaQuaterFinder == NULL || ahrs->magDeltaQuaterFinder == NULL || ahrs->accDeltaQuaterFinder[AHRS_MADGWICK_FILTER] == NULL || ahrs->magDeltaQuaterFinder[AHRS_MADGWICK_FILTER] == NULL), "ahrsMadgwickGyroAcc: NULL delta quater functions", return KALMAN_ERR);

    // Rate of change of quaternion from gyroscope
    float qDot1 = 0.5f * (-ahrs->q_Buf.v[0] * data->g[0]   - ahrs->q_Buf.v[1] * data->g[1] - ahrs->q_Buf.v[2] * data->g[2]);
    float qDot2 = 0.5f * (ahrs->q_Buf.w * data->g[0]       + ahrs->q_Buf.v[1] * data->g[2] - ahrs->q_Buf.v[2] * data->g[1]);
    float qDot3 = 0.5f * (ahrs->q_Buf.w * data->g[1]       - ahrs->q_Buf.v[0] * data->g[2] + ahrs->q_Buf.v[2] * data->g[0]);
    float qDot4 = 0.5f * (ahrs->q_Buf.w * data->g[2]       + ahrs->q_Buf.v[0] * data->g[1] - ahrs->q_Buf.v[1] * data->g[0]);

    // Compute feedback only if accelerometer measurement valid (avoids NaN in
    // accelerometer normalisation)
    if (!((data->a[0] == 0.0f) && (data->a[1] == 0.0f) && (data->a[2] == 0.0f))) {

        // Normalise accelerometer measurement
        ahrs->recipNorm = fastinvsqrtf(data->a[0] * data->a[0] + data->a[1] * data->a[1] + data->a[2] * data->a[2]);
        data->a[0] *= ahrs->recipNorm;
        data->a[1] *= ahrs->recipNorm;
        data->a[2] *= ahrs->recipNorm;

        ahrs->accDeltaQuaterFinder[AHRS_MADGWICK_FILTER](ahrs, data);
        Quaternion_normalize(&ahrs->q_ae, &ahrs->q_ae); // normalise step magnitude

        // Apply feedback step
        qDot1 -= ahrs->init->accConst_u * ahrs->q_ae.w   ;
        qDot2 -= ahrs->init->accConst_u * ahrs->q_ae.v[0];
        qDot3 -= ahrs->init->accConst_u * ahrs->q_ae.v[1];
        qDot4 -= ahrs->init->accConst_u * ahrs->q_ae.v[2];
    }

    // Integrate rate of change of quaternion to yield quaternion
    ahrs->q_Buf.w    += qDot1 * data->dt_sec;
    ahrs->q_Buf.v[0] += qDot2 * data->dt_sec;
    ahrs->q_Buf.v[1] += qDot3 * data->dt_sec;
    ahrs->q_Buf.v[2] += qDot4 * data->dt_sec;

    // Normalise quaternion
    Quaternion_normalize(&ahrs->q_Buf, &ahrs->q_Buf);

    Quaternion_set(ahrs->q_Buf.w, -ahrs->q_Buf.v[0], -ahrs->q_Buf.v[1], -ahrs->q_Buf.v[2], &ahrs->q_a);
    Quaternion_multiply(&ahrs->q_a, &ahrs->diff_q, &ahrs->RES);
    computeGravVectorAHRS(ahrs->q_a.w, ahrs->q_a.v[0], ahrs->q_a.v[1], ahrs->q_a.v[2], ahrs->grav, ahrs->init->gravityConstVect); // g calculate
    return KALMAN_OK;
}

/*
 * **********************************************************************************************************************************************************************
 * Mahony filter (from adafruit AHRS)
 * **********************************************************************************************************************************************************************
 */

int ahrsMahonyGyroAcc(AHRS_t* ahrs, AHRSinput * data)
{
    M_Assert_Break((ahrs == NULL || data == NULL), "ahrsMahonyGyroAcc: vectors is null ptr", return KALMAN_ERR);
    M_Assert_Break((ahrs->init == NULL), "ahrsMahonyGyroAcc:INIT is nullptr", return KALMAN_ERR);
    M_Assert_Break((ahrs->accDeltaQuaterFinder == NULL || ahrs->magDeltaQuaterFinder == NULL), "ahrsMahonyGyroAcc: NULL delta quater functions", return KALMAN_ERR);

    float recipNorm;
    float halfvx, halfvy, halfvz;
    float halfex, halfey, halfez;
    float qa, qb, qc;

    // Convert gyroscope degrees/sec to radians/sec
    float gx = data->g[0];
    float gy = data->g[1];
    float gz = data->g[2];

    float ax = data->a[0];
    float ay = data->a[1];
    float az = data->a[2];

    float dt = data->dt_sec;

    float q0 = ahrs->q_a.w;
    float q1 = -ahrs->q_a.v[0];
    float q2 = -ahrs->q_a.v[1];
    float q3 = -ahrs->q_a.v[2];

    // Compute feedback only if accelerometer measurement valid
    // (avoids NaN in accelerometer normalisation)
    if (!((ax == 0.0f) && (ay == 0.0f) && (az == 0.0f))) {

        // Normalise accelerometer measurement
        recipNorm = fastinvsqrtf(ax * ax + ay * ay + az * az);
        ax *= recipNorm;
        ay *= recipNorm;
        az *= recipNorm;

        // Estimated direction of gravity
        halfvx = q1 * q3 - q0 * q2;
        halfvy = q0 * q1 + q2 * q3;
        halfvz = q0 * q0 - 0.5f + q3 * q3;

        // Error is sum of cross product between estimated
        // and measured direction of gravity
        halfex = (ay * halfvz - az * halfvy);
        halfey = (az * halfvx - ax * halfvz);
        halfez = (ax * halfvy - ay * halfvx);

        // Compute and apply integral feedback if enabled
        if (ahrs->init->magConst_u > 0.0f) {
            // integral error scaled by Ki
            ahrs->rotateAxisErrAcc[0] += ahrs->init->magConst_u * halfex * dt;
            ahrs->rotateAxisErrAcc[1] += ahrs->init->magConst_u * halfey * dt;
            ahrs->rotateAxisErrAcc[2] += ahrs->init->magConst_u * halfez * dt;
            gx += ahrs->rotateAxisErrAcc[0]; // apply integral feedback
            gy += ahrs->rotateAxisErrAcc[1];
            gz += ahrs->rotateAxisErrAcc[2];
        } else {
            ahrs->rotateAxisErrAcc[0] = 0.0f; // prevent integral windup
            ahrs->rotateAxisErrAcc[1] = 0.0f;
            ahrs->rotateAxisErrAcc[2] = 0.0f;
        }

        // Apply proportional feedback
        gx += ahrs->init->accConst_u * halfex;
        gy += ahrs->init->accConst_u * halfey;
        gz += ahrs->init->accConst_u * halfez;
    }

    // Integrate rate of change of quaternion
    gx *= (0.5f * dt); // pre-multiply common factors
    gy *= (0.5f * dt);
    gz *= (0.5f * dt);
    qa = q0;
    qb = q1;
    qc = q2;
    q0 += (-qb * gx - qc * gy - q3 * gz);
    q1 += (qa * gx + qc * gz - q3 * gy);
    q2 += (qa * gy - qb * gz + q3 * gx);
    q3 += (qa * gz + qb * gy - qc * gx);

    // Normalise quaternion
    recipNorm = fastinvsqrtf(q0 * q0 + q1 * q1 + q2 * q2 + q3 * q3);
    q0 *= recipNorm;
    q1 *= recipNorm;
    q2 *= recipNorm;
    q3 *= recipNorm;

    Quaternion_set(q0, -q1, -q2, -q3, &ahrs->q_a);
    Quaternion_multiply(&ahrs->q_a, &ahrs->diff_q, &ahrs->RES);
    computeGravVectorAHRS(ahrs->q_a.w, ahrs->q_a.v[0], ahrs->q_a.v[1], ahrs->q_a.v[2], ahrs->grav, ahrs->init->gravityConstVect); // g calculate
    return KALMAN_OK;
}

int ahrsMahonyGyroAccMag(AHRS_t* ahrs, AHRSinput * data)
{
    M_Assert_Break((ahrs == NULL || data == NULL), "ahrsMahonyGyroAccMag: vectors is null ptr", return KALMAN_ERR);
    M_Assert_Break((ahrs->init == NULL), "ahrsMahonyGyroAccMag:INIT is nullptr", return KALMAN_ERR);
    M_Assert_Break((ahrs->accDeltaQuaterFinder == NULL || ahrs->magDeltaQuaterFinder == NULL), "ahrsMahonyGyroAccMag: NULL delta quater functions", return KALMAN_ERR);


    float recipNorm;
    float q0q0, q0q1, q0q2, q0q3, q1q1, q1q2, q1q3, q2q2, q2q3, q3q3;
    float hx, hy, bx, bz;
    float halfvx, halfvy, halfvz, halfwx, halfwy, halfwz;
    float halfex, halfey, halfez;
    float qa, qb, qc;

    //  // Use ahrs algorithm if magnetometer measurement invalid
    //  // (avoids NaN in magnetometer normalisation)
    //  if ((mx == 0.0f) && (my == 0.0f) && (mz == 0.0f)) {
    //    updateahrs(gx, gy, gz, ax, ay, az);
    //    return;
    //  }

    // Convert gyroscope degrees/sec to radians/sec
    float gx = data->g[0];
    float gy = data->g[1];
    float gz = data->g[2];

    float ax = data->a[0];
    float ay = data->a[1];
    float az = data->a[2];

    float mx = data->m[0];
    float my = data->m[1];
    float mz = data->m[2];

    float dt = data->dt_sec;

    float q0 = ahrs->q_a.w;
    float q1 = -ahrs->q_a.v[0];
    float q2 = -ahrs->q_a.v[1];
    float q3 = -ahrs->q_a.v[2];

    // Compute feedback only if accelerometer measurement valid
    // (avoids NaN in accelerometer normalisation)
    if (!((ax == 0.0f) && (ay == 0.0f) && (az == 0.0f))) {

        // Normalise accelerometer measurement
        recipNorm = fastinvsqrtf(ax * ax + ay * ay + az * az);
        ax *= recipNorm;
        ay *= recipNorm;
        az *= recipNorm;

        // Normalise magnetometer measurement
        magOffsetDeleteAHRS(&data->m[0], &data->m[1], &data->m[2], ahrs->init->magHardIronVect, ahrs->init->magSoftIronInvMat);
        recipNorm = fastinvsqrtf(mx * mx + my * my + mz * mz);
        mx *= recipNorm;
        my *= recipNorm;
        mz *= recipNorm;

        // Auxiliary variables to avoid repeated arithmetic
        q0q0 = q0 * q0;
        q0q1 = q0 * q1;
        q0q2 = q0 * q2;
        q0q3 = q0 * q3;
        q1q1 = q1 * q1;
        q1q2 = q1 * q2;
        q1q3 = q1 * q3;
        q2q2 = q2 * q2;
        q2q3 = q2 * q3;
        q3q3 = q3 * q3;

        // Reference direction of Earth's magnetic field
        hx = 2.0f *
                (mx * (0.5f - q2q2 - q3q3) + my * (q1q2 - q0q3) + mz * (q1q3 + q0q2));
        hy = 2.0f *
                (mx * (q1q2 + q0q3) + my * (0.5f - q1q1 - q3q3) + mz * (q2q3 - q0q1));
        bx = sqrtf(hx * hx + hy * hy);
        bz = 2.0f *
                (mx * (q1q3 - q0q2) + my * (q2q3 + q0q1) + mz * (0.5f - q1q1 - q2q2));

        // Estimated direction of gravity and magnetic field
        halfvx = q1q3 - q0q2;
        halfvy = q0q1 + q2q3;
        halfvz = q0q0 - 0.5f + q3q3;
        halfwx = bx * (0.5f - q2q2 - q3q3) + bz * (q1q3 - q0q2);
        halfwy = bx * (q1q2 - q0q3) + bz * (q0q1 + q2q3);
        halfwz = bx * (q0q2 + q1q3) + bz * (0.5f - q1q1 - q2q2);

        // Error is sum of cross product between estimated direction
        // and measured direction of field vectors
        halfex = (ay * halfvz - az * halfvy) + (my * halfwz - mz * halfwy);
        halfey = (az * halfvx - ax * halfvz) + (mz * halfwx - mx * halfwz);
        halfez = (ax * halfvy - ay * halfvx) + (mx * halfwy - my * halfwx);

        // Compute and apply integral feedback if enabled
        if (ahrs->init->magConst_u > 0.0f) {
            // integral error scaled by Ki
            ahrs->rotateAxisErrAcc[0] += ahrs->init->magConst_u * halfex * dt;
            ahrs->rotateAxisErrAcc[1] += ahrs->init->magConst_u * halfey * dt;
            ahrs->rotateAxisErrAcc[2] += ahrs->init->magConst_u * halfez * dt;
            gx += ahrs->rotateAxisErrAcc[0]; // apply integral feedback
            gy += ahrs->rotateAxisErrAcc[1];
            gz += ahrs->rotateAxisErrAcc[2];
        } else {
            ahrs->rotateAxisErrAcc[0] = 0.0f; // prevent integral windup
            ahrs->rotateAxisErrAcc[1] = 0.0f;
            ahrs->rotateAxisErrAcc[2] = 0.0f;
        }

        // Apply proportional feedback
        gx += ahrs->init->accConst_u * halfex;
        gy += ahrs->init->accConst_u * halfey;
        gz += ahrs->init->accConst_u * halfez;
    }

    // Integrate rate of change of quaternion
    gx *= (0.5f * dt); // pre-multiply common factors
    gy *= (0.5f * dt);
    gz *= (0.5f * dt);
    qa = q0;
    qb = q1;
    qc = q2;
    q0 += (-qb * gx - qc * gy - q3 * gz);
    q1 += (qa * gx + qc * gz - q3 * gy);
    q2 += (qa * gy - qb * gz + q3 * gx);
    q3 += (qa * gz + qb * gy - qc * gx);

    // Normalise quaternion
    recipNorm = fastinvsqrtf(q0 * q0 + q1 * q1 + q2 * q2 + q3 * q3);
    q0 *= recipNorm;
    q1 *= recipNorm;
    q2 *= recipNorm;
    q3 *= recipNorm;

    Quaternion_set(q0, -q1, -q2, -q3, &ahrs->q_a);
    Quaternion_multiply(&ahrs->q_a, &ahrs->diff_q, &ahrs->RES);
    computeGravVectorAHRS(ahrs->q_a.w, ahrs->q_a.v[0], ahrs->q_a.v[1], ahrs->q_a.v[2], ahrs->grav, ahrs->init->gravityConstVect); // g calculate
    return KALMAN_OK;
}


//****************************************************************************************************************************************************************************





int ahrsInitStartQuat(AHRS_t* ahrs, AHRSinput * data, int meanIterations, int iterationsStart, AHRS_polymorph_t proceedFoo) // read start quaternion
{
    M_Assert_Break((proceedFoo == NULL), "ahrsInitStartQuat: proceedFoo is null", return KALMAN_ERR);
    M_Assert_Break((ahrs == NULL || data == NULL), "ahrsInitStartQuat: vectors is null ptr", return KALMAN_ERR);
    M_Assert_Break((ahrs->init == NULL), "ahrsInitStartQuat:INIT is nullptr", return KALMAN_ERR);
    M_Assert_Break((ahrs->accDeltaQuaterFinder == NULL || ahrs->magDeltaQuaterFinder == NULL), "ahrsInitStartQuat: NULL delta quater functions", return KALMAN_ERR);

    static int iterator = 0;
    static Quaternion qTemp = (Quaternion){0.0f, {0.0f, 0.0f, 0.0f}};

    static int state = 0;
    static float temp_mag = 0.0f;
    static float temp_acc = 0.0f;


    switch (state) {

    case(0): {
        iterator = 0;
        Quaternion_set(0.0f, 0.0f, 0.0f, 0.0f, &qTemp);
        Quaternion_setIdentity(&ahrs->diff_q);
        temp_mag = ahrs->init->magConst_u;
        temp_acc = ahrs->init->accConst_u;
        ahrs->init->magConst_u = 1.0f;
        ahrs->init->accConst_u = 1.0f;
        ++state;
        return KALMAN_PROC;
        break;
    }

    case(1): {

        //ahrsComplementaryFilterGyroAccMag(ahrs, data);

        float P_init[KALx][KALx] =
        {
            {500,  0,  0,  0},
            {0,  500,  0,  0},
            {0,  0,  500,  0},
            {0,  0,  0,  500}
        };
        matrixInitFromArr(ahrs->kalman->P_est, P_init[0]);
        proceedFoo(ahrs, data);

        if(iterator < (meanIterations + iterationsStart + 1)) {
            if(iterator < iterationsStart) {
                Quaternion_set(0.0f, 0.0f, 0.0f, 0.0f, &qTemp);
            }

            if (iterator > iterationsStart && iterator <= (meanIterations + iterationsStart)) { //First x measures are discarded
                Quaternion_add(&qTemp, &ahrs->q_a, &qTemp);
            }

            if (iterator == (meanIterations + iterationsStart)) {
                Quaternion_scalar_multiplication(&qTemp, (1.0 / (double)meanIterations), &qTemp);
                Quaternion_normalize(&qTemp, &qTemp);
                ahrs->kalman->X_est->data[0][0] = qTemp.w;
                ahrs->kalman->X_est->data[1][0] = qTemp.v[0];
                ahrs->kalman->X_est->data[2][0] = qTemp.v[1];
                ahrs->kalman->X_est->data[3][0] = qTemp.v[2];

                Quaternion_conjugate(&qTemp, &qTemp);
                Quaternion_copy(&qTemp, &ahrs->diff_q);
                Quaternion_set(0.0f, 0.0f, 0.0f, 0.0f, &qTemp);
            }
            ++iterator;
        } else {
            ++state;
        }
        return KALMAN_PROC;
        break;
    }

    case(2):
        ahrs->init->magConst_u = temp_mag;
        ahrs->init->accConst_u = temp_acc;
        state = 0;
        break;

    default:
        state = 0;
        break;

    }
    return KALMAN_OK;
}



#undef KALx
#undef KALz
#undef KALu
