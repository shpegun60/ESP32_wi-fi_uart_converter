/**
 * @file    ahrs.h
 * @brief
 * @date
 */

#ifndef __AHRS_H_
#define __AHRS_H_

#include "ahrs_data.h"

typedef int (*AHRS_polymorph_t) (AHRS_t*, AHRSinput *);


AHRS_t* ahrsCreate(AHRSInit* init);

void ahrsReset(AHRS_t* ahrs);


/*
 * **********************************************************************************************************************************************************************
 * Complementary filter based algorithms polymorph`s
 * **********************************************************************************************************************************************************************
 */
int ahrsProceedKalmanComplementaryGyroAccMag(AHRS_t* ahrs, AHRSinput * data); // original kalman fusion with magnetometer
int ahrsProceedKalmanComplementaryGyroAcc(AHRS_t* ahrs, AHRSinput* data); // kalman filter based on complementary filter without magnetometer

int ahrsComplementaryFilterGyroAccMag(AHRS_t* ahrs, AHRSinput * data);
int ahrsComplementaryFilterGyroAcc(AHRS_t* ahrs, AHRSinput * data);
int ahrsOnlyGyro(AHRS_t* ahrs, AHRSinput * data);
int ahrsOnlyAcc(AHRS_t* ahrs, AHRSinput * data);
int ahrsOnlyMag(AHRS_t* ahrs, AHRSinput * data);

/*
 * **********************************************************************************************************************************************************************
 * Madgwick filter (from adafruit AHRS) polymorph`s
 * **********************************************************************************************************************************************************************
 */
int ahrsProceedKalmanMadgwickGyroAcc(AHRS_t* ahrs, AHRSinput * data); // madgwick + kalman fusion

int ahrsMadgwickGyroAccMag(AHRS_t* ahrs, AHRSinput * data);
int ahrsMadgwickGyroAcc(AHRS_t* ahrs, AHRSinput * data);

/*
 * **********************************************************************************************************************************************************************
 * Mahony filter (from adafruit AHRS) polymorph`s
 * **********************************************************************************************************************************************************************
 */
int ahrsMahonyGyroAcc(AHRS_t* ahrs, AHRSinput * data);
int ahrsMahonyGyroAccMag(AHRS_t* ahrs, AHRSinput * data);


// starting quaternion finder ------------------------------------------
int ahrsInitStartQuat(AHRS_t* ahrs, AHRSinput * data, int meanIterations, int iterationsStart, AHRS_polymorph_t proceedFoo); // read start quaternion


#endif /* __AHRS_H_ */

