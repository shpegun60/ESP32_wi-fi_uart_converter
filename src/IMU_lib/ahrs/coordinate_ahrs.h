/**
 * @file    Coordinate_ahrs.h
 * @brief
 * @date
 */

#ifndef __COORDINATE_AHRS_H_
#define __COORDINATE_AHRS_H_

#include "ahrs_data.h"

/*
 * **************************************************************************************************************************
 * Complementary filter coordinate transition functions
 * **************************************************************************************************************************
 */

// accelerometer delta quaternion finder for all coordinates --------------------------------------------
void ComplementaryAccDeltaQuaterFinder_NED(AHRS_t* ahrs, AHRSinput * data);
void ComplementaryAccDeltaQuaterFinder_ENU(AHRS_t* ahrs, AHRSinput * data);

// magnetometer delta quaternion finder for all coordinates --------------------------------------------
void ComplementaryMagDeltaQuaterFinder_NED(AHRS_t* ahrs, AHRSinput * data);
void ComplementaryMagDeltaQuaterFinder_ENU(AHRS_t* ahrs, AHRSinput * data);

/*
 * **************************************************************************************************************************
 * Madgwick filter coordinate transition functions
 * **************************************************************************************************************************
 */

// accelerometer delta quaternion finder for all coordinates --------------------------------------------
void MadgwickAccDeltaQuaterFinder_NED(AHRS_t* ahrs, AHRSinput * data);
void MadgwickAccDeltaQuaterFinder_ENU(AHRS_t* ahrs, AHRSinput * data);

// magnetometer delta quaternion finder for all coordinates --------------------------------------------
void MadgwickAccMagDeltaQuaterFinder_NED(AHRS_t* ahrs, AHRSinput * data);
void MadgwickAccMagDeltaQuaterFinder_ENU(AHRS_t* ahrs, AHRSinput * data);

#endif /* __COORDINATE_AHRS_H_ */


