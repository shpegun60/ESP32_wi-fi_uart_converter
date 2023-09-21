/**
 * @file    __AHRS_DATA.h
 * @brief
 * @date
 */

#ifndef __AHRS_DATA_H_
#define __AHRS_DATA_H_

#include "kalman_filter.h"
#include "quaternion.h"

#define AHRS_COMPLEMENTARY_FILTER 0
#define AHRS_MADGWICK_FILTER 1

// MAHONY filter coeficients definition
#define AHRS_MahonyTwoKpDef (2.0f * 0.5f)     // 2 * proportional gain
#define AHRS_MahonyTwoKiDef (2.0f * 0.0f)     // 2 * integral gain

/*
There are two main global reference frames based on the local tangent plane:
    -NED defines the X-, Y-, and Z-axis colinear to the geographical North, East, and Down directions, respectively.
    -ENU defines the X-, Y-, and Z-axis colinear to the geographical East, North, and Up directions, respectively.

    if NED gravity vector G = [0, 0, -1]
    if ENU gravity vector G = [0, 0,  1]


    if NED geomagnetic field vector R = [mN, 0, mD]
    if ENU geomagnetic field vector R = [0, mN, -mD]

    mN -> north magnetic field
    mD -> down magnetic field
    -mD -> up magnetic field(-DOWN == UP)
*/


typedef enum {
    NED,
    ENU
} GlobalCoordinateAhrs;

typedef enum {
    notKalman,
    Kalman
} updateTypeAhrs;

typedef struct {
    float dt_init_sec;
    float accConst_u;               // accelerometer influence factor to gyro quaternion (Madgwick ==> Beta; Mahony ==> 2 * proportional gain (Kp))
    float magConst_u;               // magnetometer influence factor to gyro quaternion (Mahony ==> 2 * integral gain (Ki))
    float gravityConstVect[3];      // gravity field for each axes [x, y, z] (must be positive values)==> [9.81, 9.81, 9.81] = ABS(GRAVITY)

    // biases
    float accelBiasVect[3];         // accelerometer bias vector [x, y, z] ==> (accel + bias)
    float gyroBiasVect[3];          // gyroscope bias vector [x, y, z] ==> (gyro + bias)
    // magnitometer M_calibrated = (H^-1)*(M_measured - S) ==> H - > soft iron matrix, S - >hard iron vector
    float magHardIronVect[3];       // magnitometer hard iron vector
    float magSoftIronInvMat[3][3];  // magnitometer soft iron inverted matrix


    // variances
    float gyroVarianceVect[3];   // gyroscope variance vector [x, y, z]
    float accVarianceVect[3];    // accelerometer variance vector [x, y, z]
    float magVarianceVect[3];    // magnetometer variance vector [x, y, z]

    // coordinate system
    GlobalCoordinateAhrs coordinateType; // coordinate system

    updateTypeAhrs Type; // update type Kalman or complementary filter will used for not malloc some matrics
} AHRSInit;

typedef struct
{
    float dt_sec;
    float g[3]; // [ radians/sec ]   g[0] ==> x; g[1] ==> y; g[2] ==> z;
    float a[3]; // [ m / sec^2 ]  a[0] -> x; a[1] -> y; a[2] -> z; earth gravity ==> z
    float m[3]; // [uT]    m[0] -> x; m[1] -> y; m[2] -> z; (on this time not used)
} AHRSinput;


typedef struct AHRS AHRS_t;
struct AHRS
{
    KalmanFilter* kalman;
    AHRSInit* init;
    float grav[3];              // result calculated gravity vector

    // temp data
    float rotateAxisErrAcc[3];  // error axis between calculated accelerometer data and measured accelerometer
    float rotateAxisErrMag[3];
    float refAxisMag[3];
    float grav_norm[3];         // normalized gravity vector
    Quaternion q_ae;            // accelerometer error rotation quaternion
    Quaternion q_me;            // magnetometer error rotation quaternion
    Quaternion q_a;             // result estimated quaternion from kalman
    Quaternion q_i;             // identity quaternion for filtration

    // other temp data for help`s calculation
    float recipNorm;

    // to update R --------------------------------------------------------
    Mat* J;
    Mat* J_t;
    Mat* Noise_measurment;
    Mat* NOISE_R_RES;


    void (*accDeltaQuaterFinder[2]) (AHRS_t* ahrs, AHRSinput * data); // accelerometer delta quaternion finder for choosed coordinates
    void (*magDeltaQuaterFinder[2]) (AHRS_t* ahrs, AHRSinput * data); // magnetometer delta quaternion finder for choosed coordinates

    Quaternion q_Buf;       // buffer
    Quaternion RES;         // after rotation quaternion
    Quaternion diff_q;      // diff quaternion
};

#endif /* __AHRS_DATA_H */

