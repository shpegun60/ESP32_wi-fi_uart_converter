#include "imu_worker.h"



IMU_Worker::IMU_Worker()
{
    ahrs_init.dt_init_sec = 0.0;
    ahrs_init.accConst_u = 0.9;
    ahrs_init.magConst_u = 1;

    ahrs_init.gravityConstVect[0] = 9.81;
    ahrs_init.gravityConstVect[1] = 9.81;
    ahrs_init.gravityConstVect[2] = 9.81;

    //gx_offset:  3.85777e-05  gy_offset:  -8.43319e-05  gz_offset:  -0.000154138  ax_offset:  0.414031  ay_offset:  0.7603  az_offset:  -0.174724
    //gx_mean:  0.000221085  gy_mean:  0.000321808  gz_mean:  1.00147e-06  ax_mean:  0.00354765  ay_mean:  0.000808039  az_mean:  0.000183113

    // acc gyro bias
    ahrs_init.accelBiasVect[0] = 0.414031;
    ahrs_init.accelBiasVect[1] = 0.7603;
    ahrs_init.accelBiasVect[2] = -0.174724;

    ahrs_init.gyroBiasVect[0] = 3.85777e-05;
    ahrs_init.gyroBiasVect[1] = -8.43319e-05;
    ahrs_init.gyroBiasVect[2] = -0.000154138;

    //mag biases and conv matrix


//A_inv:
//[[ 0.0237342  -0.00016105 -0.00016956]
// [-0.00016105  0.02376902  0.00174307]
// [-0.00016956  0.00174307  0.02516871]]

//b
//[[54.67591725]
// [83.32482557]
// [ 2.00444644]]

//Total Error: 3.882451

    ahrs_init.magHardIronVect[0] = 54.67591725;
    ahrs_init.magHardIronVect[1] = 83.32482557;
    ahrs_init.magHardIronVect[2] =  2.00444644;

    ahrs_init.magSoftIronInvMat[0][0] =  0.0237342 ;
    ahrs_init.magSoftIronInvMat[1][0] = -0.00016105;
    ahrs_init.magSoftIronInvMat[2][0] = -0.00016956;

    ahrs_init.magSoftIronInvMat[0][1] = -0.00016105;
    ahrs_init.magSoftIronInvMat[1][1] =  0.02376902;
    ahrs_init.magSoftIronInvMat[2][1] =  0.00174307;

    ahrs_init.magSoftIronInvMat[0][2] = -0.00016956;
    ahrs_init.magSoftIronInvMat[1][2] =  0.00174307;
    ahrs_init.magSoftIronInvMat[2][2] =  0.02516871;


    // variances
    ahrs_init.gyroVarianceVect[0] = 0.1;
    ahrs_init.gyroVarianceVect[1] = 0.1;
    ahrs_init.gyroVarianceVect[2] = 0.1;

    ahrs_init.accVarianceVect[0] = 0.015;
    ahrs_init.accVarianceVect[1] = 0.015;
    ahrs_init.accVarianceVect[2] = 0.015;

    ahrs_init.magVarianceVect[0] = 0.0015;
    ahrs_init.magVarianceVect[1] = 0.0015;
    ahrs_init.magVarianceVect[2] = 0.0015;

    ahrs_init.coordinateType = NED;//ENU;
    ahrs_init.Type = Kalman;


    AHRS_Reinit(&ahrs_init, ahrsProceedKalmanComplementaryGyroAcc);

    setPolymorphTrajectory(proceedTrajectorySimple);

}

int IMU_Worker::imuProceed(float time_ms, float a[3], float g[3], float m[3])
{
    // filtration values-----------------------------------------------------
    ahrs_data.dt_sec = (float)((double)((double)time_ms - (double)lastTime) * time_const);
    for(int i = 0; i < 3; ++i) {
        ahrs_data.a[i] = (a[i] - mean_a[i]);
        ahrs_data.g[i] = (g[i] - mean_g[i]);
        ahrs_data.m[i] =  m[i];
    }

    lastTime = time_ms;
//    // ----------------------------------------------------------------------

    switch (procState) {

    case(0):
        for(int i = 0; i < 3; ++i) {
            mean_a[i] = 0.0f;
            mean_g[i] = 0.0f;
            buff_a[i] = 0.0f;
            buff_g[i] = 0.0f;
        }
        iterator = 0;
        ++procState;
        break;

    case(1):
        if(calculateMeans(100, 200)) {
            mean_a[2] = 9.81 + mean_a[2];
            ++procState;
        }
        break;

    case(2): {
        int state = AhrsInitQuat(ahrs_data, 100, 200);
        AhrsProceed(ahrs_data);
        if(state == KALMAN_OK) {
            ++procState;
        }
        break;
    }

    case(3): {
        AhrsProceed(ahrs_data);

        trajectory_data.dt_sec = ahrs_data.dt_sec;
        trajectory_data.a = ahrs_data.a;
        trajectory_data.q = getQuaternion_ptr();

        tragectoryProceed(&trajectory_data);
        break;
    }

    default:
        procState = 0;
        break;
    }
    return 1;
}


int IMU_Worker::calculateMeans(int meanIterations, int iteration_start)
{
    if(iterator < (meanIterations + iteration_start + 1)) {
        if (iterator > iteration_start && iterator <= (meanIterations + iteration_start)) { //First X measures are discarded
            for(int i = 0; i < 3; ++i) {
                buff_a[i] += ahrs_data.a[i];
                buff_g[i] += ahrs_data.g[i];
            }
        }
        if (iterator == (meanIterations + iteration_start)) {
            float invNum = 1.0f / meanIterations;
            for(int i = 0; i < 3; ++i) {
                mean_a[i] = buff_a[i] * invNum;
                mean_g[i] = buff_g[i] * invNum;
            }
        }
        ++iterator;

        return 0;
    }
    return 1;
}


void IMU_Worker::imuRestart()
{
    procState = 0;
    AHRS_reset();
    trajectory_reset();
}
