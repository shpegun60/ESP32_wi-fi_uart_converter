#ifndef IMU_WORKER_H_
#define IMU_WORKER_H_

#include "ahrs_top.h"
#include "trajectory_top.h"


class IMU_Worker : public AHRS_Worker, public Trajectory_Worker
{
public:
    IMU_Worker();
    int imuProceed(float time_ms, float a[3], float g[3], float m[3]);
    void imuRestart();

private:
    int calculateMeans(int meanIterations, int iteration_start);

private:

    static constexpr double time_const = (1.0 / 1000.0);

    // AHRS data ---------------------------------
    AHRSInit ahrs_init;
    AHRSinput ahrs_data;

    int procState = 0;
    float lastTime = 0.0f;


    float mean_a[3] = {0.0,};
    float mean_g[3] = {0.0,};
    float buff_a[3] = {0.0,};
    float buff_g[3] = {0.0,};
    int iterator = 0;

    // trajectory data
     TrajTrackInput_t trajectory_data;

};

#endif // IMU_WORKER_H_
