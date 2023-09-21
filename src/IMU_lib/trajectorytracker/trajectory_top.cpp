#include "trajectory_top.h"
#include "smart_assert.h"

Trajectory_Worker::Trajectory_Worker(TrajTrack_polymorph_t function)
{
    trajectory = tralectoryTrackerConstructor();
    trajectoryFusion = function;
}

Trajectory_Worker::Trajectory_Worker()
{
    trajectory = tralectoryTrackerConstructor();
}

void Trajectory_Worker::trajectory_reset()
{
    trajectoryTrackerReset(trajectory);
}

int Trajectory_Worker::tragectoryProceed(TrajTrackInput_t *data)
{
    return trajectoryFusion(trajectory, data);
}

int Trajectory_Worker::tragectoryProceed(float dt, float a[3], float g[3], float m[3], Quaternion* q)
{
    TrajTrackInput_t data;

    data.dt_sec = dt;
    data.a = a;
    (void)g;
    (void)m;
    data.q = q;
    return trajectoryFusion(trajectory, &data);
}

int Trajectory_Worker::tragectoryProceed(float dt, float ax, float ay, float az, float gx, float gy, float gz, float mx, float my, float mz, Quaternion *q)
{
    TrajTrackInput_t data;
    float acc[3] = {ax, ay, az};
    float gyro[3] = {gx, gy, gz};
    float mag[3] = {mx, my, mz};


    data.dt_sec = dt;
    data.a = acc;
    (void)gyro;
    (void)mag;
    data.q = q;
    return trajectoryFusion(trajectory, &data);
}

void Trajectory_Worker::setPolymorphTrajectory(TrajTrack_polymorph_t function)
{
    trajectoryFusion = function;
}
