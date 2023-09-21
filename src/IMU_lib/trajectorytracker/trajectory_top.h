#ifndef TRAJECTORY_TOP_H_
#define TRAJECTORY_TOP_H_

extern "C" {
    #include "trajectorytracker.h"
}


class Trajectory_Worker // this class is wrapper to C trajectorytracker functions
{
public:
    Trajectory_Worker(TrajTrack_polymorph_t function);
    Trajectory_Worker();

    void trajectory_reset();


    // function for update
    int tragectoryProceed(TrajTrackInput_t *data);
    int tragectoryProceed(float dt, float a[3], float g[3], float m[3], Quaternion* q);
    int tragectoryProceed(float dt, float ax, float ay, float az, float gx, float gy, float gz, float mx, float my, float mz, Quaternion* q);

    // getters/ setters -------------------------------------------------------------------------
    void getPositions(float *posX, float *posY, float *posZ)
    {
        *posX = trajectory->posX[1];
        *posY = trajectory->posY[1];
        *posZ = 0.0;/*this->posZ[1];*/
    }
    void getVelocities(float *velX, float *velY, float *velZ)
    {
        *velX = trajectory->velocityX[1];
        *velY = trajectory->velocityY[1];
        *velZ = 0.0;/*this->velocityZ[1];*/
    }
    void getAccelGlobal(float *accG_X, float *accG_Y, float *accG_Z)
    {
        *accG_X = trajectory->a_dataGlobal[0];
        *accG_Y = trajectory->a_dataGlobal[1];
        *accG_Z = trajectory->a_dataGlobal[2];
    }
    void getAccelFilteredGlobal(float *accF_X, float *accF_Y, float *accF_Z)
    {
        *accF_X = trajectory->accelX[1];
        *accF_Y = trajectory->accelY[1];
        *accF_Z = 0.0;//this->accelZ[1];
    }


    void setPolymorphTrajectory(TrajTrack_polymorph_t);
private:
    TrajTrack_polymorph_t trajectoryFusion = (TrajTrack_polymorph_t)NULL;
    TrajTrack_t* trajectory;
};

#endif // TRAJECTORY_TOP_H_
