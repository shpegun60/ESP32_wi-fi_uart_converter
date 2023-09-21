#ifndef AHRS_TOP_H_
#define AHRS_TOP_H_

extern "C" {
    #include "ahrs.h"
}



class AHRS_Worker // this class is wrapper to C ahrs functions
{
public:
    AHRS_Worker(AHRSInit* init, AHRS_polymorph_t proceedFoo);
    AHRS_Worker();

    int AHRS_Reinit(AHRSInit* init, AHRS_polymorph_t proceedFoo);
    void AHRS_reset();

    // function for update
    int AhrsProceed(AHRSinput * data);
    int AhrsProceed(AHRSinput data);
    int AhrsProceed(float dt, float a[3], float g[3], float m[3]);
    int AhrsProceed(float dt, float ax, float ay, float az, float gx, float gy, float gz, float mx, float my, float mz);

    //function for starting calculate quaternion
    int AhrsInitQuat(AHRSinput *data, int meanIterations, int iterationsStart);
    int AhrsInitQuat(AHRSinput data, int meanIterations, int iterationsStart);
    int AhrsInitQuat(float dt, float a[3], float g[3], float m[3], int meanIterations, int iterationsStart);
    int AhrsInitQuat(float dt, float ax, float ay, float az, float gx, float gy, float gz, float mx, float my, float mz, int meanIterations, int iterationsStart);


    // fetters / setters ------------------------------------------
    Mat * getResultData();
    float *getGravityData();
    Quaternion getQuaternion();
    Quaternion* getQuaternion_ptr();

    void printKalmanTop();

    float *getGravity();
    Mat *getLinearAcceleration();

    Mat *getSystem();

    void setPolymorphAHRS(AHRS_polymorph_t);
private:
    AHRS_polymorph_t ahrsFusion = (AHRS_polymorph_t)NULL;
    AHRS_t* ahrs = NULL;
};

#endif // AHRS_TOP_H_
