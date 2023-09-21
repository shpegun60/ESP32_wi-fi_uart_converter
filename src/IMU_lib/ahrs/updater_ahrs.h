/**
 * @file    __UPDATER_AHRS.h
 * @brief
 * @date
 */

#ifndef __UPDATER_AHRS_H_
#define __UPDATER_AHRS_H_

void computeGravVectorAHRS(const float q0, const float q1, const float q2, const float q3, float* const grav, const float* const constant);
void magOffsetDeleteAHRS(float* iBpx, float* iBpy, float* iBpz, float iV[3], float InvW[3][3]);

// helper for update matrix from maple (predict)
void updateSysAHRS(float * g, float dt, float** cg, float** cg0); // cg ==> F, cg0 ==> F_T, cg1 ==> G

// predict noise system
void updatePredictionNoiseAHRS(float gx, float gy, float gz, float q0, float q1, float q2,float q3, float dt, float** cg); // cg ==> Q

// measurments noise update
void updateMeasurmentJacobianMagAHRS_NED(float mag[3], float magRef[3], float acc[3], float q0, float q1, float q2,float q3, float** cg, float** cgt); // cg ==> J, cgt ==> J_T
void updateMeasurmentJacobianAHRS_NED(float acc[3], float q0, float q1, float q2,float q3, float** cg, float** cgt); // cg ==> J, cgt ==> J_T

void updateMeasurmentJacobianMagAHRS_ENU(float mag[3], float magRef[3], float acc[3], float q0, float q1, float q2,float q3, float** cg, float** cgt); // cg ==> J, cgt ==> J_T
void updateMeasurmentJacobianAHRS_ENU(float acc[3], float q0, float q1, float q2,float q3, float** cg, float** cgt); // cg ==> J, cgt ==> J_T

#endif /* __UPDATER_AHRS_H_ */


