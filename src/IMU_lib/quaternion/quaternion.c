/**
 * @file    quaternion.c
 * @brief   A basic quaternion library written in C
 * @date    2019-11-28
 */
#include "quaternion.h"
#include "smart_assert.h"
#include "fastmath.h"

#include <stdlib.h>
#include <assert.h>
#include <math.h>

//-------------------------------------------------------------------------------------------
// Fast inverse square-root

#ifdef FAST_CALCULATE_INV_SQRT
    static_assert (sizeof(long) == 4, "Quaternion: need rewrite platform depend invSqrt and fastSqrt function or commit FAST_CALCULATE_INV_SQRT because type long != 4");
#endif

//-------------------------------------------------------------------------------------------


void Quaternion_set(float w, float v1, float v2, float v3, Quaternion* output)
{
    M_Assert_Break((output == NULL), "Quaternion_set: NULL", return);
    output->w = w;
    output->v[0] = v1;
    output->v[1] = v2;
    output->v[2] = v3;
}

void Quaternion_setIdentity(Quaternion* q)
{
    M_Assert_Break((q == NULL), "Quaternion_setIdentity: NULL", return);
    Quaternion_set(1.0f, 0.0f, 0.0f, 0.0f, q);
}

void Quaternion_copy(Quaternion* q, Quaternion* output)
{
    M_Assert_Break((q == NULL || output == NULL), "Quaternion_setIdentity: NULL", return);
    Quaternion_set(q->w, q->v[0], q->v[1], q->v[2], output);
}

bool Quaternion_equal(Quaternion* q1, Quaternion* q2)
{
    M_Assert_Break((q1 == NULL || q2 == NULL), "Quaternion_equal: NULL", return 0);
    bool equalW  = fabs(q1->w - q2->w) <= QUATERNION_EPS;
    bool equalV0 = fabs(q1->v[0] - q2->v[0]) <= QUATERNION_EPS;
    bool equalV1 = fabs(q1->v[1] - q2->v[1]) <= QUATERNION_EPS;
    bool equalV2 = fabs(q1->v[2] - q2->v[2]) <= QUATERNION_EPS;
    return equalW && equalV0 && equalV1 && equalV2;
}

void Quaternion_fprint(FILE* file, Quaternion* q)
{
    M_Assert_Break((file == NULL || q == NULL), "Quaternion_fprint: NULL", return);
    fprintf(file, "(%.3f, %.3f, %.3f, %.3f)",
            q->w, q->v[0], q->v[1], q->v[2]);
}


void Quaternion_fromAxisAngle(float axis[3], float angle, Quaternion* output)
{
    M_Assert_Break((output == NULL), "Quaternion_fromAxisAngle: NULL", return);
    // Formula from http://www.euclideanspace.com/maths/geometry/rotations/conversions/angleToQuaternion/
    output->w = cos(angle * 0.5f);
    float c = sin(angle * 0.5f);
    output->v[0] = c * axis[0];
    output->v[1] = c * axis[1];
    output->v[2] = c * axis[2];
}


float Quaternion_toAxisAngle(Quaternion* q, float output[3])
{
    M_Assert_Break((output == NULL || q == NULL), "Quaternion_toAxisAngle: NULL", return 0.0);
    // Formula from http://www.euclideanspace.com/maths/geometry/rotations/conversions/quaternionToAngle/
    float angle = 2.0f * acos(q->w);

    if(q->w < 1.0f) {
        // Calculate the axis
#ifdef FAST_CALCULATE_INV_SQRT

        float multiplier = fastinvsqrtf(1.0f - q->w * q->w);
#else
        float multiplier = 1.0/sqrt(1.0 - q->w * q->w);
#endif
        output[0] = q->v[0] * multiplier;
        output[1] = q->v[1] * multiplier;
        output[2] = q->v[2] * multiplier;
    } else {
        // Arbitrary normalized axis
        output[0] = 1.0f;
        output[1] = 0.0f;
        output[2] = 0.0f;
    }
    return angle;
}

void Quaternion_betweenAngle(Quaternion* a, Quaternion* b, float* angle)
{
    M_Assert_Break((a == NULL || b == NULL || angle == NULL), "Quaternion_betweenAngle: NULL", return);
    (*angle) = acos((a->w * b->w) + (a->v[0] * b->v[0]) + (a->v[1] * b->v[1]) + (a->v[2] * b->v[2]));
}


void Quaternion_fromXRotation(float angle, Quaternion* output)
{
    M_Assert_Break((output == NULL ), "Quaternion_fromXRotation: NULL", return);
    float axis[3] = {1.0f, 0.0f, 0.0f};
    Quaternion_fromAxisAngle(axis, angle, output);
}

void Quaternion_fromYRotation(float angle, Quaternion* output)
{
    M_Assert_Break((output == NULL ), "Quaternion_fromYRotation: NULL", return);
    float axis[3] = {0.0f, 1.0f, 0.0f};
    Quaternion_fromAxisAngle(axis, angle, output);
}

void Quaternion_fromZRotation(float angle, Quaternion* output)
{
    M_Assert_Break((output == NULL ), "Quaternion_fromZRotation: NULL", return);
    float axis[3] = {0.0f, 0.0f, 1.0f};
    Quaternion_fromAxisAngle(axis, angle, output);
}

void Quaternion_fromEulerZYX(float eulerZYX[3], Quaternion* output)
{
   M_Assert_Break((output == NULL || eulerZYX == NULL), "Quaternion_fromEulerZYX: NULL", return);
    // Based on https://en.wikipedia.org/wiki/Conversion_between_quaternions_and_Euler_angles
    float cy = cos(eulerZYX[2] * 0.5f);
    float sy = sin(eulerZYX[2] * 0.5f);
    float cr = cos(eulerZYX[0] * 0.5f);
    float sr = sin(eulerZYX[0] * 0.5f);
    float cp = cos(eulerZYX[1] * 0.5f);
    float sp = sin(eulerZYX[1] * 0.5f);

    output->w = cy * cr * cp + sy * sr * sp;
    output->v[0] = cy * sr * cp - sy * cr * sp;
    output->v[1] = cy * cr * sp + sy * sr * cp;
    output->v[2] = sy * cr * cp - cy * sr * sp;
}

void Quaternion_toEulerZYX(Quaternion* q, float output[3])
{
    M_Assert_Break((output == NULL || q == NULL), "Quaternion_toEulerZYX: NULL", return);

    // Roll (x-axis rotation)
    float sinr_cosp = +2.0f * (q->w * q->v[0] + q->v[1] * q->v[2]);
    float cosr_cosp = +1.0f - 2.0f * (q->v[0] * q->v[0] + q->v[1] * q->v[1]);
    output[0] = atan2(sinr_cosp, cosr_cosp);

    // Pitch (y-axis rotation)
    float sinp = +2.0f * (q->w * q->v[1] - q->v[2] * q->v[0]);
    if (fabs(sinp) >= 1.0f) {
        output[1] = copysign((M_PI * 0.5f), sinp); // use 90 degrees if out of range
    } else {
        output[1] = asin(sinp);
    }

    // Yaw (z-axis rotation)
    float siny_cosp = +2.0f * (q->w * q->v[2] + q->v[0] * q->v[1]);
    float cosy_cosp = +1.0f - 2.0f * (q->v[1] * q->v[1] + q->v[2] * q->v[2]);
    output[2] = atan2(siny_cosp, cosy_cosp);
}

void Quaternion_conjugate(Quaternion* q, Quaternion* output)
{
    M_Assert_Break((output == NULL || q == NULL), "Quaternion_conjugate: NULL", return);
    output->w = q->w;
    output->v[0] = -q->v[0];
    output->v[1] = -q->v[1];
    output->v[2] = -q->v[2];
}

float Quaternion_norm(Quaternion* q)
{
    M_Assert_Break((q == NULL), "Quaternion_norm: NULL", return 0.0f);
#ifdef FAST_CALCULATE_INV_SQRT
    return fastSqrtf_math(q->w*q->w + q->v[0]*q->v[0] + q->v[1]*q->v[1] + q->v[2]*q->v[2]);
#else
    return sqrt(q->w*q->w + q->v[0]*q->v[0] + q->v[1]*q->v[1] + q->v[2]*q->v[2]);
#endif
}

void Quaternion_normalize(Quaternion* q, Quaternion* output)
{
    M_Assert_Break((output == NULL || q == NULL), "Quaternion_normalize: NULL", return);
#ifdef FAST_CALCULATE_INV_SQRT
    float inverse_len = fastinvsqrtf(q->w*q->w + q->v[0]*q->v[0] + q->v[1]*q->v[1] + q->v[2]*q->v[2]);
#else
    float inverse_len = 1.0 / Quaternion_norm(q);
#endif

    output->w    = q->w    * inverse_len;
    output->v[0] = q->v[0] * inverse_len;
    output->v[1] = q->v[1] * inverse_len;
    output->v[2] = q->v[2] * inverse_len;
}

void Quaternion_normalize_vect(float* q0, float* q1, float* q2, float* q3)
{
    M_Assert_Break((q0 == NULL || q1 == NULL || q2 == NULL || q3 == NULL), "Quaternion_normalize_vect: NULL", return);

    float q0_quad = (*q0) * (*q0);
    float q1_quad = (*q1) * (*q1);
    float q2_quad = (*q2) * (*q2);
    float q3_quad = (*q3) * (*q3);
#ifdef FAST_CALCULATE_INV_SQRT
    float inverse_len = fastinvsqrtf(q0_quad + q1_quad + q2_quad + q3_quad);
#else
    float inverse_len = 1.0 / sqrt(q0_quad + q1_quad + q2_quad + q3_quad);
#endif

    (*q0) = (*q0) * inverse_len;
    (*q1) = (*q1) * inverse_len;
    (*q2) = (*q2) * inverse_len;
    (*q3) = (*q3) * inverse_len;
}

void Quaternion_multiply(Quaternion* q1, Quaternion* q2, Quaternion* output)
{
    M_Assert_Break((q1 == NULL || q2 == NULL || output == NULL), "Quaternion_multiply: NULL", return);
    Quaternion result;

    /*
    Formula from http://www.euclideanspace.com/maths/algebra/realNormedAlgebra/quaternions/arithmetic/index.htm
             a*e - b*f - c*g - d*h
        + i (b*e + a*f + c*h- d*g)
        + j (a*g - b*h + c*e + d*f)
        + k (a*h + b*g - c*f + d*e)
    */
    result.w =    q1->w   *q2->w    - q1->v[0]*q2->v[0] - q1->v[1]*q2->v[1] - q1->v[2]*q2->v[2];
    result.v[0] = q1->v[0]*q2->w    + q1->w   *q2->v[0] + q1->v[1]*q2->v[2] - q1->v[2]*q2->v[1];
    result.v[1] = q1->w   *q2->v[1] - q1->v[0]*q2->v[2] + q1->v[1]*q2->w    + q1->v[2]*q2->v[0];
    result.v[2] = q1->w   *q2->v[2] + q1->v[0]*q2->v[1] - q1->v[1]*q2->v[0] + q1->v[2]*q2->w   ;

    *output = result;
}

void Quaternion_multiply_to_arrayLN(Quaternion* q1, Quaternion* q2, float** output)
{
     M_Assert_Break((q1 == NULL || q2 == NULL || output == NULL), "Quaternion_multiply_to_arrayLN: NULL", return);

    /*
    Formula from http://www.euclideanspace.com/maths/algebra/realNormedAlgebra/quaternions/arithmetic/index.htm
             a*e - b*f - c*g - d*h
        + i (b*e + a*f + c*h- d*g)
        + j (a*g - b*h + c*e + d*f)
        + k (a*h + b*g - c*f + d*e)
    */
    output[0][0] = q1->w   *q2->w    - q1->v[0]*q2->v[0] - q1->v[1]*q2->v[1] - q1->v[2]*q2->v[2];
    output[1][0] = q1->v[0]*q2->w    + q1->w   *q2->v[0] + q1->v[1]*q2->v[2] - q1->v[2]*q2->v[1];
    output[2][0] = q1->w   *q2->v[1] - q1->v[0]*q2->v[2] + q1->v[1]*q2->w    + q1->v[2]*q2->v[0];
    output[3][0] = q1->w   *q2->v[2] + q1->v[0]*q2->v[1] - q1->v[1]*q2->v[0] + q1->v[2]*q2->w   ;
}

void Quaternion_scalar_multiplication(Quaternion* q, float s, Quaternion* Dest)
{
    M_Assert_Break((q == NULL || Dest == NULL), "Quaternion_scalar_multiplication: NULL", return);
    Dest->w     = q->w    * s;
    Dest->v[0]  = q->v[0] * s;
    Dest->v[1]  = q->v[1] * s;
    Dest->v[2]  = q->v[2] * s;
}

void Quaternion_add(Quaternion *a, Quaternion *b, Quaternion *Dest)
{
    M_Assert_Break((a == NULL || b == NULL || Dest == NULL), "Quaternion_add: NULL", return);
    Dest->w     = a->w    + b->w   ;
    Dest->v[0]  = a->v[0] + b->v[0];
    Dest->v[1]  = a->v[1] + b->v[1];
    Dest->v[2]  = a->v[2] + b->v[2];
}

void Quaternion_rotate(Quaternion* q, float v[3], float output[3])
{
    M_Assert_Break((output == NULL || v == NULL || q == NULL), "Quaternion_rotate: NULL", return);
    float result[3];

    float ww = q->w * q->w;
    float xx = q->v[0] * q->v[0];
    float yy = q->v[1] * q->v[1];
    float zz = q->v[2] * q->v[2];
    float wx = q->w * q->v[0];
    float wy = q->w * q->v[1];
    float wz = q->w * q->v[2];
    float xy = q->v[0] * q->v[1];
    float xz = q->v[0] * q->v[2];
    float yz = q->v[1] * q->v[2];

    // Formula from http://www.euclideanspace.com/maths/algebra/realNormedAlgebra/quaternions/transforms/index.htm
    // p2.x = w*w*p1.x + 2*y*w*p1.z - 2*z*w*p1.y + x*x*p1.x + 2*y*x*p1.y + 2*z*x*p1.z - z*z*p1.x - y*y*p1.x;
    // p2.y = 2*x*y*p1.x + y*y*p1.y + 2*z*y*p1.z + 2*w*z*p1.x - z*z*p1.y + w*w*p1.y - 2*x*w*p1.z - x*x*p1.y;
    // p2.z = 2*x*z*p1.x + 2*y*z*p1.y + z*z*p1.z - 2*w*y*p1.x - y*y*p1.z + 2*w*x*p1.y - x*x*p1.z + w*w*p1.z;

    result[0] = ww*v[0] + 2.0f*wy*v[2] - 2.0f*wz*v[1] +
            xx*v[0] + 2.0f*xy*v[1] + 2.0f*xz*v[2] -
            zz*v[0] - yy*v[0];
    result[1] = 2.0f*xy*v[0] + yy*v[1] + 2.0f*yz*v[2] +
            2.0f*wz*v[0] - zz*v[1] + ww*v[1] -
            2.0f*wx*v[2] - xx*v[1];
    result[2] = 2.0f*xz*v[0] + 2.0f*yz*v[1] + zz*v[2] -
            2.0f*wy*v[0] - yy*v[2] + 2.0f*wx*v[1] -
            xx*v[2] + ww*v[2];

    // Copy result to output
    output[0] = result[0];
    output[1] = result[1];
    output[2] = result[2];
}

void Quaternion_slerp(Quaternion* q1, Quaternion* q2, float t, Quaternion* output)
{
    M_Assert_Break((q1 == NULL || q2 == NULL || output == NULL), "Quaternion_slerp: NULL", return);
    Quaternion result;

    // Based on http://www.euclideanspace.com/maths/algebra/realNormedAlgebra/quaternions/slerp/index.htm
    float cosHalfTheta = q1->w*q2->w + q1->v[0]*q2->v[0] + q1->v[1]*q2->v[1] + q1->v[2]*q2->v[2];

    // if q1=q2 or qa=-q2 then theta = 0 and we can return qa
    if (fabs(cosHalfTheta) >= 1.0f) {
        Quaternion_copy(q1, output);
        return;
    }

    float halfTheta = acos(cosHalfTheta);
#ifdef FAST_CALCULATE_INV_SQRT
    float sinHalfTheta = fastinvsqrtf(1.0f - cosHalfTheta*cosHalfTheta);
#else
    float sinHalfTheta = 1.0 / sqrt(1.0 - cosHalfTheta*cosHalfTheta);
#endif
    // If theta = 180 degrees then result is not fully defined
    // We could rotate around any axis normal to q1 or q2
    if (fabs(sinHalfTheta) > QUATERNION_EPS_INV) {
        result.w = (q1->w * 0.5f + q2->w * 0.5f);
        result.v[0] = (q1->v[0] * 0.5f + q2->v[0] * 0.5f);
        result.v[1] = (q1->v[1] * 0.5f + q2->v[1] * 0.5f);
        result.v[2] = (q1->v[2] * 0.5f + q2->v[2] * 0.5f);
        *output = result;
        return;
    }

    // Calculate Quaternion
    float ratioA = sin((1.0f - t) * halfTheta) * sinHalfTheta;
    float ratioB = sin(t * halfTheta) * sinHalfTheta;

    result.w    = (q1->w    * ratioA    + q2->w    * ratioB);
    result.v[0] = (q1->v[0] * ratioA    + q2->v[0] * ratioB);
    result.v[1] = (q1->v[1] * ratioA    + q2->v[1] * ratioB);
    result.v[2] = (q1->v[2] * ratioA    + q2->v[2] * ratioB);

    *output = result;
}


void Quaternion_lerp(Quaternion* q1, Quaternion* q2, float t, Quaternion* output)
{
    M_Assert_Break((q1 == NULL || q2 == NULL || output == NULL), "Quaternion_lerp: NULL", return);
    Quaternion result;

    // Calculate Quaternion
    float ratioA = 1.0 - t;

    result.w    = (q1->w    * ratioA     + q2->w    * t);
    result.v[0] = (q1->v[0] * ratioA     + q2->v[0] * t);
    result.v[1] = (q1->v[1] * ratioA     + q2->v[1] * t);
    result.v[2] = (q1->v[2] * ratioA     + q2->v[2] * t);

#ifdef FAST_CALCULATE_INV_SQRT
    ratioA = fastinvsqrtf(result.w*result.w + result.v[0]*result.v[0] + result.v[1]*result.v[1] + result.v[2]*result.v[2]);
#else
    ratioA = 1.0 / sqrt(result.w*result.w + result.v[0]*result.v[0] + result.v[1]*result.v[1] + result.v[2]*result.v[2]);
#endif
    result.w    *= ratioA;
    result.v[0] *= ratioA;
    result.v[1] *= ratioA;
    result.v[2] *= ratioA;

    *output = result;
}

