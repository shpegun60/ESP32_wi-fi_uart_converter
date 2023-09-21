#ifndef __QUATERNION_TEST_H_
#define __QUATERNION_TEST_H_

//   C++ linking for mixed C++/C code
#ifdef __cplusplus
extern "C" {
#endif

void ASSERT_SAME_DOUBLE(const char* description, double x, double y);
void ASSERT_TRUE(const char* description, bool check);
void ASSERT_FALSE(const char* description, bool check);

void testQuaternion_set(void);

void testQuaternion_setIdentity(void);

void testQuaternion_copy(void);

void testQuaternion_conjugate(void);
void testQuaternion_equal(void);

void testQuaternion_norm(void);
void testQuaternion_normalize(void);

void testQuaternion_fromAxisAngle(void);

void testQuaternion_fromXRotation(void);

void testQuaternion_fromYRotation(void);

void testQuaternion_fromZRotation(void);
void testQuaternion_toAxisAngle(void);

void testQuaternion_fromEulerZYX(void);
void testQuaternion_toEulerZYX(void);

void testQuaternion_multiply(void);
void testQuaternion_rotate(void);
void testQuaternion_slerp(void);
int test_quaternion(void);

#ifdef __cplusplus
}
#endif

#endif /* __QUATERNION_TEST_H_ */
