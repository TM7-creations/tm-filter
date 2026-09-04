#ifndef TM_EXAMPLE_MADGWICK_H
#define TM_EXAMPLE_MADGWICK_H
#ifdef __cplusplus
extern "C" {
#endif
/* Comparison implementation of Madgwick's IMU equations, not an official
 * distribution. Same [w,x,y,z], body->world and SI conventions as TM.
 * beta >= 0. Returns 1 on success; invalid gyro/dt/beta/q leaves q unchanged. */
int madgwick_update_imu(float q[4], float gx, float gy, float gz,
                        float ax, float ay, float az, float dt, float beta);
#ifdef __cplusplus
}
#endif
#endif
