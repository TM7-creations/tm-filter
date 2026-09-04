#ifndef TM_FILTER_H
#define TM_FILTER_H

#ifdef __cplusplus
extern "C" {
#endif

#define TM_GYRO_METHOD_FAST 0
#define TM_GYRO_METHOD_EXACT 1
#ifndef TM_GYRO_METHOD
#define TM_GYRO_METHOD TM_GYRO_METHOD_FAST
#endif
#if TM_GYRO_METHOD != TM_GYRO_METHOD_FAST && TM_GYRO_METHOD != TM_GYRO_METHOD_EXACT
#error "TM_GYRO_METHOD must be FAST (0) or EXACT (1)"
#endif
#define TM_GRAVITY 9.80665f
#define TM_DEG_TO_RAD 0.01745329251994329577f

typedef struct {
    int gyro_method;                 /* FAST or EXACT; may change between updates. */
    float fusion_gain;               /* Weight = clamp(gain * angle_rad, 0, 1). */
    float gravity;                   /* Expected acceleration magnitude, m/s^2. */
    float accel_tolerance;           /* Allowed absolute magnitude error, m/s^2. */
    float max_accel_direction_rate;  /* rad/s; 0 disables the direction gate. */
} tm_filter_config;

typedef struct {
    float q[4];                     /* Hamilton quaternion [w,x,y,z], body -> world. */
    tm_filter_config config;
    float previous_accel[3];        /* Internal history, in m/s^2. */
    int has_previous_accel;
} tm_filter;

/* All pointers must be non-null and point to arrays of the declared size.
 * No allocation or global mutable state. Use one instance per sensor. */
void tm_filter_init(tm_filter *filter);
/* Reset orientation/history while keeping the configuration. */
void tm_filter_reset(tm_filter *filter);

/* Gyroscope in body-frame rad/s, dt in seconds. Return 1 on success, 0 on
 * invalid input; invalid input leaves q unchanged. Output is normalized. */
int tm_gyro_update_exact(float q[4], float gx, float gy, float gz, float dt);
int tm_gyro_update_fast(float q[4], float gx, float gy, float gz, float dt);

/* Closest unit quaternion satisfying R(qa)^T * world_Z = normalized(accel).
 * Accel is specific force: at rest with aligned frames it points to +Z.
 * Returns 0 for invalid input, leaving qa unchanged. q and qa may alias.
 * At the antipodal ambiguity, returns a deterministic minimizer. */
int tm_accel_quaternion(const float q[4], const float accel[3], float qa[4]);

/* Shortest-sign normalized linear interpolation. Weight must be in [0,1]. */
int tm_quaternion_blend(float q[4], const float target[4], float weight);

/* SI inputs, dt > 0. Returns -1 for invalid gyro/dt/quaternion/config (no
 * state change), 0 for gyro only, 1 when an accel correction was applied.
 * Invalid/zero acceleration skips correction and clears accel history.
 * The first valid accel sample uses only the magnitude gate. */
int tm_filter_update(tm_filter *filter, float gx, float gy, float gz,
                     float ax, float ay, float az, float dt);

#ifdef __cplusplus
}
#endif
#endif
