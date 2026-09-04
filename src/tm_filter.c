#include "tm_filter.h"
#include <math.h>
#include <string.h>

static float clampf(float x, float lo, float hi) {
    return fminf(hi, fmaxf(lo, x));
}

/* Scale first so normalizing a finite quaternion cannot overflow its norm. */
static int normalize(float q[4]) {
    float scale = 0.0f, sum = 0.0f;
    for (int i = 0; i < 4; ++i) {
        if (!isfinite(q[i])) return 0;
        scale = fmaxf(scale, fabsf(q[i]));
    }
    if (scale == 0.0f) return 0;
    for (int i = 0; i < 4; ++i) { q[i] /= scale; sum += q[i]*q[i]; }
    const float norm = sqrtf(sum);
    for (int i = 0; i < 4; ++i) q[i] /= norm;
    return 1;
}

static float dot4(const float a[4], const float b[4]) {
    return a[0]*b[0] + a[1]*b[1] + a[2]*b[2] + a[3]*b[3];
}

/* Right multiplication by the pure body-rate quaternion: q * [0,gx,gy,gz]. */
static int integrate(float q[4], float gx, float gy, float gz, float dt, int exact) {
    float p[4], out[4];
    if (!isfinite(dt) || dt <= 0.0f || !isfinite(gx) ||
        !isfinite(gy) || !isfinite(gz)) return 0;
    memcpy(p, q, sizeof(p));
    if (!normalize(p)) return 0;
    float c = 1.0f, s = 0.5f * dt;
    const float wn = exact ? hypotf(hypotf(gx, gy), gz) : 0.0f;
    if (!isfinite(wn)) return 0;
    if (exact && wn > 0.0f) {
        const float half_angle = (0.5f * wn) * dt;
        if (!isfinite(half_angle)) return 0;
        c = cosf(half_angle);
        /* sin(x)/x has a finite limit at zero; no small-rate dead band. */
        if (fabsf(half_angle) < 1e-4f)
            s *= 1.0f - half_angle*half_angle / 6.0f;
        else
            s = sinf(half_angle) / wn;
    }
    out[0] = c*p[0] + s*(-p[1]*gx - p[2]*gy - p[3]*gz);
    out[1] = c*p[1] + s*( p[0]*gx + p[2]*gz - p[3]*gy);
    out[2] = c*p[2] + s*( p[0]*gy - p[1]*gz + p[3]*gx);
    out[3] = c*p[3] + s*( p[0]*gz + p[1]*gy - p[2]*gx);
    if (!normalize(out)) return 0;
    memcpy(q, out, sizeof(out));
    return 1;
}

int tm_gyro_update_exact(float q[4], float gx, float gy, float gz, float dt) {
    return integrate(q, gx, gy, gz, dt, 1);
}
int tm_gyro_update_fast(float q[4], float gx, float gy, float gz, float dt) {
    return integrate(q, gx, gy, gz, dt, 0);
}

int tm_accel_quaternion(const float q[4], const float accel[3], float qa[4]) {
    float p[4], out[4], b[4];
    memcpy(p, q, sizeof(p));
    if (!normalize(p)) return 0;
    for (int i = 0; i < 3; ++i) if (!isfinite(accel[i])) return 0;
    const float norm = hypotf(hypotf(accel[0], accel[1]), accel[2]);
    if (!isfinite(norm) || norm == 0.0f) return 0;
    const float ax = accel[0]/norm, ay = accel[1]/norm;
    const float az = clampf(accel[2]/norm, -1.0f, 1.0f);
    const float radial = hypotf(ax, ay);
    /* b rotates the measured body direction onto world +Z.
     * Separate hemispheres avoid cancellation around az = -1. */
    if (radial == 0.0f) {
        b[0] = az >= 0.0f ? 1.0f : 0.0f;
        b[1] = az >= 0.0f ? 0.0f : 1.0f;
        b[2] = b[3] = 0.0f;
    } else {
        float sine, cosine;
        if (az >= 0.0f) {
            cosine = sqrtf(0.5f*(1.0f + az));
            sine = radial/(2.0f*cosine);
        } else {
            sine = sqrtf(0.5f*(1.0f - az));
            cosine = radial/(2.0f*sine);
        }
        b[0] = cosine; b[1] = sine*(ay/radial);
        b[2] = -sine*(ax/radial); b[3] = 0.0f;
    }
    /* All compatible quaternions are b*cos(t) + h*sin(t),
     * where h = [0,0,0,1] (Hamilton product) b. Project onto that plane. */
    const float h[4] = {-b[3], -b[2], b[1], b[0]};
    const float u = dot4(p, b), v = dot4(p, h);
    const float length = hypotf(u, v);
    if (length == 0.0f) {
        memcpy(out, b, sizeof(out)); /* Every feasible attitude is equally near. */
    } else {
        for (int i = 0; i < 4; ++i) out[i] = (u/length)*b[i] + (v/length)*h[i];
    }
    if (!normalize(out)) return 0;
    memcpy(qa, out, sizeof(out));
    return 1;
}

int tm_quaternion_blend(float q[4], const float target[4], float weight) {
    float p[4], t[4], out[4];
    if (!isfinite(weight) || weight < 0.0f || weight > 1.0f) return 0;
    memcpy(p, q, sizeof(p)); memcpy(t, target, sizeof(t));
    if (!normalize(p) || !normalize(t)) return 0;
    const float sign = dot4(p, t) < 0.0f ? -1.0f : 1.0f;
    for (int i = 0; i < 4; ++i)
        out[i] = (1.0f - weight)*p[i] + weight*sign*t[i];
    if (!normalize(out)) return 0;
    memcpy(q, out, sizeof(out));
    return 1;
}

void tm_filter_reset(tm_filter *filter) {
    filter->q[0] = 1.0f;
    for (int i = 1; i < 4; ++i) filter->q[i] = 0.0f;
    for (int i = 0; i < 3; ++i) filter->previous_accel[i] = 0.0f;
    filter->has_previous_accel = 0;
}
void tm_filter_init(tm_filter *filter) {
    filter->config.gyro_method = TM_GYRO_METHOD;
    filter->config.fusion_gain = 0.1f;
    filter->config.gravity = TM_GRAVITY;
    filter->config.accel_tolerance = 0.5f;
    filter->config.max_accel_direction_rate = 0.4f;
    tm_filter_reset(filter);
}

static int config_valid(const tm_filter_config *c) {
    return (c->gyro_method == TM_GYRO_METHOD_FAST || c->gyro_method == TM_GYRO_METHOD_EXACT)
        && isfinite(c->fusion_gain) && c->fusion_gain >= 0.0f
        && isfinite(c->gravity) && c->gravity > 0.0f
        && isfinite(c->accel_tolerance) && c->accel_tolerance >= 0.0f
        && isfinite(c->max_accel_direction_rate) && c->max_accel_direction_rate >= 0.0f;
}

int tm_filter_update(tm_filter *filter, float gx, float gy, float gz,
                     float ax, float ay, float az, float dt) {
    const tm_filter_config *c = &filter->config;
    float predicted[4];
    memcpy(predicted, filter->q, sizeof(predicted));
    if (!config_valid(c) || !integrate(predicted, gx, gy, gz, dt,
                                       c->gyro_method == TM_GYRO_METHOD_EXACT)) return -1;
    const float a[3] = {ax, ay, az};
    const float norm = hypotf(hypotf(ax, ay), az);
    const int valid = isfinite(ax) && isfinite(ay) && isfinite(az)
                   && isfinite(norm) && norm > 0.0f;
    int allowed = valid && fabsf(norm - c->gravity) <= c->accel_tolerance;
    if (allowed && filter->has_previous_accel && c->max_accel_direction_rate > 0.0f) {
        const float *prev = filter->previous_accel;
        const float pn = hypotf(hypotf(prev[0], prev[1]), prev[2]);
        const float u[3] = {ax/norm, ay/norm, az/norm};
        const float v[3] = {prev[0]/pn, prev[1]/pn, prev[2]/pn};
        const float cross[3] = {u[1]*v[2]-u[2]*v[1], u[2]*v[0]-u[0]*v[2], u[0]*v[1]-u[1]*v[0]};
        const float angle = atan2f(hypotf(hypotf(cross[0], cross[1]), cross[2]),
                                  u[0]*v[0]+u[1]*v[1]+u[2]*v[2]);
        allowed = angle <= c->max_accel_direction_rate * dt;
    }
    int corrected = 0;
    if (allowed && c->fusion_gain > 0.0f) {
        float target[4];
        if (tm_accel_quaternion(predicted, a, target)) {
            const float angle = 2.0f*acosf(clampf(fabsf(dot4(predicted, target)), 0.0f, 1.0f));
            const float weight = clampf(c->fusion_gain * angle, 0.0f, 1.0f);
            corrected = tm_quaternion_blend(predicted, target, weight);
        }
    }
    memcpy(filter->q, predicted, sizeof(predicted));
    if (valid) memcpy(filter->previous_accel, a, sizeof(a));
    filter->has_previous_accel = valid;
    return corrected;
}
