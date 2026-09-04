#include "tm_filter.h"
#include <stdio.h>

int main(void) {
    tm_filter filter;
    tm_filter_init(&filter);
    filter.config.gyro_method = TM_GYRO_METHOD_EXACT;
    /* One second of positive 90 deg/s yaw, sampled at 100 Hz.
     * Body +Z remains aligned with world +Z; acceleration cannot observe yaw. */
    for (int i = 0; i < 100; ++i) {
        if (tm_filter_update(&filter, 0.0f, 0.0f, 90.0f * TM_DEG_TO_RAD,
                             0.0f, 0.0f, TM_GRAVITY, 0.01f) < 0) return 1;
    }
    printf("q [w,x,y,z] = [%.6f, %.6f, %.6f, %.6f]\n",
           filter.q[0], filter.q[1], filter.q[2], filter.q[3]);
    return 0;
}
