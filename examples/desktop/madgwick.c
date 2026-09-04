/* IMU gradient correction described in Sebastian O. H. Madgwick's report,
 * "An efficient orientation filter for inertial and inertial/magnetic
 * sensor arrays" (2010), section 3.2 and appendix A:
 * https://x-io.co.uk/downloads/madgwick_internal_report.pdf
 * Reorganized from the project's comparison code; no raylib dependency. */
#include "madgwick.h"
#include <math.h>
#include <string.h>

int madgwick_update_imu(float q[4], float gx, float gy, float gz,
                        float ax, float ay, float az, float dt, float beta) {
    if (!isfinite(dt) || dt <= 0 || !isfinite(beta) || beta < 0 ||
        !isfinite(gx) || !isfinite(gy) || !isfinite(gz)) return 0;
    float norm = hypotf(hypotf(q[0], q[1]), hypotf(q[2], q[3]));
    if (!isfinite(norm) || norm == 0) return 0;
    const float w=q[0]/norm, x=q[1]/norm, y=q[2]/norm, z=q[3]/norm;
    float derivative[4] = {
        0.5f*(-x*gx-y*gy-z*gz), 0.5f*(w*gx+y*gz-z*gy),
        0.5f*(w*gy-x*gz+z*gx), 0.5f*(w*gz+x*gy-y*gx)
    };
    norm = hypotf(hypotf(ax, ay), az);
    if (isfinite(ax) && isfinite(ay) && isfinite(az) && isfinite(norm) && norm > 0) {
        ax/=norm; ay/=norm; az/=norm;
        const float f[3] = {2*(x*z-w*y)-ax, 2*(w*x+y*z)-ay, 1-2*(x*x+y*y)-az};
        const float gradient[4] = {
            -2*y*f[0]+2*x*f[1], 2*z*f[0]+2*w*f[1]-4*x*f[2],
            -2*w*f[0]+2*z*f[1]-4*y*f[2], 2*x*f[0]+2*y*f[1]
        };
        const float gn = hypotf(hypotf(gradient[0], gradient[1]), hypotf(gradient[2], gradient[3]));
        /* A zero gradient means no correction, not no gyroscope update. */
        if (gn > 0 && isfinite(gn))
            for (int i=0; i<4; ++i) derivative[i] -= beta*gradient[i]/gn;
    }
    float out[4] = {w+dt*derivative[0], x+dt*derivative[1],
                    y+dt*derivative[2], z+dt*derivative[3]};
    norm = hypotf(hypotf(out[0], out[1]), hypotf(out[2], out[3]));
    if (!isfinite(norm) || norm == 0) return 0;
    for (int i=0; i<4; ++i) out[i] /= norm;
    memcpy(q, out, sizeof(out));
    return 1;
}
