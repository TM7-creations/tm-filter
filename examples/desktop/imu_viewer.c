/* POSIX serial viewer (macOS/Linux). No worker thread or shared globals. */
#define _DEFAULT_SOURCE
#include "raylib.h"
#include "tm_filter.h"
#include "madgwick.h"
#include "imu_packet.h"
#include <errno.h>
#include <fcntl.h>
#include <math.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <termios.h>
#include <unistd.h>

static int open_serial(const char *port) {
    int fd=open(port, O_RDONLY | O_NOCTTY | O_NONBLOCK);
    if (fd < 0) return -1;
    struct termios tty;
    if (tcgetattr(fd, &tty) != 0) { close(fd); return -1; }
    cfmakeraw(&tty);
    if (cfsetispeed(&tty, B115200) != 0 || cfsetospeed(&tty, B115200) != 0) {
        close(fd); return -1;
    }
    tty.c_cflag |= CLOCAL | CREAD;
    tty.c_cflag &= ~(PARENB | CSTOPB | CSIZE);
    tty.c_cflag |= CS8;
#ifdef CRTSCTS
    tty.c_cflag &= ~CRTSCTS;
#endif
    tty.c_cc[VMIN]=0; tty.c_cc[VTIME]=0;
    if (tcsetattr(fd, TCSANOW, &tty) != 0) { close(fd); return -1; }
    return fd;
}

static void draw_attitude(const float q[4], Vector3 origin, float length) {
    const float w=q[0], x=q[1], y=q[2], z=q[3];
    const Vector3 axes[3] = {
        {1-2*(y*y+z*z), 2*(x*y+w*z), 2*(x*z-w*y)},
        {2*(x*y-w*z), 1-2*(x*x+z*z), 2*(y*z+w*x)},
        {2*(x*z+w*y), 2*(y*z-w*x), 1-2*(x*x+y*y)}
    };
    const Color colors[3]={RED, GREEN, BLUE};
    for (int i=0; i<3; ++i) {
        Vector3 tip={origin.x+length*axes[i].x, origin.y+length*axes[i].y, origin.z+length*axes[i].z};
        DrawLine3D(origin, tip, colors[i]);
    }
    DrawSphere(origin, 0.04f, DARKGRAY);
}

int main(int argc, char **argv) {
    if (argc < 2 || argc > 4) {
        fprintf(stderr, "Usage: %s SERIAL_PORT [fast|exact] [sample_hz]\n", argv[0]);
        return 1;
    }
    tm_filter filter; tm_filter_init(&filter);
    if (argc >= 3) {
        if (strcmp(argv[2], "exact")==0) filter.config.gyro_method=TM_GYRO_METHOD_EXACT;
        else if (strcmp(argv[2], "fast")==0) filter.config.gyro_method=TM_GYRO_METHOD_FAST;
        else { fprintf(stderr, "Method must be fast or exact.\n"); return 1; }
    }
    float hz=100.0f;
    if (argc == 4) {
        char *end; hz=strtof(argv[3], &end);
        if (*end || !isfinite(hz) || hz <= 0) { fprintf(stderr, "Invalid sample rate.\n"); return 1; }
    }
    int fd=open_serial(argv[1]);
    if (fd < 0) { perror(argv[1]); return 1; }
    float comparison[4]={1,0,0,0};
    imu_decoder decoder={0};
    int have_sequence=0, corrected=0, failed=0;
    int32_t previous_sequence=0;
    unsigned long packets=0, lost=0;
    double last_packet=0;
    InitWindow(1000,650,"TM IMU - analytical correction / Madgwick comparison");
    SetTargetFPS(60);
    Camera3D camera={0};
    camera.position=(Vector3){4,6,4}; camera.target=(Vector3){0,0,0};
    camera.up=(Vector3){0,0,1}; camera.fovy=45; camera.projection=CAMERA_PERSPECTIVE;
    while (!WindowShouldClose() && !failed) {
        /* Bound work per frame even if the sender floods the serial port. */
        for (int batch=0; batch<16; ++batch) {
            uint8_t bytes[256]; ssize_t count=read(fd, bytes, sizeof(bytes));
            if (count < 0 && errno == EINTR) continue;
            if (count < 0 && errno != EAGAIN && errno != EWOULDBLOCK) {
                perror("serial read"); failed=1; break;
            }
            if (count <= 0) break;
            for (ssize_t i=0; i<count; ++i) {
                imu_sample s;
                if (!imu_decode_byte(&decoder, bytes[i], &s)) continue;
                ++packets; last_packet=GetTime();
                if (s.sequence == -1) {
                    tm_filter_reset(&filter);
                    comparison[0]=1; comparison[1]=comparison[2]=comparison[3]=0;
                    have_sequence=1; previous_sequence=-1;
                    /* Calibration marker carries an initial sample, not an interval. */
                    continue;
                }
                int64_t steps=have_sequence ? (int64_t)s.sequence-previous_sequence : 1;
                if (steps <= 0) { /* Board restart or out-of-order data: rebase safely. */
                    previous_sequence=s.sequence;
                    filter.has_previous_accel=0;
                    continue;
                }
                previous_sequence=s.sequence; have_sequence=1;
                if (steps > 1) lost+=(unsigned long)(steps-1);
                if ((double)steps/hz > 0.5) { filter.has_previous_accel=0; continue; }
                const float dt=(float)steps/hz;
                const float gx=s.gyro[0]*TM_DEG_TO_RAD;
                const float gy=s.gyro[1]*TM_DEG_TO_RAD;
                const float gz=s.gyro[2]*TM_DEG_TO_RAD;
                corrected=tm_filter_update(&filter,gx,gy,gz,s.accel[0],s.accel[1],s.accel[2],dt);
                madgwick_update_imu(comparison,gx,gy,gz,s.accel[0],s.accel[1],s.accel[2],dt,0.1f);
            }
        }
        BeginDrawing(); ClearBackground(RAYWHITE);
        BeginMode3D(camera);
        const float identity[4]={1,0,0,0};
        draw_attitude(identity,(Vector3){0,0,0},0.5f);
        draw_attitude(filter.q,(Vector3){-1.5f,0,0},1.2f);
        draw_attitude(comparison,(Vector3){1.5f,0,0},1.2f);
        EndMode3D();
        DrawText("TM at x=-1.5 | Reference at x=0 | Madgwick at x=+1.5",20,20,20,BLACK);
        DrawText("X: red   Y: green   Z: blue | Hold calibration button while still, release to reset",20,50,16,DARKGRAY);
        char status[240];
        snprintf(status,sizeof(status),"TM: %s | accel correction: %s | packets: %lu | missing: %lu",
                 filter.config.gyro_method==TM_GYRO_METHOD_EXACT ? "exact" : "Euler normalized",
                 corrected==1 ? "accepted" : "skipped",packets,lost);
        DrawText(status,20,590,16,DARKGRAY);
        if (packets==0 || GetTime()-last_packet>1.0)
            DrawText("Waiting for sensor data...",20,620,18,MAROON);
        EndDrawing();
    }
    close(fd); CloseWindow();
    return failed ? 1 : 0;
}
