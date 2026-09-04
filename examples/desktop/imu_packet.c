#include "imu_packet.h"
#include <float.h>
#include <math.h>
#include <string.h>
#if FLT_RADIX != 2 || FLT_MANT_DIG != 24 || FLT_MAX_EXP != 128
#error "This example requires IEEE-754 float32"
#endif
typedef char float_must_be_32_bits[(sizeof(float) == 4) ? 1 : -1];

static uint32_t read_u32(const uint8_t *p) {
    return (uint32_t)p[0] | ((uint32_t)p[1]<<8) | ((uint32_t)p[2]<<16) | ((uint32_t)p[3]<<24);
}
static float read_float(const uint8_t *p) {
    uint32_t bits=read_u32(p); float value;
    memcpy(&value, &bits, sizeof(value));
    return value;
}
int imu_decode_byte(imu_decoder *d, uint8_t byte, imu_sample *sample) {
    d->bytes[d->used++] = byte;
    while (d->used && (d->bytes[0] != 0xAA || (d->used >= 2 && d->bytes[1] != 0x55))) {
        memmove(d->bytes, d->bytes+1, --d->used);
    }
    if (d->used < IMU_PACKET_SIZE) return 0;
    imu_sample s;
    int valid=1;
    for (int i=0; i<3; ++i) {
        s.accel[i]=read_float(d->bytes+2+4*i);
        s.gyro[i]=read_float(d->bytes+14+4*i);
        if (!isfinite(s.accel[i]) || !isfinite(s.gyro[i])) valid=0;
    }
    uint32_t seq=read_u32(d->bytes+26);
    if (seq == UINT32_MAX) s.sequence=-1;
    else if (seq <= INT32_MAX) s.sequence=(int32_t)seq;
    else valid=0;
    if (valid) { *sample=s; d->used=0; return 1; }
    /* Keep the suffix: the next byte will resynchronize on a new header. */
    memmove(d->bytes, d->bytes+1, IMU_PACKET_SIZE-1);
    d->used=IMU_PACKET_SIZE-1;
    return 0;
}
