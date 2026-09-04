#ifndef TM_IMU_PACKET_H
#define TM_IMU_PACKET_H
#include <stddef.h>
#include <stdint.h>
#define IMU_PACKET_SIZE 30

typedef struct { float accel[3], gyro[3]; int32_t sequence; } imu_sample;
typedef struct { uint8_t bytes[IMU_PACKET_SIZE]; size_t used; } imu_decoder;
/* Zero-initialize decoder. Feed bytes in order. Returns 1 for a valid packet.
 * Wire format is little-endian IEEE-754 float32, independent of host endian. */
int imu_decode_byte(imu_decoder *decoder, uint8_t byte, imu_sample *sample);
#endif
