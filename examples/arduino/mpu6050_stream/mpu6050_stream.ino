/* MPU-6050 acquisition only: the filters run in the desktop viewer.
 * Binary 115200-baud stream. See docs/serial-protocol.md.
 * Keep the sensor still while holding the button; release to save gyro bias.
 */
#include <Wire.h>
#include <stdint.h>
#include <string.h>

const uint8_t MPU_ADDRESS = 0x68;
const uint8_t BUTTON_PIN = 2; // Button to GND; internal pull-up.
const unsigned long SAMPLE_PERIOD_US = 10000; // Nominal 100 Hz.
const float GRAVITY = 9.80665f;
/* Specific force, in sensor axes. At rest with +Z up: az ~= +9.81 m/s^2.
 * Replace these zero defaults with YOUR measured biases, in m/s^2. */
const float ACCEL_BIAS[3] = {0, 0, 0};
float gyroBias[3] = {0, 0, 0};
float gyroSum[3] = {0, 0, 0};
uint32_t calibrationSamples = 0;
int32_t sequence = -1; // Reset the receiver when the board boots.
unsigned long lastSample = 0;
bool calibrating = false;
bool sensorReady = false;

typedef char float_must_be_32_bits[(sizeof(float) == 4) ? 1 : -1];

bool writeRegister(uint8_t reg, uint8_t value) {
  Wire.beginTransmission(MPU_ADDRESS);
  Wire.write(reg); Wire.write(value);
  return Wire.endTransmission() == 0;
}
int16_t readSigned16() {
  const uint16_t high = (uint16_t)Wire.read();
  const uint16_t bits = (high << 8) | (uint16_t)Wire.read();
  return bits < 0x8000u ? (int16_t)bits : (int16_t)((int32_t)bits - 65536L);
}
void writeU32(uint32_t value) {
  for (uint8_t i = 0; i < 4; ++i) Serial.write((uint8_t)(value >> (8*i)));
}
void writeFloat(float value) {
  uint32_t bits; memcpy(&bits, &value, 4); writeU32(bits);
}
void setup() {
  Serial.begin(115200);
  Wire.begin();
  pinMode(BUTTON_PIN, INPUT_PULLUP);
  delay(100);
  sensorReady = writeRegister(0x6B, 0x01); // Wake, PLL clock.
  delay(100);
  sensorReady = sensorReady && writeRegister(0x1A, 0x03); // DLPF enabled.
  sensorReady = sensorReady && writeRegister(0x19, 9);    // 1 kHz / (9+1).
  sensorReady = sensorReady && writeRegister(0x1B, 0x18); // Gyro +/-2000 deg/s.
  sensorReady = sensorReady && writeRegister(0x1C, 0x00); // Accel +/-2 g.
  lastSample = micros();
}
void loop() {
  if (!sensorReady) return; // No text mixed into the binary stream.
  const unsigned long now = micros();
  if ((unsigned long)(now-lastSample) < SAMPLE_PERIOD_US) return;
  lastSample = now;
  const bool pressed = digitalRead(BUTTON_PIN) == LOW;
  if (pressed && !calibrating) {
    calibrating = true; calibrationSamples = 0;
    for (uint8_t i=0; i<3; ++i) gyroSum[i]=0;
  }
  if (!pressed && calibrating) {
    calibrating = false;
    if (calibrationSamples > 0)
      for (uint8_t i=0; i<3; ++i) gyroBias[i]=gyroSum[i]/calibrationSamples;
    sequence=-1; // Reset marker; biases are kept in RAM only.
  }
  Wire.beginTransmission(MPU_ADDRESS); Wire.write(0x3B);
  if (Wire.endTransmission(false) != 0) return;
  if (Wire.requestFrom(MPU_ADDRESS, (uint8_t)14) != 14) return;
  int16_t rawAccel[3], rawGyro[3];
  for (uint8_t i=0; i<3; ++i) rawAccel[i]=readSigned16();
  (void)readSigned16(); // Temperature.
  for (uint8_t i=0; i<3; ++i) rawGyro[i]=readSigned16();
  float accel[3], gyro[3];
  for (uint8_t i=0; i<3; ++i) {
    accel[i]=(rawAccel[i]/16384.0f)*GRAVITY-ACCEL_BIAS[i];
    gyro[i]=rawGyro[i]/16.4f;
  }
  if (calibrating) {
    for (uint8_t i=0; i<3; ++i) gyroSum[i]+=gyro[i];
    ++calibrationSamples; return;
  }
  Serial.write((uint8_t)0xAA); Serial.write((uint8_t)0x55);
  for (uint8_t i=0; i<3; ++i) writeFloat(accel[i]);
  for (uint8_t i=0; i<3; ++i) writeFloat(gyro[i]-gyroBias[i]);
  writeU32((uint32_t)sequence);
  sequence = sequence == INT32_MAX ? -1 : sequence+1;
}
