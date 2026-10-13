/**
 * imu.cpp - QMI8658 accelerometer driver and tilt/shake separation.
 */
#include <Arduino.h>
#include <Wire.h>
#include <math.h>
#include "config.h"
#include "imu.h"

// QMI8658 registers used by this driver.
static constexpr uint8_t REG_WHO_AM_I = 0x00;   // reads 0x05 on a QMI8658
static constexpr uint8_t REG_CTRL1    = 0x02;   // interface settings
static constexpr uint8_t REG_CTRL2    = 0x03;   // accelerometer range and data rate
static constexpr uint8_t REG_CTRL7    = 0x08;   // sensor enable bits
static constexpr uint8_t REG_AX_L     = 0x35;   // first accel output register (X low byte)
static constexpr float   LSB_PER_G    = 8192.0f;   // sensitivity at the +-4 g range

bool Imu::writeReg(uint8_t reg, uint8_t val) {
  Wire.beginTransmission(addr_);
  Wire.write(reg); Wire.write(val);
  return Wire.endTransmission() == 0;
}

bool Imu::readRegs(uint8_t reg, uint8_t *buf, uint8_t n) {
  Wire.beginTransmission(addr_);
  Wire.write(reg);
  if (Wire.endTransmission(false) != 0) return false;   // repeated start, keep the bus
  if (Wire.requestFrom((int)addr_, (int)n) != n) return false;
  for (uint8_t i = 0; i < n; i++) buf[i] = Wire.read();
  return true;
}

bool Imu::begin() {
  Wire.begin(PIN_IMU_SDA, PIN_IMU_SCL, 400000);   // 400 kHz fast-mode I2C
  // The chip's address depends on how its SA0 pin is wired, so try both.
  const uint8_t addrs[2] = {0x6B, 0x6A};
  for (uint8_t a : addrs) {
    addr_ = a;
    uint8_t id = 0;
    if (readRegs(REG_WHO_AM_I, &id, 1) && id == 0x05) { ok_ = true; break; }
  }
  if (!ok_) return false;
  writeReg(REG_CTRL1, 0x40);   // auto-increment register address (allows the 6-byte burst read)
  writeReg(REG_CTRL2, 0x14);   // accel +-4 g, output data rate 500 Hz
  writeReg(REG_CTRL7, 0x01);   // enable accel only (gyro stays off)
  delay(20);                   // let the sensor start up
  return true;
}

void Imu::read(float &ax, float &ay) {
  uint8_t b[6];
  if (ok_ && readRegs(REG_AX_L, b, 6)) {
    // Registers hold X, Y, Z as little-endian signed 16-bit values; convert to g.
    float x = (int16_t)(b[0] | (b[1] << 8)) / LSB_PER_G;
    float y = (int16_t)(b[2] | (b[3] << 8)) / LSB_PER_G;
    // Map sensor axes to screen axes using the options in config.h.
#if IMU_SWAP_XY
    float t = x; x = y; y = t;
#endif
#if IMU_INVERT_X
    x = -x;
#endif
#if IMU_INVERT_Y
    y = -y;
#endif
#ifdef DEBUG_IMU
    float z = (int16_t)(b[4] | (b[5] << 8)) / LSB_PER_G;
    Serial.printf("ax=%.2f ay=%.2f az=%.2f\n", x, y, z);
#endif
    // On the very first sample, start every filter at the real value so the
    // fluid does not lurch while the filters warm up.
    if (!primed_) { sx_ = lpx_ = x; sy_ = lpy_ = y; lastUs_ = micros(); primed_ = true; }
    // Light exponential smoothing of the raw signal.
    sx_ += IMU_SMOOTHING * (x - sx_);
    sy_ += IMU_SMOOTHING * (y - sy_);
  }

  // Time since the previous call (clamped so a long stall cannot upset the filter).
  uint32_t now = micros();
  float dt = (now - lastUs_) * 1e-6f;
  lastUs_ = now;
  if (dt > 0.05f) dt = 0.05f;

  // Slow low-pass filter = the tilt (where gravity points). The leftover
  // (smoothed - tilt) is the fast part, i.e. shaking and sudden movement.
  float k = 1.0f - expf(-dt / GRAV_TAU);
  lpx_ += k * (sx_ - lpx_);
  lpy_ += k * (sy_ - lpy_);

  // Tilt unchanged, fast movement amplified by SHAKE_GAIN.
  ax = lpx_ + (sx_ - lpx_) * SHAKE_GAIN;
  ay = lpy_ + (sy_ - lpy_) * SHAKE_GAIN;
}
