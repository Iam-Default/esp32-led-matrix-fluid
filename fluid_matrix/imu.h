/**
 * imu.h - minimal driver for the QMI8658 accelerometer on the Waveshare board.
 *
 * Only the accelerometer is used (+-4 g range, 500 Hz). The reading is converted
 * into the tilt/shake signal that drives the fluid's gravity (see read()).
 */
#pragma once
#include <stdint.h>

class Imu {
public:
  /// Starts the I2C bus, finds the sensor (address 0x6B or 0x6A) and enables the
  /// accelerometer. Returns false if the sensor does not answer.
  bool begin();

  /// Reads the accelerometer and returns the signal that drives gravity, in g.
  ///
  /// Axes are mapped to the screen (+x right, +y down) using IMU_SWAP_XY /
  /// IMU_INVERT_X / IMU_INVERT_Y and lightly smoothed (IMU_SMOOTHING). The result
  /// is then split into a slow part (tilt, a low-pass filter with time constant
  /// GRAV_TAU) and a fast part (shaking). Tilt passes through unchanged while the
  /// fast part is multiplied by SHAKE_GAIN. Call it once per frame.
  ///
  /// Note: this is the sensor reading, i.e. the REACTION to gravity. The caller
  /// negates it to get effective gravity.
  void read(float &ax, float &ay);

private:
  uint8_t addr_ = 0x6B;         // I2C address found by begin()
  float sx_ = 0, sy_ = 0;       // smoothed raw reading (g)
  float lpx_ = 0, lpy_ = 0;     // slow estimate of gravity, i.e. the tilt (g)
  uint32_t lastUs_ = 0;         // timestamp of the previous read(), microseconds
  bool  primed_ = false;        // true once the filters have been seeded with a real sample
  bool  ok_ = false;            // true if the sensor was found

  bool  writeReg(uint8_t reg, uint8_t val);                  // write one register
  bool  readRegs(uint8_t reg, uint8_t *buf, uint8_t n);      // read n consecutive registers
};
