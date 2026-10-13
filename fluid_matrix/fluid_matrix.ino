/**
 * fluid_matrix.ino - LED-matrix fluid for the Waveshare ESP32-S3-Touch-LCD-1.28.
 *
 * Every frame:
 *   1. read the IMU and turn it into an effective gravity vector,
 *   2. advance the particle simulation by the elapsed real time (fixed-size steps),
 *   3. render the lit cells into a framebuffer,
 *   4. send the framebuffer to the display with DMA.
 *
 * Two framebuffers alternate, so the CPU computes and renders the next frame
 * while the previous one is still being sent to the display over SPI. The frame
 * rate is therefore limited by the SPI transfer, not by the simulation.
 *
 * All tunable settings are in config.h.
 */
#include <Arduino.h>
#include <esp_heap_caps.h>
#include "config.h"
#include "display.h"
#include "imu.h"
#include "sim.h"
#include "renderer.h"

static LGFX      lcd;        // display driver (LovyanGFX)
static Imu       imu;        // accelerometer
static FluidSim  sim;        // particle simulation
static Renderer  renderer;   // LED-style drawing
static uint16_t *fb[2];      // two DMA-capable RGB565 framebuffers
static int       cur = 0;    // index of the framebuffer being drawn into

void setup() {
  Serial.begin(115200);

  lcd.init();
  lcd.setBrightness(BACKLIGHT);
  lcd.fillScreen(TFT_BLACK);

  // Two DMA-capable framebuffers (2 x 112.5 KB): draw into one while the other is being sent.
  for (int i = 0; i < 2; i++) {
    fb[i] = (uint16_t *)heap_caps_malloc(SCREEN_PX * SCREEN_PX * 2, MALLOC_CAP_DMA | MALLOC_CAP_INTERNAL);
    if (!fb[i]) { Serial.println("framebuffer alloc failed"); while (true) delay(1000); }
    memset(fb[i], 0, SCREEN_PX * SCREEN_PX * 2);
  }

  if (!imu.begin()) Serial.println("QMI8658 not found - check I2C pins/address");

  sim.begin();
  renderer.begin();
  lcd.startWrite();   // keep one long SPI transaction open; frames are pushed with pushImageDMA()
}

void loop() {
  static uint32_t last = micros();   // timestamp of the previous frame
  static float acc = 0;              // simulation time still owed (seconds)
  uint32_t now = micros();
  float dt = (now - last) * 1e-6f;   // real time since the last frame
  last = now;
  if (dt > 0.05f) dt = 0.05f;        // after a stall, do not try to catch up all at once

  // Raw acceleration (g). The sensor reports the reaction to gravity plus any
  // motion, and effective gravity is its negation, so tilt AND shaking both work.
  float ax, ay;
  imu.read(ax, ay);
  float gx = -ax, gy = -ay;
  float m = sqrtf(gx * gx + gy * gy);
  if (m > IMU_MAX_G) { gx *= IMU_MAX_G / m; gy *= IMU_MAX_G / m; }   // limit violent shakes
  gx *= GRAVITY_CELLS_PER_G;         // convert g to cells/s^2
  gy *= GRAVITY_CELLS_PER_G;

  // Run the simulation in fixed SIM_DT steps until it has caught up with real time.
  // If it cannot (frame too slow), drop the backlog so it runs slower instead of freezing.
  acc += dt;
  int steps = 0;
  while (acc >= SIM_DT && steps < MAX_STEPS_PER_FRAME) { sim.step(SIM_DT, gx, gy); acc -= SIM_DT; steps++; }
  if (steps == MAX_STEPS_PER_FRAME) acc = 0;

  renderer.render(fb[cur], sim);     // CPU work above overlaps the previous frame's DMA

  lcd.waitDMA();                     // make sure the previous frame has finished sending
  lcd.pushImageDMA(0, 0, SCREEN_PX, SCREEN_PX, (const lgfx::rgb565_t *)fb[cur]);
  cur ^= 1;                          // next frame draws into the other buffer
}
