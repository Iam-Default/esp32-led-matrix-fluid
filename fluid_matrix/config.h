/**
 * config.h - every tunable setting for the LED-matrix fluid.
 *
 * Distances are measured in grid CELLS (one cell = CELL_PX pixels = ~0.95 mm on
 * the physical 1.28" panel), times in seconds, and accelerations in cells/s^2
 * unless a comment says otherwise. Colours are {R, G, B} with 0..255 per channel.
 *
 * If you change CELL_PX, re-tune CONTAINER_RADIUS, PARTICLE_COUNT, CORE_PX and
 * the speed settings, because they are all expressed in cells.
 */
#pragma once
#include <stdint.h>

// ============================================================================
//  Hardware (Waveshare ESP32-S3-Touch-LCD-1.28: GC9A01 display + QMI8658 IMU)
// ============================================================================

// Display SPI pins.
#define PIN_LCD_SCLK 10
#define PIN_LCD_MOSI 11
#define PIN_LCD_MISO 12
#define PIN_LCD_DC    8   // data/command select
#define PIN_LCD_CS    9   // chip select
#define PIN_LCD_RST  14   // reset
#define PIN_LCD_BL    2   // backlight (PWM)

// IMU I2C pins (the touch controller shares this bus but is unused).
#define PIN_IMU_SDA   6
#define PIN_IMU_SCL   7

#define LCD_SPI_HZ   80000000   // SPI clock; drop to 40000000 if you see glitches
#define LCD_INVERT   true       // panel colour inversion; set false if colours look inverted
#define BACKLIGHT    255        // backlight brightness, 0..255

// ============================================================================
//  Grid
// ============================================================================

constexpr int   SCREEN_PX        = 240;                          // physical panel, pixels per side
constexpr int   CELL_PX          = 7;                            // pixels per cell
constexpr int   GRID             = SCREEN_PX / CELL_PX;          // cells per side (34)
constexpr int   GRID_PX          = GRID * CELL_PX;               // pixels the grid covers (238)
constexpr int   GRID_OFFSET      = (SCREEN_PX - GRID_PX) / 2;    // margin that centres the grid (1 px)
constexpr float CONTAINER_RADIUS = 16.5f;  // radius of the round container; cells whose centre is further out stay dark
constexpr int   PARTICLE_COUNT   = 300;    // number of lit cells, ALWAYS (~35% of the circle's ~848 cells)

// ============================================================================
//  Colours and LED look
//
//  A lit cell is drawn as a CORE_PX x CORE_PX square of COLOR_CORE with a single
//  COLOR_CENTER pixel in the middle, surrounded by a glow that fades from
//  COLOR_GLOW (scaled by GLOW_PEAK) to COLOR_BG at the cell edge.
// ============================================================================

struct RGB { uint8_t r, g, b; };
constexpr RGB COLOR_CORE  {110, 255, 150};  // hard-edged body of a lit cell
constexpr RGB COLOR_CENTER{255, 255, 255};  // the single middle pixel of a lit cell
constexpr RGB COLOR_GLOW  {  0, 190,  80};  // soft ring around the core
constexpr RGB COLOR_OFF   {  5,  18,   9};  // centre of an UNLIT cell (set equal to COLOR_BG to hide)
constexpr RGB COLOR_BG    {  0,   0,   0};  // gaps between cells and everything outside the circle

constexpr int   CORE_PX   = 5;     // hard core size; use an odd number (3 or 5) for a 7 px cell
constexpr float GLOW_PEAK = 0.9f;  // glow brightness right next to the core, 0..1

// ============================================================================
//  IMU and gravity
//
//  Pipeline: raw accel -> axis swap/invert -> light smoothing -> split into
//  slow part (tilt) and fast part (shake) -> shake is multiplied -> negated to
//  get effective gravity -> clamped -> scaled to cells/s^2.
// ============================================================================

// Axis mapping. Change these if tilting moves the fluid the wrong way.
#define IMU_SWAP_XY    1       // 1 = swap the sensor's X and Y axes
#define IMU_INVERT_X   1       // 1 = flip the sign of screen-X gravity
#define IMU_INVERT_Y   0       // 1 = flip the sign of screen-Y gravity

constexpr float IMU_SMOOTHING       = 0.85f;  // weight of the newest sample, 0..1 (1 = no smoothing)
constexpr float GRAVITY_CELLS_PER_G = 120.f;  // cells/s^2 produced by 1 g; THE main speed control
constexpr float IMU_MAX_G           = 10.0f;  // clamp on effective acceleration, in g
constexpr float SHAKE_GAIN          = 3.0f;   // how much more strongly quick movement acts than slow tilting
constexpr float GRAV_TAU            = 0.4f;   // seconds; changes slower than this count as tilt, faster as shake
// #define DEBUG_IMU                          // uncomment to print x/y/z accel over serial (115200 baud)

// ============================================================================
//  Simulation
// ============================================================================

constexpr int   SIM_HZ              = 480;          // fixed physics steps per second (also caps speed at ~0.9*SIM_HZ cells/s)
constexpr float SIM_DT              = 1.0f / SIM_HZ; // duration of one physics step, seconds
constexpr int   MAX_STEPS_PER_FRAME = 16;           // cap on steps per drawn frame; if exceeded the sim slows instead of freezing
constexpr float DRAG          = 0.4f;    // velocity damping, 1/s
constexpr float WALL_REST     = 0.20f;   // bounciness against the container wall, 0..1
constexpr float PART_REST     = 0.10f;   // bounciness between particles, 0..1
constexpr float IMPACT_VEL    = 8.0f;    // cells/s; contacts faster than this pass momentum on, slower ones just stop (keeps rest calm)
constexpr float SPREAD_ACCEL  = 90.0f;   // cells/s^2 sideways push that makes the surface level out like a liquid
constexpr int   SPREAD_REACH  = 11;      // how far (cells) along the surface a particle looks for a lower spot
constexpr float SPREAD_MIN_G  = 0.05f;   // below this tilt (in g) the fluid just sits still
