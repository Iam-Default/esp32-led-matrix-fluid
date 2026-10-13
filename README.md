# ESP32-S3 LED-Matrix Fluid

A liquid simulation that looks like a glowing dot-matrix LED display, running on the
**Waveshare ESP32-S3-Touch-LCD-1.28** (round 240x240 GC9A01 LCD + QMI8658 IMU).
Tilt the board and the fluid sloshes toward "down"; shake it and it splashes.

* 34x34 grid of 7x7 px cells, masked to a circle
* Always exactly `PARTICLE_COUNT` lit cells (default 300, about 35% of the circle): one particle per cell, never overlapping
* Each lit cell has a hard 5x5 core, a white centre pixel and a soft 1 px glow ring, so the LCD looks like separate LEDs
* Driven by the raw accelerometer: tilt sets the gravity direction, quick movement adds to it
* Double-buffered DMA display push, so the frame rate is limited by the SPI transfer

## Hardware

| Part | Details |
|---|---|
| Board | [Waveshare ESP32-S3-Touch-LCD-1.28](https://www.waveshare.com/esp32-s3-touch-lcd-1.28.htm) |
| Display | GC9A01, 240x240, SPI |
| IMU | QMI8658 (I2C, address 0x6B or 0x6A, auto-detected) |

Pins (set in `config.h`): LCD SCLK 10, MOSI 11, MISO 12, DC 8, CS 9, RST 14, BL 2. IMU SDA 6, SCL 7.
The touchscreen is not used.

## Build and flash (Arduino IDE)

1. Install **esp32 by Espressif Systems** in Boards Manager.
2. Install **LovyanGFX** (by lovyan03) in Library Manager.
3. Open `fluid_matrix/fluid_matrix.ino` (the folder name must match the `.ino` name).
4. In the Tools menu:

   | Setting | Value |
   |---|---|
   | Board | ESP32S3 Dev Module |
   | USB CDC On Boot | Enabled |
   | Flash Size | 16MB (128Mb) |
   | Partition Scheme | any 16MB scheme |
   | Flash Mode | QIO 80MHz |
   | PSRAM | Disabled |

5. Select the COM port and upload. If it hangs on "Connecting...", hold **BOOT** while plugging in USB, then retry.

## How it works

* **Particles:** every particle has a continuous position and velocity but may only enter a neighbouring cell if that cell is free. An occupancy grid is the source of truth, so cells never hold two particles. This is why the display never needs a density threshold or downscaling.
* **Collisions:** walls bounce particles a little. Fast particle-to-particle impacts pass momentum along, while slow contact just stops them, so resting fluid stays calm.
* **Liquid behaviour:** a particle resting on something scans sideways and diagonally along the surface for a spot where it could drop at least one cell, and is nudged toward it. Because it only moves when it can fall, it never oscillates, and the surface levels out.
* **Gravity from the IMU:** effective gravity is the negated accelerometer reading (in the screen's x/y plane). The reading is split into a slow part (tilt) and a fast part (shaking), and the fast part is multiplied by `SHAKE_GAIN`.
* **Rendering:** three pre-baked 7x7 tiles (lit, unlit, background) are copied into an RGB565 framebuffer, which is sent by DMA while the next frame is computed.

## Settings reference (`fluid_matrix/config.h`)

Everything adjustable is in this one file.

### Display and hardware

| Setting | Default | Meaning |
|---|---|---|
| `PIN_LCD_*`, `PIN_IMU_*` | see above | GPIO pins for the display and IMU |
| `LCD_SPI_HZ` | `80000000` | SPI clock. Drop to `40000000` if you see glitches |
| `LCD_INVERT` | `true` | Panel colour inversion. Set `false` if colours look inverted |
| `BACKLIGHT` | `255` | Backlight brightness, 0 to 255 |

### Grid and fluid size

| Setting | Default | Meaning |
|---|---|---|
| `CELL_PX` | `7` | Pixels per cell. `GRID`, `GRID_PX` and `GRID_OFFSET` follow from it (34 cells, 238 px, 1 px margin) |
| `CONTAINER_RADIUS` | `16.5` | Radius of the circular container in cells. Cells whose centre is further out stay dark |
| `PARTICLE_COUNT` | `300` | Number of lit cells, always. The circle has about 848 cells, so 300 is about 35% |

If you change `CELL_PX`, retune `CORE_PX`, `CONTAINER_RADIUS`, `PARTICLE_COUNT` and the speed values below, since they are measured in cells.

### Look and colours

Colours are `{R, G, B}` values from 0 to 255.

| Setting | Default | Meaning |
|---|---|---|
| `COLOR_CORE` | `{110,255,150}` | Hard-edged centre of a lit cell |
| `COLOR_CENTER` | `{255,255,255}` | The single middle pixel of a lit cell |
| `COLOR_GLOW` | `{0,190,80}` | Colour of the soft ring around the core |
| `COLOR_OFF` | `{5,18,9}` | Centre of an unlit cell. Set equal to `COLOR_BG` to hide the dark LEDs |
| `COLOR_BG` | `{0,0,0}` | Gaps between cells and everything outside the circle |
| `CORE_PX` | `5` | Size of the hard core. Use an odd number (3 or 5) for a 7 px cell |
| `GLOW_PEAK` | `0.9` | Glow brightness next to the core, 0 to 1 |

### IMU and gravity

| Setting | Default | Meaning |
|---|---|---|
| `IMU_SWAP_XY` | `1` | Swap the sensor's X and Y axes |
| `IMU_INVERT_X` | `1` | Flip left/right gravity |
| `IMU_INVERT_Y` | `0` | Flip up/down gravity |
| `IMU_SMOOTHING` | `0.85` | Weight of the newest accelerometer sample, from 0 to 1. 1 means no smoothing |
| `GRAVITY_CELLS_PER_G` | `120` | Cells/s^2 produced by 1 g. **The main speed control**: raise for faster fluid |
| `IMU_MAX_G` | `10.0` | Clamp on the effective acceleration, so hard shakes cannot go off the scale |
| `SHAKE_GAIN` | `3.0` | How much quick movement is amplified compared with slow tilting. Raise for bigger splashes |
| `GRAV_TAU` | `0.4` | Seconds. Changes slower than this count as tilt, faster ones as shake |
| `DEBUG_IMU` | off | Uncomment to print x/y/z acceleration over serial at 115200 baud |

If tilting moves the fluid the wrong way, flip `IMU_INVERT_X` or `IMU_INVERT_Y`, or change `IMU_SWAP_XY`. Only the screen's x/y plane is used, so a perfectly flat board does not move the fluid.

### Physics

| Setting | Default | Meaning |
|---|---|---|
| `SIM_HZ` | `480` | Fixed physics steps per second. Also sets the speed ceiling (about `0.9 x SIM_HZ` cells/s) |
| `MAX_STEPS_PER_FRAME` | `16` | Cap on physics steps per drawn frame. If a frame takes too long the sim slows down instead of freezing |
| `DRAG` | `0.4` | Velocity damping per second |
| `WALL_REST` | `0.20` | Bounciness against the container wall, 0 to 1 |
| `PART_REST` | `0.10` | Bounciness between particles, 0 to 1 |
| `IMPACT_VEL` | `8.0` | Cells/s. Particle contacts faster than this transfer momentum, slower ones just stop |
| `SPREAD_ACCEL` | `90` | Sideways push that makes the surface level out. Higher levels faster |
| `SPREAD_REACH` | `11` | How many cells along the surface a particle looks for a lower spot. Higher levels wide pools faster |
| `SPREAD_MIN_G` | `0.05` | Below this tilt (in g) the fluid just sits still |

### Tuning tips

* **Too fast or too slow:** change `GRAVITY_CELLS_PER_G`. Scale `SPREAD_ACCEL` (about 0.75 times it) and `IMPACT_VEL` (about 0.07 times it) with it, and keep `SHAKE_GAIN` times gravity roughly constant to keep shaking at the same strength.
* **Fluid reacts too much when you turn the board:** lower `SHAKE_GAIN` or raise `GRAV_TAU`.
* **Surface looks jagged or sandy:** raise `SPREAD_ACCEL` or `SPREAD_REACH`.
* **Fluid jitters when still:** lower `IMU_SMOOTHING` (a lower number means more smoothing), or raise `SPREAD_MIN_G`.
* **Higher speeds hit a ceiling:** raise `SIM_HZ` (and `MAX_STEPS_PER_FRAME` with it).

## Project layout

```
fluid_matrix/
  fluid_matrix.ino   setup and main loop (IMU -> sim -> render -> DMA)
  config.h           all settings
  sim.h / sim.cpp    particle simulation
  imu.h / imu.cpp    QMI8658 driver and tilt/shake separation
  renderer.h / .cpp  LED tile rendering
  display.h          LovyanGFX configuration for the GC9A01
```

## Notes

* Gravity is deliberately much slower than real scale. At this size (about 0.95 mm per cell) real gravity would cross the display in under a tenth of a second, which looks like a teleport.
* The simulation logic has no Arduino dependencies and can be compiled and tested on a PC.

## License

MIT, see [LICENSE](LICENSE).
