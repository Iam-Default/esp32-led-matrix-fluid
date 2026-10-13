/**
 * renderer.h - draws the fluid as a dot-matrix LED display.
 *
 * Three small tiles (CELL_PX x CELL_PX pixels) are pre-computed once: one for a
 * lit cell, one for an unlit cell and one for background. Each frame the
 * framebuffer is built by copying the right tile into every cell, which is far
 * cheaper than drawing pixel by pixel.
 */
#pragma once
#include <stdint.h>
#include "config.h"
#include "sim.h"

class Renderer {
public:
  /// Pre-computes the lit/unlit/background tiles from the colour settings in config.h.
  void begin();

  /// Draws the current state of the simulation into a SCREEN_PX x SCREEN_PX
  /// RGB565 framebuffer (row-major, no padding). Every pixel is overwritten.
  void render(uint16_t *fb, const FluidSim &sim) const;

private:
  uint16_t tileOn_[CELL_PX * CELL_PX];    // lit cell: core + white centre + glow
  uint16_t tileOff_[CELL_PX * CELL_PX];   // unlit cell: faint dot (COLOR_OFF)
  uint16_t tileBg_[CELL_PX * CELL_PX];    // cell outside the circle: solid background
  uint16_t bg_;                           // background colour as RGB565
};
