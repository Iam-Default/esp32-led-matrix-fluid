/**
 * renderer.cpp - tile baking and framebuffer rendering.
 */
#include <string.h>
#include <math.h>
#include "renderer.h"

/// Packs 8-bit-per-channel colour values into a 16-bit RGB565 pixel (5 bits red,
/// 6 green, 5 blue). Inputs are clamped to 0..255.
static inline uint16_t rgb565(float r, float g, float b) {
  uint8_t R = (uint8_t)fminf(255.f, fmaxf(0.f, r));
  uint8_t G = (uint8_t)fminf(255.f, fmaxf(0.f, g));
  uint8_t B = (uint8_t)fminf(255.f, fmaxf(0.f, b));
  return ((R & 0xF8) << 8) | ((G & 0xFC) << 3) | (B >> 3);
}
static inline uint16_t rgb565(const RGB &c) { return rgb565(c.r, c.g, c.b); }

/// Linear blend from colour a (t = 0) to colour b (t = 1), returned as RGB565.
static inline uint16_t mix(const RGB &a, const RGB &b, float t) {
  return rgb565(a.r + (b.r - a.r) * t, a.g + (b.g - a.g) * t, a.b + (b.b - a.b) * t);
}

void Renderer::begin() {
  bg_ = rgb565(COLOR_BG);
  const float half = CELL_PX * 0.5f;        // distance from tile centre to its edge
  const float coreHalf = CORE_PX * 0.5f;    // distance from tile centre to the core's edge
  for (int y = 0; y < CELL_PX; y++)
    for (int x = 0; x < CELL_PX; x++) {
      // Square (Chebyshev) distance of this pixel's centre from the tile centre.
      float d = fmaxf(fabsf(x + 0.5f - half), fabsf(y + 0.5f - half));
      int i = y * CELL_PX + x;
      tileBg_[i] = rgb565(COLOR_BG);
      if (d < coreHalf) {
        // Inside the hard core. The single middle pixel is drawn in COLOR_CENTER.
        bool centre = (x == CELL_PX / 2 && y == CELL_PX / 2);
        tileOn_[i]  = rgb565(centre ? COLOR_CENTER : COLOR_CORE);
        tileOff_[i] = rgb565(COLOR_OFF);
      } else {
        // In the glow ring: fades linearly from the core edge to the tile edge.
        float t = 1.0f - (d - coreHalf) / (half - coreHalf);   // 1 at core edge -> 0 at tile edge
        tileOn_[i]  = mix(COLOR_BG, COLOR_GLOW, GLOW_PEAK * fmaxf(0.f, t));
        tileOff_[i] = rgb565(COLOR_BG);                        // unlit cells have no glow
      }
    }
}

void Renderer::render(uint16_t *fb, const FluidSim &sim) const {
  // Clear the whole frame to the background colour, two pixels per 32-bit write.
  // This also paints the thin margin around the grid.
  uint32_t bg32 = ((uint32_t)bg_ << 16) | bg_;
  uint32_t *f32 = (uint32_t *)fb;
  for (int i = 0; i < SCREEN_PX * SCREEN_PX / 2; i++) f32[i] = bg32;

  for (int cy = 0; cy < GRID; cy++) {
    // Choose the tile for each cell in this row: background outside the circle,
    // otherwise lit or unlit depending on whether a particle is there.
    const uint16_t *tp[GRID];
    for (int cx = 0; cx < GRID; cx++)
      tp[cx] = !sim.isValid(cx, cy) ? tileBg_ : (sim.isLit(cx, cy) ? tileOn_ : tileOff_);
    // Copy the tiles one pixel row at a time into the framebuffer.
    for (int py = 0; py < CELL_PX; py++) {
      uint16_t *dst = fb + (GRID_OFFSET + cy * CELL_PX + py) * SCREEN_PX + GRID_OFFSET;
      for (int cx = 0; cx < GRID; cx++)
        memcpy(dst + cx * CELL_PX, tp[cx] + py * CELL_PX, CELL_PX * sizeof(uint16_t));
    }
  }
}
