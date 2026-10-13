/**
 * sim.h - the fluid simulation.
 *
 * The fluid is PARTICLE_COUNT particles, each of which owns exactly ONE grid cell
 * (so exactly PARTICLE_COUNT cells are lit at any time). Each particle has a
 * continuous position and velocity inside its cell, which lets gravity point in
 * any direction and lets momentum build up smoothly. However a particle may only
 * cross into a neighbouring cell if that cell is free, so the occupancy grid is
 * the source of truth and two particles can never overlap.
 *
 * This file has no Arduino dependencies, so it can be compiled and tested on a PC.
 *
 * Coordinates: cell (cx, cy) covers x in [cx, cx+1) and y in [cy, cy+1).
 * +x is right and +y is down on screen.
 */
#pragma once
#include <stdint.h>
#include "config.h"

class FluidSim {
public:
  /// Builds the circular container mask and fills the bottom of it with
  /// PARTICLE_COUNT particles at rest. Call once before step().
  void begin();

  /// Advances the simulation by dt seconds.
  /// @param dt  time step in seconds (use SIM_DT)
  /// @param gx  effective gravity along x in cells/s^2 (+ = right)
  /// @param gy  effective gravity along y in cells/s^2 (+ = down)
  void step(float dt, float gx, float gy);

  /// True if cell (cx, cy) is inside the circular container. Cell must be in [0, GRID).
  bool isValid(int cx, int cy) const { return valid_[cy * GRID + cx]; }

  /// True if a particle currently occupies cell (cx, cy). Cell must be in [0, GRID).
  bool isLit(int cx, int cy)   const { return occ_[cy * GRID + cx] != EMPTY; }

  /// Number of cells inside the container.
  int  validCells() const;

  // Particle positions in cell units, exposed for tests.
  float px(int i) const { return p_[i].x; }
  float py(int i) const { return p_[i].y; }

private:
  static constexpr uint16_t EMPTY = 0xFFFF;   // marker for "no particle in this cell"

  /// One particle: position (cell units) and velocity (cells/s).
  struct P { float x, y, vx, vy; };

  P        p_[PARTICLE_COUNT];   // all particles
  uint16_t occ_[GRID * GRID];    // per cell: index of the particle in it, or EMPTY
  bool     valid_[GRID * GRID];  // per cell: inside the circular container?
  uint16_t order_[PARTICLE_COUNT]; // processing order, reshuffled every step
  uint32_t rng_ = 0x1234ABCD;    // xorshift32 state

  uint32_t rnd();                                   // fast pseudo-random number
  bool freeCell(int cx, int cy) const;              // in bounds, inside container and unoccupied
  bool spreadDir(int cx, int cy, int k, float &dx, float &dy);  // liquid levelling push direction
  void moveAxis(int idx, int axis, float dt);       // move one particle along x (0) or y (1)
};
