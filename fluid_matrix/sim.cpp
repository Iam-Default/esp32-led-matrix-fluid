/**
 * sim.cpp - implementation of the fluid simulation (see sim.h for the overview).
 *
 * Each physics step does, for every particle (in a random order):
 *   1. work out its acceleration: gravity, plus a sideways "spread" push if it is
 *      resting on something and could flow somewhere lower,
 *   2. update its velocity (with drag and a speed limit),
 *   3. move it along x, then along y, only entering a neighbouring cell if free.
 */
#include "sim.h"
#include <math.h>

// The 8 neighbour directions in angle order starting at +x and turning toward +y
// (screen y points down): E, SE, S, SW, W, NW, N, NE. An index k therefore
// represents an angle of k * 45 degrees, which lets us rotate a direction by
// adding to its index.
static const int8_t DX[8] = { 1, 1, 0,-1,-1,-1, 0, 1};
static const int8_t DY[8] = { 0, 1, 1, 1, 0,-1,-1,-1};

// Tiny margin used to park a blocked particle just inside its own cell edge so
// that (int)position still maps to the cell it really occupies.
static constexpr float EPS = 1e-3f;

/// xorshift32 pseudo-random generator: very cheap and good enough for shuffling.
uint32_t FluidSim::rnd() {
  rng_ ^= rng_ << 13; rng_ ^= rng_ >> 17; rng_ ^= rng_ << 5;
  return rng_;
}

/// A cell is "free" if it is inside the grid, inside the circular container and
/// no particle is in it. Out-of-bounds and masked-out cells count as walls.
bool FluidSim::freeCell(int cx, int cy) const {
  if ((unsigned)cx >= (unsigned)GRID || (unsigned)cy >= (unsigned)GRID) return false;
  int i = cy * GRID + cx;
  return valid_[i] && occ_[i] == EMPTY;
}

int FluidSim::validCells() const {
  int n = 0;
  for (int i = 0; i < GRID * GRID; i++) n += valid_[i];
  return n;
}

void FluidSim::begin() {
  // Mark every cell whose centre lies within CONTAINER_RADIUS of the grid centre.
  for (int cy = 0; cy < GRID; cy++)
    for (int cx = 0; cx < GRID; cx++) {
      float dx = cx + 0.5f - GRID / 2.0f, dy = cy + 0.5f - GRID / 2.0f;
      valid_[cy * GRID + cx] = (dx * dx + dy * dy) <= CONTAINER_RADIUS * CONTAINER_RADIUS;
      occ_[cy * GRID + cx] = EMPTY;
    }
  // Fill from the bottom of the circle upwards, one particle per cell, at rest
  // and centred in its cell.
  int n = 0;
  for (int cy = GRID - 1; cy >= 0 && n < PARTICLE_COUNT; cy--)
    for (int cx = 0; cx < GRID && n < PARTICLE_COUNT; cx++)
      if (valid_[cy * GRID + cx]) {
        p_[n] = {cx + 0.5f, cy + 0.5f, 0, 0};
        occ_[cy * GRID + cx] = n;
        n++;
      }
  for (int i = 0; i < PARTICLE_COUNT; i++) order_[i] = i;
}

/**
 * Liquid levelling. If the particle in cell (cx, cy) is resting on something in
 * the gravity direction, look sideways and diagonally along the surface (up to
 * SPREAD_REACH cells) for a spot it could drop into, and return a unit push
 * direction toward it. A move is only proposed when the particle can actually
 * fall afterwards, so it can never ping-pong between two equal levels, and a
 * flat pool stays perfectly still.
 *
 * @param k          gravity direction index (0..7, see DX/DY)
 * @param outx,outy  receives the push direction when true is returned
 * @return true if there is somewhere lower to flow to
 */
bool FluidSim::spreadDir(int cx, int cy, int k, float &outx, float &outy) {
  int gdx = DX[k], gdy = DY[k];
  if (freeCell(cx + gdx, cy + gdy)) return false;      // not supported, it simply falls

  float cand[4][2]; int n = 0;                         // candidate push directions
  // Try the two directions at +-45 degrees and the two at +-90 degrees from gravity
  // (for downward gravity: down-left, down-right, left, right).
  for (int off = -2; off <= 2; off++) {
    if (off == 0) continue;
    int kk = (k + off + 8) & 7;
    int ddx = DX[kk], ddy = DY[kk];

    // Walk along this direction while the path is clear; stop at the first cell
    // that has a free cell "below" it (in the gravity direction).
    bool found = false;
    for (int m = 1; m <= SPREAD_REACH; m++) {
      int tx = cx + ddx * m, ty = cy + ddy * m;
      if (!freeCell(tx, ty)) break;                    // path blocked
      if (freeCell(tx + gdx, ty + gdy)) { found = true; break; }   // can fall here
    }
    if (!found) continue;

    // Convert the direction into a push vector. For a diagonal, only push along
    // an axis whose neighbouring cell is free, so the particle cannot cut a corner.
    float ax, ay;
    if (ddx == 0 || ddy == 0) { ax = ddx; ay = ddy; }
    else {
      bool sx = freeCell(cx + ddx, cy), sy = freeCell(cx, cy + ddy);
      if (sx && sy) { ax = ddx * 0.7071f; ay = ddy * 0.7071f; }   // 1/sqrt(2): unit length
      else if (sx)  { ax = ddx; ay = 0; }
      else if (sy)  { ax = 0;   ay = ddy; }
      else continue;
    }
    cand[n][0] = ax; cand[n][1] = ay; n++;
  }
  if (!n) return false;
  int pick = (n == 1) ? 0 : (int)(rnd() % n);         // random choice avoids a left/right bias
  outx = cand[pick][0]; outy = cand[pick][1];
  return true;
}

/**
 * Moves one particle along one axis by velocity * dt.
 *  - Staying inside the same cell: just update the position.
 *  - Crossing into a free neighbour: update the occupancy grid and the position.
 *  - Blocked: park the particle at its cell edge and resolve the collision.
 *    Against a wall the velocity is reflected (scaled by WALL_REST). Against another
 *    particle, a hard hit exchanges momentum (PART_REST) and a gentle contact makes
 *    this particle just follow the other one's velocity, so resting stacks stay quiet.
 *
 * @param idx   particle index
 * @param axis  0 = x, 1 = y
 */
void FluidSim::moveAxis(int idx, int axis, float dt) {
  P &p = p_[idx];
  float &pos = axis ? p.y : p.x;      // position and velocity along the chosen axis
  float &vel = axis ? p.vy : p.vx;
  int c0 = (int)pos;                  // cell index along this axis before the move
  float np = pos + vel * dt;          // desired new position
  int c1 = (int)floorf(np);           // cell index it would end up in
  if (c1 == c0) { pos = np; return; }

  int ox = (int)p.x, oy = (int)p.y;                   // current cell
  int tx = axis ? ox : c1, ty = axis ? c1 : oy;       // target cell
  if (freeCell(tx, ty)) {
    occ_[oy * GRID + ox] = EMPTY;
    occ_[ty * GRID + tx] = idx;
    pos = np;
    return;
  }

  // Blocked: stop at the edge of the current cell, on the side we were heading.
  int s = (c1 > c0) ? 1 : -1;                         // direction of travel, +1 or -1
  pos = (s > 0) ? (c0 + 1 - EPS) : (c0 + EPS);

  bool inGrid = (unsigned)tx < (unsigned)GRID && (unsigned)ty < (unsigned)GRID;
  uint16_t o = (inGrid && valid_[ty * GRID + tx]) ? occ_[ty * GRID + tx] : EMPTY;
  if (o == EMPTY) {                                   // wall (or out of the container)
    vel = -vel * WALL_REST;
  } else {                                            // another particle, index o
    float &qv = axis ? p_[o].vy : p_[o].vx;           // its velocity along the same axis
    float rel = (vel - qv) * s;                       // closing speed (positive = approaching)
    if (rel > IMPACT_VEL) {
      float j = (1.0f + PART_REST) * rel * 0.5f;      // equal-mass collision impulse
      vel -= s * j;
      qv  += s * j;
    } else if (rel > 0) {
      vel = qv;                                       // gentle contact: just follow it
    }
  }
}

void FluidSim::step(float dt, float gx, float gy) {
  // Is the tilt large enough to matter? Below SPREAD_MIN_G the liquid-levelling
  // rule is disabled, so a board lying flat leaves the fluid alone.
  float gm2 = gx * gx + gy * gy;
  float minG = SPREAD_MIN_G * GRAVITY_CELLS_PER_G;
  bool hasG = gm2 > minG * minG;

  // Quantise the gravity direction to one of 8 neighbour directions (index k,
  // see DX/DY). This is only used for the levelling rule; the real gravity vector
  // is applied in full below.
  int k = 0;
  if (hasG) k = ((int)lroundf(atan2f(gy, gx) / (float)(M_PI / 4)) + 8) & 7;

  // Random processing order each step (Fisher-Yates shuffle) removes any
  // directional bias from who gets to move first.
  for (int i = PARTICLE_COUNT - 1; i > 0; i--) {
    int j = rnd() % (i + 1);
    uint16_t t = order_[i]; order_[i] = order_[j]; order_[j] = t;
  }

  const float vmax = 0.9f / dt;                       // never cross more than one cell per step
  const float drag = 1.0f - DRAG * dt;                // per-step velocity damping factor

  for (int n = 0; n < PARTICLE_COUNT; n++) {
    int idx = order_[n];
    P &p = p_[idx];

    // 1. Acceleration: gravity plus the optional levelling push.
    float ax = gx, ay = gy;
    if (hasG) {
      float sx, sy;
      if (spreadDir((int)p.x, (int)p.y, k, sx, sy)) {
        ax += sx * SPREAD_ACCEL;
        ay += sy * SPREAD_ACCEL;
      }
    }

    // 2. Velocity update with drag, then clamp the speed.
    p.vx = (p.vx + ax * dt) * drag;
    p.vy = (p.vy + ay * dt) * drag;
    if (p.vx >  vmax) p.vx =  vmax;
    if (p.vx < -vmax) p.vx = -vmax;
    if (p.vy >  vmax) p.vy =  vmax;
    if (p.vy < -vmax) p.vy = -vmax;

    // 3. Move one axis at a time so particles slide along walls and each other.
    moveAxis(idx, 0, dt);
    moveAxis(idx, 1, dt);
  }
}
