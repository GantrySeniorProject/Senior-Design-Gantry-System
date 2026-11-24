/*
  motion.h - 3-axis motion controller (X, Y = carriage, Z = attachment up/down).
  Hardware-independent: the only hardware call is the step callback, so the
  whole controller is tested on the PC by test/test_motion.c. main.c calls
  motion_tick() from its 1 kHz timer with the keypad inputs.

  - Moves: one S-curve profile (sine-shaped acceleration) along the longest
    axis; the other axes follow in proportion, so X/Y moves are straight lines.
    No motor exceeds vmax.
  - Homing: one axis at a time, from a queue (e.g. Z, X, Y). Each axis is
    two-ended (rail.h): toward MIN until MIN is pressed (step 0), then toward
    MAX until MAX is pressed (step count = rail span). Z: MIN = down, MAX = up,
    so after homing the attachment is raised. When the whole queue is done,
    X and Y (if they were in it) drive back to coordinate 0 with a normal move
    (cfg.homeReturn), so 0,0 is the starting position; Z stays raised.
  - Paths: a list of up to MOTION_PATH_MAX X/Y waypoints, driven one straight
    S-curve move after another (each corner stops). The web page plans them
    with A* around blocked cells (web-control/gantry.js).
  - Faults latch and stop every axis: E-stop, a limit hit while moving toward
    it, skipped steps (commanded vs encoder), homing timeout, rail too short.
    Every axis loses its homing; "reset and keep" (E-stop only) restores the
    axes that were homed when it hit, if none of their encoders moved.
  - Skips are simulated: skipInject drops steps (counted as commanded but not
    made), like a belt jumping a tooth.

  Not thread-safe: the caller keeps tick and commands from overlapping
  (main.c uses motionMux).
*/
#pragma once
#include <math.h>
#include <stdbool.h>
#include <stdint.h>
#include "rail.h"

#define MOTION_AXES 3
enum { AXIS_X, AXIS_Y, AXIS_Z };

typedef enum { MM_IDLE, MM_HOMING, MM_MOVE, MM_FAULT } motion_mode_t;
typedef enum { MF_NONE, MF_ESTOP, MF_LIMIT, MF_SKIP, MF_HOME_TIMEOUT, MF_RAIL_SHORT } motion_fault_t;
typedef enum { MOVE_OK, MOVE_BUSY, MOVE_NOT_HOMED, MOVE_BAD_PATH } move_result_t;

#define MOTION_PATH_MAX 64   // waypoints in one path

// Next half-step phase (0..7) for one step in direction dir (+1 / -1);
// reverse flips which way the motor turns for the same commanded direction.
static inline uint8_t stepper_phase_next(uint8_t idx, int dir, bool reverse) {
  bool fwd = (dir > 0) != reverse;
  return (uint8_t)((idx + (fwd ? 1 : 7)) & 7);
}

static inline char motion_axis_name(int a) { return a >= 0 && a < MOTION_AXES ? "XYZ"[a] : '-'; }
static inline const char *motion_mode_name(motion_mode_t m) {
  switch (m) { case MM_IDLE: return "IDLE"; case MM_HOMING: return "HOME";
               case MM_MOVE: return "MOVE"; case MM_FAULT: return "FAULT"; }
  return "?";
}
static inline const char *motion_fault_name(motion_fault_t f) {
  switch (f) { case MF_NONE: return "NONE"; case MF_ESTOP: return "ESTOP"; case MF_LIMIT: return "LIMIT";
               case MF_SKIP: return "SKIP"; case MF_HOME_TIMEOUT: return "HOME TIMEOUT";
               case MF_RAIL_SHORT: return "RAIL SHORT"; }
  return "?";
}

// ----------------------------- S-curve profile ----------------------------
// Distance 0..dist (steps) over time 0..total. Accel ramps up and back down as
// half a sine, so it starts and ends at zero; peak accel = amax.
typedef struct { float dist, vp, ta, tc, total, dacc; } profile_t;

static inline void profile_plan(profile_t *p, float dist, float vmax, float amax) {
  const float PI_F = 3.14159265f;
  float D = fabsf(dist), vp = vmax;
  float ta = PI_F * vp / (2.0f * amax);
  float dacc = vp * ta / 2.0f;
  if (2.0f * dacc > D) {                       // too short to reach vmax
    vp   = sqrtf(2.0f * amax * D / PI_F);
    ta   = PI_F * vp / (2.0f * amax);
    dacc = vp * ta / 2.0f;
  }
  p->dist = D; p->vp = vp; p->ta = ta; p->dacc = dacc;
  p->tc = (vp > 0.0f) ? (D - 2.0f * dacc) / vp : 0.0f;
  if (p->tc < 0.0f) p->tc = 0.0f;
  p->total = 2.0f * ta + p->tc;
}

static inline float profile_s_accel(const profile_t *p, float t) {
  const float PI_F = 3.14159265f;
  if (p->ta <= 0.0f) return 0.0f;
  return 0.5f * p->vp * (t - p->ta / PI_F * sinf(PI_F * t / p->ta));
}

static inline float profile_pos(const profile_t *p, float t) {
  if (t <= 0.0f) return 0.0f;
  if (t >= p->total) return p->dist;           // exactly the distance at the end
  if (t < p->ta) return profile_s_accel(p, t);
  if (t < p->ta + p->tc) return p->dacc + p->vp * (t - p->ta);
  return p->dist - profile_s_accel(p, p->total - t);
}

// ----------------------------- Controller ---------------------------------
typedef struct {
  float   vmax, amax, homeSpeed;     // steps/s, steps/s^2, steps/s
  int32_t homeMaxSteps;              // homing gives up after this many steps per end; <= 0 = never
  int32_t skipThreshold;             // allowed commanded-vs-encoder error
  bool    homeReturn;                // after homing, drive X / Y back to coordinate 0
} motion_config_t;

typedef struct {
  int32_t pos;          // commanded step (0 = MIN end once homed)
  int32_t enc;          // "encoder": steps the motor actually made
  int32_t span;         // steps MIN..MAX, measured by homing
  int32_t skipInject;   // steps still to drop (simulated skips)
  bool    homed;
  bool    homedAtFault; // for keep-home: homed when the fault hit,
  int32_t encAtFault;   // and the encoder then
} motion_axis_t;

typedef struct { bool estop; bool limMin[MOTION_AXES], limMax[MOTION_AXES]; } motion_inputs_t;

typedef void (*motion_step_fn)(void *ctx, int axis, int dir);

typedef struct {
  motion_config_t cfg;
  motion_axis_t   ax[MOTION_AXES];
  motion_mode_t   mode;
  motion_fault_t  fault;
  int             faultAxis;                 // -1: not tied to one axis (E-stop)
  // move
  profile_t prof;
  float     moveT, moveLen;
  int32_t   moveStart[MOTION_AXES], moveDist[MOTION_AXES];
  // homing
  int          homeQueue[MOTION_AXES], homeQLen, homeQIdx;
  // path (X / Y waypoints in rail coordinates); pathIdx = waypoints started so far
  int32_t      pathPts[MOTION_PATH_MAX][2];
  int          pathLen, pathIdx;
  home_phase_t homePhase;
  float        homeVel, homeAcc;
  int32_t      homeSteps;
  // hardware
  motion_step_fn step;
  void          *ctx;
} motion_t;

static inline void motion_init(motion_t *m, const motion_config_t *cfg, motion_step_fn step, void *ctx) {
  *m = (motion_t){0};
  m->cfg = *cfg; m->mode = MM_IDLE; m->fault = MF_NONE; m->faultAxis = -1;
  m->step = step; m->ctx = ctx;
}

// Axis being homed right now, or -1.
static inline int motion_homing_axis(const motion_t *m) {
  return m->mode == MM_HOMING ? m->homeQueue[m->homeQIdx] : -1;
}

// Rail coordinate 0..RAIL_COORD_MAX of an axis, or -1 when it isn't homed.
static inline int32_t motion_coord(const motion_t *m, int a) {
  return m->ax[a].homed ? rail_steps_to_coord(m->ax[a].pos, m->ax[a].span) : -1;
}

// Latches the first fault; later ones are ignored until reset.
static inline void motion_fault(motion_t *m, motion_fault_t f, int axis) {
  if (m->fault != MF_NONE) return;
  for (int a = 0; a < MOTION_AXES; a++) {
    m->ax[a].homedAtFault = m->ax[a].homed;
    m->ax[a].encAtFault   = m->ax[a].enc;
    m->ax[a].homed = false;
  }
  m->fault = f; m->faultAxis = axis; m->mode = MM_FAULT;
  m->homeVel = m->homeAcc = 0;
  m->pathLen = m->pathIdx = 0;
}

static inline void motion_do_step(motion_t *m, int a, int dir) {
  motion_axis_t *x = &m->ax[a];
  x->pos += dir;
  if (x->skipInject > 0) { x->skipInject--; return; }   // simulated skip: motor didn't move
  x->enc += dir;
  if (m->step) m->step(m->ctx, a, dir);
}

static inline void motion_home_next_axis(motion_t *m) {
  m->homePhase = HOME_SEEK_MIN; m->homeVel = m->homeAcc = 0; m->homeSteps = 0;
}

// Plans one move (no checks). coord[a] < 0 leaves that axis where it is.
static inline void motion_begin_move(motion_t *m, const int32_t coord[MOTION_AXES]) {
  float len = 0.0f;
  for (int a = 0; a < MOTION_AXES; a++) {
    int32_t tgt = coord[a] >= 0 ? rail_coord_to_steps(coord[a], m->ax[a].span) : m->ax[a].pos;
    m->moveStart[a] = m->ax[a].pos;
    m->moveDist[a]  = tgt - m->ax[a].pos;
    float d = fabsf((float)m->moveDist[a]);
    if (d > len) len = d;
  }
  profile_plan(&m->prof, len, m->cfg.vmax, m->cfg.amax);
  m->moveT = 0.0f; m->moveLen = len; m->mode = MM_MOVE;
}

static inline void motion_begin_path_point(motion_t *m) {   // starts waypoint pathIdx
  int32_t c[MOTION_AXES] = { m->pathPts[m->pathIdx][0], m->pathPts[m->pathIdx][1], -1 };
  m->pathIdx++;
  motion_begin_move(m, c);
}

static inline void motion_tick(motion_t *m, const motion_inputs_t *in, float dt) {
  if (in->estop) motion_fault(m, MF_ESTOP, -1);
  for (int a = 0; a < MOTION_AXES; a++) {
    int32_t err = m->ax[a].pos - m->ax[a].enc;
    if (err > m->cfg.skipThreshold || -err > m->cfg.skipThreshold) motion_fault(m, MF_SKIP, a);
  }

  switch (m->mode) {
    case MM_IDLE:
    case MM_FAULT:
      break;

    case MM_MOVE: {
      m->moveT += dt;
      float frac = m->moveLen > 0.0f ? profile_pos(&m->prof, m->moveT) / m->moveLen : 1.0f;
      bool done = m->moveT >= m->prof.total;
      for (int a = 0; a < MOTION_AXES && m->mode == MM_MOVE; a++) {
        int32_t target = m->moveStart[a] + (int32_t)lroundf((float)m->moveDist[a] * frac);
        for (int i = 0; i < 2 && m->ax[a].pos != target; i++) {     // at most 2 steps per tick
          int dir = target > m->ax[a].pos ? 1 : -1;
          if ((dir < 0 && in->limMin[a]) || (dir > 0 && in->limMax[a])) { motion_fault(m, MF_LIMIT, a); break; }
          motion_do_step(m, a, dir);
        }
        if (m->ax[a].pos != target) done = false;
      }
      if (m->mode == MM_MOVE && done) {
        if (m->pathIdx < m->pathLen) motion_begin_path_point(m);   // next corner of the path
        else { m->mode = MM_IDLE; m->pathLen = m->pathIdx = 0; }
      }
    } break;

    case MM_HOMING: {
      int a = m->homeQueue[m->homeQIdx];
      motion_axis_t *x = &m->ax[a];
      home_event_t ev = home_check(m->homePhase, in->limMin[a], in->limMax[a],
                                   x->pos, m->homeSteps, m->cfg.homeMaxSteps);
      if (ev == HOME_FOUND_MIN) {
        x->pos = 0; x->enc = 0;
        motion_home_next_axis(m);
        m->homePhase = HOME_SEEK_MAX;
        break;
      }
      if (ev == HOME_FOUND_MAX) {
        x->span = x->pos; x->homed = true;
        if (++m->homeQIdx < m->homeQLen) { motion_home_next_axis(m); break; }
        m->mode = MM_IDLE;
        if (m->cfg.homeReturn) {             // back to 0,0: the starting position
          int32_t c[MOTION_AXES] = { -1, -1, -1 };
          bool any = false;
          for (int i = 0; i < m->homeQLen; i++)
            if (m->homeQueue[i] != AXIS_Z) { c[m->homeQueue[i]] = 0; any = true; }
          if (any) motion_begin_move(m, c);
        }
        break;
      }
      if (ev == HOME_ERR_SHORT)   { motion_fault(m, MF_RAIL_SHORT, a);   break; }
      if (ev == HOME_ERR_TIMEOUT) { motion_fault(m, MF_HOME_TIMEOUT, a); break; }
      int dir = (m->homePhase == HOME_SEEK_MIN) ? -1 : 1;
      m->homeVel = fminf(m->homeVel + m->cfg.amax * dt, m->cfg.homeSpeed);
      m->homeAcc += m->homeVel * dt;
      while (m->homeAcc >= 1.0f) { motion_do_step(m, a, dir); m->homeAcc -= 1.0f; m->homeSteps++; }
    } break;
  }
}

// Move to rail coordinates (0..RAIL_COORD_MAX); coord[a] < 0 leaves that axis where it is.
static inline move_result_t motion_start_move(motion_t *m, const int32_t coord[MOTION_AXES]) {
  if (m->mode != MM_IDLE) return MOVE_BUSY;
  for (int a = 0; a < MOTION_AXES; a++)
    if (coord[a] >= 0 && !m->ax[a].homed) return MOVE_NOT_HOMED;
  motion_begin_move(m, coord);
  return MOVE_OK;
}

// Drive X / Y through n waypoints (rail coordinates 0..RAIL_COORD_MAX), in order,
// one straight move each. Needs X and Y homed. Faults and stops drop the rest.
static inline move_result_t motion_start_path(motion_t *m, const int32_t pts[][2], int n) {
  if (m->mode != MM_IDLE) return MOVE_BUSY;
  if (n < 1 || n > MOTION_PATH_MAX) return MOVE_BAD_PATH;
  for (int i = 0; i < n; i++)
    if (pts[i][0] < 0 || pts[i][0] > RAIL_COORD_MAX || pts[i][1] < 0 || pts[i][1] > RAIL_COORD_MAX)
      return MOVE_BAD_PATH;
  if (!m->ax[AXIS_X].homed || !m->ax[AXIS_Y].homed) return MOVE_NOT_HOMED;
  for (int i = 0; i < n; i++) { m->pathPts[i][0] = pts[i][0]; m->pathPts[i][1] = pts[i][1]; }
  m->pathLen = n; m->pathIdx = 0;
  motion_begin_path_point(m);
  return MOVE_OK;
}

// Home the listed axes one after another, in that order. They lose their homing now.
static inline bool motion_start_homing(motion_t *m, const int *axes, int n) {
  if (m->mode != MM_IDLE || n < 1 || n > MOTION_AXES) return false;
  for (int i = 0; i < n; i++) if (axes[i] < 0 || axes[i] >= MOTION_AXES) return false;
  for (int i = 0; i < n; i++) { m->homeQueue[i] = axes[i]; m->ax[axes[i]].homed = false; }
  m->homeQLen = n; m->homeQIdx = 0;
  motion_home_next_axis(m);
  m->mode = MM_HOMING;
  return true;
}

static inline void motion_stop(motion_t *m) {   // stops a move or homing (the homing axis stays unhomed)
  if (m->mode != MM_FAULT) m->mode = MM_IDLE;
  m->pathLen = m->pathIdx = 0;
}

// "Reset and keep homing": only after an E-stop, for axes that were homed when
// it hit and whose encoder still agrees (rail.h keep_home_check).
static inline keep_home_t motion_keep_check(const motion_t *m) {
  if (!(m->mode == MM_FAULT && m->fault == MF_ESTOP)) return KEEP_NOT_ESTOP;
  bool any = false;
  for (int a = 0; a < MOTION_AXES; a++) {
    const motion_axis_t *x = &m->ax[a];
    if (!x->homedAtFault) continue;
    any = true;
    keep_home_t k = keep_home_check(true, true, x->encAtFault, x->enc, x->pos, m->cfg.skipThreshold);
    if (k != KEEP_OK) return k;
  }
  return any ? KEEP_OK : KEEP_NOT_HOMED;
}

// Clears the fault (and stops any motion). keep = false: every axis must be
// homed again. keep = true: refused (returns why) unless motion_keep_check passes.
static inline keep_home_t motion_reset(motion_t *m, bool keep) {
  if (keep) {
    keep_home_t k = motion_keep_check(m);
    if (k != KEEP_OK) return k;
  }
  for (int a = 0; a < MOTION_AXES; a++) {
    motion_axis_t *x = &m->ax[a];
    bool kept = keep && x->homedAtFault;
    if (!kept) x->enc = x->pos;            // true position unknown: re-home this axis
    x->homed = kept; x->homedAtFault = false; x->skipInject = 0;
  }
  m->fault = MF_NONE; m->faultAxis = -1; m->mode = MM_IDLE;
  m->pathLen = m->pathIdx = 0;
  return KEEP_OK;
}
