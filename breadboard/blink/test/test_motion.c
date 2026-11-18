/*
  test_motion.c - PC test for main/motion.h (3-axis controller).
  Build and run from breadboard/blink:
    gcc -Wall -Wextra -Werror -std=c11 -I main test/test_motion.c -o test_motion -lm && ./test_motion

  A simulated machine: each axis has an absolute carriage position and a MIN /
  MAX switch that is pressed while the carriage is at or past it. The step
  callback moves the carriage, like the coils do on the board. motion_tick runs
  every simulated 1 ms with the switch states, same as main.c.
*/
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include "motion.h"

static int failures;
#define CHECK(cond, ...) do { if (!(cond)) { failures++; if (failures <= 30) { \
  printf("FAIL line %d: ", __LINE__); printf(__VA_ARGS__); printf("\n"); } } } while (0)

static const motion_config_t CFG = { .vmax = 600.0f, .amax = 1200.0f, .homeSpeed = 300.0f,
                                     .homeMaxSteps = 3 * 4096, .skipThreshold = 4,
                                     .homeReturn = true };

// ----------------------------- Simulated machine --------------------------
typedef struct {
  int32_t abs[MOTION_AXES], minAt[MOTION_AXES], maxAt[MOTION_AXES];
  int stepsThisTick[MOTION_AXES], maxStepsPerTick[MOTION_AXES];
  long hwSteps[MOTION_AXES];
  bool estop, forceMin[MOTION_AXES], forceMax[MOTION_AXES];
  bool stepWhileFault;
  motion_t *m;
} sim_t;

static void sim_step(void *ctx, int a, int dir) {
  sim_t *s = ctx;
  if (dir != 1 && dir != -1) { failures++; printf("FAIL: step dir %d\n", dir); }
  if (s->m->mode == MM_FAULT) s->stepWhileFault = true;
  s->abs[a] += dir; s->hwSteps[a]++; s->stepsThisTick[a]++;
}

static void sim_init(sim_t *s, motion_t *m, const int32_t start[3], const int32_t minAt[3], const int32_t maxAt[3]) {
  memset(s, 0, sizeof *s);
  s->m = m;
  memcpy(s->abs, start, sizeof s->abs); memcpy(s->minAt, minAt, sizeof s->minAt); memcpy(s->maxAt, maxAt, sizeof s->maxAt);
  motion_init(m, &CFG, sim_step, s);
}

static motion_inputs_t sim_inputs(const sim_t *s) {
  motion_inputs_t in = { .estop = s->estop };
  for (int a = 0; a < MOTION_AXES; a++) {
    in.limMin[a] = s->abs[a] <= s->minAt[a] || s->forceMin[a];
    in.limMax[a] = s->abs[a] >= s->maxAt[a] || s->forceMax[a];
  }
  return in;
}

static void sim_tick(sim_t *s) {
  memset(s->stepsThisTick, 0, sizeof s->stepsThisTick);
  motion_inputs_t in = sim_inputs(s);
  motion_tick(s->m, &in, 0.001f);
  for (int a = 0; a < MOTION_AXES; a++)
    if (s->stepsThisTick[a] > s->maxStepsPerTick[a]) s->maxStepsPerTick[a] = s->stepsThisTick[a];
}

// Tick until the mode leaves `mode` (or limit). Returns ticks run.
static int run_while(sim_t *s, motion_mode_t mode, int limit) {
  int n = 0;
  while (s->m->mode == mode && n < limit) { sim_tick(s); n++; }
  return n;
}

static const int32_t RAIL_MIN[3] = { 0, 0, 0 };
static const int32_t RAIL_MAX[3] = { 2900, 2100, 1500 };   // X, Y, Z switch positions
static const int ALL[3] = { AXIS_Z, AXIS_X, AXIS_Y };

static void home_all(sim_t *s, motion_t *m) {   // homes Z, X, Y; X / Y then drive back to 0,0
  CHECK(motion_start_homing(m, ALL, 3), "start homing");
  run_while(s, MM_HOMING, 200000);
  run_while(s, MM_MOVE, 100000);
}

// ----------------------------- Tests --------------------------------------
static void test_profile(void) {
  const float dists[] = { 0, 1, 2, 3, 7, 50, 333, 1450, 4096, 12288 };
  for (size_t i = 0; i < sizeof dists / sizeof dists[0]; i++) {
    profile_t p; profile_plan(&p, dists[i], CFG.vmax, CFG.amax);
    float prev = 0, peakA = 0, peakV = 0;
    int32_t prevStep = 0;
    for (int k = 1; ; k++) {
      float t = (float)(k * 0.001);
      float x = profile_pos(&p, t), v = (x - prev) / 0.001f;
      CHECK(x >= prev - 1e-3f, "dist %.0f: reverses at t=%.3f (%.4f < %.4f)", dists[i], t, x, prev);
      int32_t st = (int32_t)lroundf(x);
      CHECK(st - prevStep <= 1, "dist %.0f: %ld steps in one tick", dists[i], (long)(st - prevStep));
      if (v > peakV) peakV = v;
      // Acceleration over 10 ms (a 1 ms second difference is only float rounding noise)
      const double h = 0.01;
      double tt = k * 0.001;
      if (tt > h && tt + h < p.total) {
        double a = ((double)profile_pos(&p, (float)(tt + h)) - 2.0 * profile_pos(&p, (float)tt)
                    + profile_pos(&p, (float)(tt - h))) / (h * h);
        if (fabs(a) > peakA) peakA = (float)fabs(a);
      }
      prev = x; prevStep = st;
      if (t >= p.total) break;
      if (k > 100000) { CHECK(0, "dist %.0f: never ends", dists[i]); break; }
    }
    CHECK(profile_pos(&p, p.total) == dists[i], "dist %.0f: ends at %.4f", dists[i], profile_pos(&p, p.total));
    CHECK(profile_pos(&p, p.total + 5) == dists[i], "dist %.0f: stays at the end", dists[i]);
    CHECK(peakV <= CFG.vmax * 1.01f, "dist %.0f: peak speed %.1f > vmax", dists[i], peakV);
    CHECK(peakA <= CFG.amax * 1.05f, "dist %.0f: peak accel %.1f > amax", dists[i], peakA);
    if (dists[i] >= 1450) CHECK(peakA >= CFG.amax * 0.95f, "dist %.0f: peak accel %.1f, want ~amax", dists[i], peakA);
    if (dists[i] >= 4096) CHECK(peakV >= CFG.vmax * 0.99f, "dist %.0f: never reaches vmax (%.1f)", dists[i], peakV);
  }
  profile_t p; profile_plan(&p, -500, CFG.vmax, CFG.amax);
  CHECK(p.dist == 500, "negative distance planned as its size");
}

static void test_names(void) {
  CHECK(motion_axis_name(AXIS_X) == 'X' && motion_axis_name(AXIS_Z) == 'Z' && motion_axis_name(-1) == '-', "axis names");
  CHECK(!strcmp(motion_mode_name(MM_HOMING), "HOME") && !strcmp(motion_mode_name(MM_FAULT), "FAULT"), "mode names");
  CHECK(!strcmp(motion_fault_name(MF_HOME_TIMEOUT), "HOME TIMEOUT") && !strcmp(motion_fault_name(MF_NONE), "NONE"), "fault names");
}

static void test_homing_all(void) {
  motion_t m; sim_t s;
  const int32_t start[3] = { 1000, 700, 400 };
  sim_init(&s, &m, start, RAIL_MIN, RAIL_MAX);
  CHECK(motion_coord(&m, AXIS_X) == -1, "unhomed coord is -1");

  int32_t c[3] = { 100, 100, -1 };
  CHECK(motion_start_move(&m, c) == MOVE_NOT_HOMED, "move refused before homing");

  CHECK(motion_start_homing(&m, ALL, 3), "start homing all");
  CHECK(!motion_start_homing(&m, ALL, 3), "can't restart homing while homing");
  CHECK(motion_start_move(&m, c) == MOVE_BUSY, "move refused while homing");

  // Order: Z first, and only the homing axis moves
  int order[3], nOrder = 0, lastAxis = -1;
  int32_t before[3];
  while (m.mode == MM_HOMING) {
    int a = motion_homing_axis(&m);
    if (a != lastAxis) { order[nOrder++] = a; lastAxis = a; }
    memcpy(before, s.abs, sizeof before);
    sim_tick(&s);
    for (int o = 0; o < 3; o++) if (o != a) CHECK(s.abs[o] == before[o], "axis %d moved while %d homes", o, a);
  }
  CHECK(nOrder == 3 && order[0] == AXIS_Z && order[1] == AXIS_X && order[2] == AXIS_Y, "homing order Z, X, Y");
  // Right when the last MAX is found: every axis sits on its MAX switch, and the
  // drive back to 0,0 has just been planned
  CHECK(m.mode == MM_MOVE && m.fault == MF_NONE, "return move after homing (mode %d fault %d)", m.mode, m.fault);
  for (int a = 0; a < 3; a++) {
    CHECK(m.ax[a].homed, "axis %d homed", a);
    CHECK(m.ax[a].span == RAIL_MAX[a] - RAIL_MIN[a], "axis %d span %ld", a, (long)m.ax[a].span);
    CHECK(s.abs[a] == RAIL_MAX[a] && m.ax[a].pos == m.ax[a].span, "axis %d found its MAX switch", a);
    CHECK(s.maxStepsPerTick[a] <= 1, "axis %d: %d steps in one homing tick", a, s.maxStepsPerTick[a]);
  }
  CHECK(motion_homing_axis(&m) == -1, "no homing axis after homing");
  CHECK(motion_start_move(&m, c) == MOVE_BUSY, "busy during the return");
  run_while(&s, MM_MOVE, 100000);
  CHECK(m.mode == MM_IDLE && m.fault == MF_NONE, "idle at the start position");
  CHECK(s.abs[AXIS_X] == 0 && s.abs[AXIS_Y] == 0 && motion_coord(&m, AXIS_X) == 0 && motion_coord(&m, AXIS_Y) == 0,
        "homing ends at X,Y = 0,0 (at %ld,%ld)", (long)s.abs[AXIS_X], (long)s.abs[AXIS_Y]);
  CHECK(s.abs[AXIS_Z] == RAIL_MAX[AXIS_Z] && motion_coord(&m, AXIS_Z) == RAIL_COORD_MAX, "Z stays raised");
  for (int a = 0; a < 3; a++) CHECK(m.ax[a].enc == m.ax[a].pos && m.ax[a].homed, "axis %d encoder agrees, still homed", a);

  // Without homeReturn the axes stay at their MAX ends
  motion_t m2; sim_t s2;
  sim_init(&s2, &m2, start, RAIL_MIN, RAIL_MAX);
  m2.cfg.homeReturn = false;
  motion_start_homing(&m2, ALL, 3);
  run_while(&s2, MM_HOMING, 200000);
  CHECK(m2.mode == MM_IDLE && s2.abs[AXIS_X] == RAIL_MAX[AXIS_X] && s2.abs[AXIS_Y] == RAIL_MAX[AXIS_Y],
        "homeReturn off: idle at the MAX ends");
}

static void test_home_one_axis(void) {
  motion_t m; sim_t s;
  const int32_t start[3] = { 1000, 700, 400 };
  sim_init(&s, &m, start, RAIL_MIN, RAIL_MAX);
  int y = AXIS_Y;
  CHECK(motion_start_homing(&m, &y, 1), "home Y only");
  run_while(&s, MM_HOMING, 100000);
  run_while(&s, MM_MOVE, 100000);
  CHECK(m.ax[AXIS_Y].homed && !m.ax[AXIS_X].homed && !m.ax[AXIS_Z].homed, "only Y homed");
  CHECK(s.abs[AXIS_Y] == 0 && m.mode == MM_IDLE, "h y: Y returns to 0");
  CHECK(s.abs[AXIS_X] == 1000 && s.abs[AXIS_Z] == 400, "X and Z didn't move");
  int32_t c[3] = { -1, -1, 2048 };
  CHECK(motion_start_move(&m, c) == MOVE_NOT_HOMED, "Z move refused, Z not homed");
  int32_t cy[3] = { -1, 0, -1 };
  CHECK(motion_start_move(&m, cy) == MOVE_OK, "Y-only move allowed");
  run_while(&s, MM_MOVE, 100000);
  CHECK(s.abs[AXIS_Y] == 0 && m.mode == MM_IDLE, "Y at 0");

  int bad[2] = { AXIS_X, 7 };
  CHECK(!motion_start_homing(&m, bad, 2) && m.mode == MM_IDLE, "bad axis rejected");
  CHECK(!motion_start_homing(&m, ALL, 0) && !motion_start_homing(&m, ALL, 4), "bad count rejected");
}

// Straight line: every tick, each axis has covered the same fraction of its distance.
static void check_line_move(sim_t *s, motion_t *m, int32_t cx, int32_t cy, int32_t cz, const char *name) {
  int32_t c[3] = { cx, cy, cz }, st[3], tg[3];
  for (int a = 0; a < 3; a++) {
    st[a] = s->abs[a];
    tg[a] = c[a] >= 0 ? rail_coord_to_steps(c[a], m->ax[a].span) : st[a];
  }
  memset(s->maxStepsPerTick, 0, sizeof s->maxStepsPerTick);
  CHECK(motion_start_move(m, c) == MOVE_OK, "%s: start", name);
  int lead = 0;
  for (int a = 1; a < 3; a++) if (abs(tg[a] - st[a]) > abs(tg[lead] - st[lead])) lead = a;
  float worst = 0;
  int ticks = 0;
  while (m->mode == MM_MOVE && ticks < 100000) {
    sim_tick(s); ticks++;
    if (tg[lead] == st[lead]) continue;
    float f = (float)(s->abs[lead] - st[lead]) / (float)(tg[lead] - st[lead]);
    for (int a = 0; a < 3; a++) {
      float expect = st[a] + f * (float)(tg[a] - st[a]);
      float off = fabsf((float)s->abs[a] - expect);
      if (off > worst) worst = off;
      // never past the target, never backwards
      int32_t lo = st[a] < tg[a] ? st[a] : tg[a], hi = st[a] < tg[a] ? tg[a] : st[a];
      CHECK(s->abs[a] >= lo && s->abs[a] <= hi, "%s: axis %d outside start..target (%ld)", name, a, (long)s->abs[a]);
    }
  }
  CHECK(m->mode == MM_IDLE, "%s: ends idle (mode %d, fault %d)", name, m->mode, m->fault);
  for (int a = 0; a < 3; a++) {
    CHECK(s->abs[a] == tg[a] && m->ax[a].pos == tg[a], "%s: axis %d at %ld, want %ld", name, a, (long)s->abs[a], (long)tg[a]);
    CHECK(s->maxStepsPerTick[a] <= 1, "%s: axis %d made %d steps in one tick", name, a, s->maxStepsPerTick[a]);
  }
  CHECK(worst <= 1.5f, "%s: off the straight line by %.2f steps", name, worst);
}

static void test_moves(void) {
  motion_t m; sim_t s;
  const int32_t start[3] = { 1000, 700, 400 };
  sim_init(&s, &m, start, RAIL_MIN, RAIL_MAX);
  home_all(&s, &m);

  check_line_move(&s, &m, 0, 0, -1, "diagonal to 0,0");
  CHECK(s.abs[AXIS_Z] == RAIL_MAX[AXIS_Z], "g leaves Z raised");
  check_line_move(&s, &m, 4096, 1024, -1, "shallow line");
  check_line_move(&s, &m, 1000, 4096, -1, "steep line");
  check_line_move(&s, &m, 1000, 4096, -1, "zero-length move");
  check_line_move(&s, &m, 2048, 4096, -1, "X only");
  check_line_move(&s, &m, -1, -1, 0, "Z down");
  CHECK(s.abs[AXIS_Z] == 0 && motion_coord(&m, AXIS_Z) == 0, "attachment down");
  check_line_move(&s, &m, 2048, 2048, 2048, "all three");
  CHECK(motion_coord(&m, AXIS_X) == 2048 && s.abs[AXIS_X] == 1450, "X coord 2048 = middle step 1450");
  CHECK(motion_coord(&m, AXIS_Y) == 2048 && s.abs[AXIS_Y] == 1050, "Y coord 2048 = middle step 1050");

  // Motion is consistent with the hardware: every commanded step reached the motor
  for (int a = 0; a < 3; a++) CHECK(m.ax[a].enc == m.ax[a].pos, "axis %d encoder agrees", a);
  CHECK(!s.stepWhileFault, "no steps while faulted");

  // A long move takes the S-curve time: 2900 steps at 600 steps/s + ramps ~ 5.6 s
  int32_t to0[3] = { 0, -1, -1 };
  motion_start_move(&m, to0);
  int ticks = run_while(&s, MM_MOVE, 100000);
  CHECK(ticks > 2000 && ticks < 4000, "X 1450 steps took %d ms", ticks);

  // Stop mid-move: stays put and is idle
  int32_t toMax[3] = { 4096, 4096, -1 };
  motion_start_move(&m, toMax);
  for (int i = 0; i < 800; i++) sim_tick(&s);
  motion_stop(&m);
  int32_t xs = s.abs[AXIS_X];
  for (int i = 0; i < 200; i++) sim_tick(&s);
  CHECK(m.mode == MM_IDLE && s.abs[AXIS_X] == xs && xs > 0 && xs < RAIL_MAX[AXIS_X], "x stops a move (X at %ld)", (long)xs);
  CHECK(m.ax[AXIS_X].homed, "stop keeps homing");
}

static void test_limit_and_skip(void) {
  motion_t m; sim_t s;
  const int32_t start[3] = { 1000, 700, 400 };
  sim_init(&s, &m, start, RAIL_MIN, RAIL_MAX);
  home_all(&s, &m);

  // Y MAX switch pressed early while moving toward it -> LIMIT Y, everything stops
  int32_t c[3] = { 0, 0, -1 };
  check_line_move(&s, &m, 0, 0, -1, "back to 0,0");
  int32_t up[3] = { 4096, 4096, -1 };
  CHECK(motion_start_move(&m, up) == MOVE_OK, "move up");
  for (int i = 0; i < 1000; i++) sim_tick(&s);
  s.forceMax[AXIS_Y] = true;
  int32_t y0 = s.abs[AXIS_Y];
  for (int i = 0; i < 50 && m.mode == MM_MOVE; i++) sim_tick(&s);   // faults on Y's next step
  CHECK(s.abs[AXIS_Y] == y0, "Y never stepped into the pressed MAX switch");
  CHECK(m.mode == MM_FAULT && m.fault == MF_LIMIT && m.faultAxis == AXIS_Y, "LIMIT Y (fault %d axis %d)", m.fault, m.faultAxis);
  CHECK(!m.ax[0].homed && !m.ax[1].homed && !m.ax[2].homed, "every axis loses homing");
  int32_t frozen[3]; memcpy(frozen, s.abs, sizeof frozen);
  s.forceMax[AXIS_Y] = false;
  for (int i = 0; i < 500; i++) sim_tick(&s);
  CHECK(!memcmp(frozen, s.abs, sizeof frozen), "nothing moves while faulted");
  CHECK(motion_keep_check(&m) == KEEP_NOT_ESTOP, "LIMIT can't keep homing");
  CHECK(motion_reset(&m, true) == KEEP_NOT_ESTOP && m.mode == MM_FAULT, "r k refused after LIMIT");
  CHECK(motion_start_move(&m, c) == MOVE_BUSY, "no moves while faulted");
  CHECK(!motion_start_homing(&m, ALL, 3), "no homing while faulted");
  CHECK(motion_reset(&m, false) == KEEP_OK && m.mode == MM_IDLE && m.fault == MF_NONE, "r clears it");
  CHECK(motion_start_move(&m, c) == MOVE_NOT_HOMED, "must home again");

  // A limit pressed on the side we're NOT moving toward doesn't stop the move
  home_all(&s, &m);
  check_line_move(&s, &m, 0, 0, -1, "to the MIN corner");
  CHECK(sim_inputs(&s).limMin[AXIS_X] && sim_inputs(&s).limMin[AXIS_Y], "X and Y MIN switches pressed at 0,0");
  check_line_move(&s, &m, 2048, 1024, -1, "away from pressed MIN switches");
  // A limit on an axis that isn't moving is ignored too (Z sits on its MAX switch after homing)
  CHECK(sim_inputs(&s).limMax[AXIS_Z], "Z MAX pressed");
  check_line_move(&s, &m, 0, 0, -1, "Z MAX held during an X/Y move");

  // Simulated skip on Z: commanded steps that never reach the motor
  m.ax[AXIS_Z].skipInject = 8;
  int32_t zdown[3] = { -1, -1, 0 };
  CHECK(motion_start_move(&m, zdown) == MOVE_OK, "Z move with skips");
  run_while(&s, MM_MOVE, 100000);
  CHECK(m.mode == MM_FAULT && m.fault == MF_SKIP && m.faultAxis == AXIS_Z, "SKIP Z (fault %d axis %d)", m.fault, m.faultAxis);
  CHECK(abs(m.ax[AXIS_Z].pos - m.ax[AXIS_Z].enc) == CFG.skipThreshold + 1, "faults right after the threshold (err %ld)",
        (long)(m.ax[AXIS_Z].pos - m.ax[AXIS_Z].enc));
  CHECK(motion_keep_check(&m) == KEEP_NOT_ESTOP, "SKIP can't keep homing");
  motion_reset(&m, false);
  for (int i = 0; i < 10; i++) sim_tick(&s);
  CHECK(m.mode == MM_IDLE && m.ax[AXIS_Z].skipInject == 0, "reset clears the mismatch (no instant re-fault)");

  // Skips within the threshold are tolerated
  home_all(&s, &m);                                   // X at 0
  m.ax[AXIS_X].skipInject = CFG.skipThreshold;
  int32_t xMax[3] = { RAIL_COORD_MAX, -1, -1 };
  CHECK(motion_start_move(&m, xMax) == MOVE_OK, "move with 4 skips");
  run_while(&s, MM_MOVE, 100000);
  CHECK(m.mode == MM_IDLE && m.fault == MF_NONE, "4 skips (= threshold) don't fault");
  CHECK(m.ax[AXIS_X].pos == RAIL_MAX[AXIS_X] && s.abs[AXIS_X] == RAIL_MAX[AXIS_X] - CFG.skipThreshold,
        "motor really ended %ld steps short", (long)(RAIL_MAX[AXIS_X] - s.abs[AXIS_X]));
}

static void test_estop_keep(void) {
  motion_t m; sim_t s;
  const int32_t start[3] = { 1000, 700, 400 };
  sim_init(&s, &m, start, RAIL_MIN, RAIL_MAX);
  home_all(&s, &m);

  // E-stop mid-move
  int32_t c[3] = { 1000, 3000, 1000 };
  motion_start_move(&m, c);
  for (int i = 0; i < 700; i++) sim_tick(&s);
  s.estop = true;
  sim_tick(&s);
  CHECK(m.mode == MM_FAULT && m.fault == MF_ESTOP && m.faultAxis == -1, "E-stop latches");
  int32_t at[3]; memcpy(at, s.abs, sizeof at);
  for (int i = 0; i < 300; i++) sim_tick(&s);
  CHECK(!memcmp(at, s.abs, sizeof at), "stopped dead");
  CHECK(motion_keep_check(&m) == KEEP_OK, "keep offered after E-stop on homed axes");
  s.estop = false;
  CHECK(motion_reset(&m, true) == KEEP_OK, "r k accepted");
  for (int a = 0; a < 3; a++) CHECK(m.ax[a].homed && m.ax[a].pos == at[a], "axis %d keeps homing and position", a);
  check_line_move(&s, &m, 1000, 3000, 1000, "finish the move after keep");

  // Encoder moved while stopped (on the gantry: someone pushed the carriage)
  s.estop = true; sim_tick(&s); s.estop = false;
  m.ax[AXIS_Y].enc += 10;
  CHECK(motion_keep_check(&m) == KEEP_MOVED, "moved encoder refuses keep");
  CHECK(motion_reset(&m, true) == KEEP_MOVED && m.mode == MM_FAULT, "r k refused, still faulted");
  motion_reset(&m, false);
  CHECK(m.ax[AXIS_Y].enc == m.ax[AXIS_Y].pos && !m.ax[AXIS_Y].homed, "plain r resyncs and unhomes");

  // E-stop while homing the first axis: nothing homed -> no keep
  sim_init(&s, &m, start, RAIL_MIN, RAIL_MAX);
  motion_start_homing(&m, ALL, 3);
  for (int i = 0; i < 200; i++) sim_tick(&s);
  s.estop = true; sim_tick(&s); s.estop = false;
  CHECK(motion_keep_check(&m) == KEEP_NOT_HOMED, "E-stop during first homing: no keep");

  // E-stop while homing Y after Z and X are done: Z and X kept, Y must re-home
  sim_init(&s, &m, start, RAIL_MIN, RAIL_MAX);
  motion_start_homing(&m, ALL, 3);
  while (motion_homing_axis(&m) != AXIS_Y) sim_tick(&s);
  for (int i = 0; i < 300; i++) sim_tick(&s);
  s.estop = true; sim_tick(&s); s.estop = false;
  CHECK(motion_keep_check(&m) == KEEP_OK, "keep offered for the axes already homed");
  CHECK(motion_reset(&m, true) == KEEP_OK, "r k");
  CHECK(m.ax[AXIS_Z].homed && m.ax[AXIS_X].homed && !m.ax[AXIS_Y].homed, "Z, X kept; Y not homed");
  for (int i = 0; i < 20; i++) sim_tick(&s);
  CHECK(m.mode == MM_IDLE, "no fault after a partial keep");
  int32_t xz[3] = { 0, -1, 0 }, xy[3] = { 0, 0, -1 };
  CHECK(motion_start_move(&m, xz) == MOVE_OK, "X/Z move allowed");
  run_while(&s, MM_MOVE, 100000);
  CHECK(motion_start_move(&m, xy) == MOVE_NOT_HOMED, "Y move needs homing");

  // E-stop held: keeps faulting, even right after a reset
  s.estop = true; sim_tick(&s);
  motion_reset(&m, false);
  sim_tick(&s);
  CHECK(m.mode == MM_FAULT && m.fault == MF_ESTOP, "held E-stop re-latches");
  s.estop = false;
}

// USE_Z_AXIS 0 in main.c: "h" homes only X then Y. Z must never move or need homing for X/Y.
static void test_xy_only(void) {
  motion_t m; sim_t s;
  const int32_t start[3] = { 1000, 700, 0 };   // Z count and carriage agree (both 0, never homed)
  const int32_t zMin[3] = { 0, 0, -500 };       // Z off its switch, so nothing about Z is pressed
  sim_init(&s, &m, start, zMin, RAIL_MAX);
  const int XY[2] = { AXIS_X, AXIS_Y };
  CHECK(motion_start_homing(&m, XY, 2), "home X, Y");
  int order[2], n = 0, last = -1;
  while (m.mode == MM_HOMING) {
    int a = motion_homing_axis(&m);
    if (a != last && n < 2) { order[n++] = a; last = a; }
    sim_tick(&s);
  }
  CHECK(n == 2 && order[0] == AXIS_X && order[1] == AXIS_Y, "order X, Y");
  run_while(&s, MM_MOVE, 100000);
  CHECK(s.abs[AXIS_X] == 0 && s.abs[AXIS_Y] == 0, "X, Y back at 0,0");
  CHECK(m.mode == MM_IDLE && m.ax[AXIS_X].homed && m.ax[AXIS_Y].homed && !m.ax[AXIS_Z].homed, "X, Y homed, Z not");
  CHECK(s.abs[AXIS_Z] == 0 && s.hwSteps[AXIS_Z] == 0, "Z never stepped");
  check_line_move(&s, &m, 1024, 3072, -1, "X/Y move without Z homed");
  int32_t z[3] = { -1, -1, 0 };
  CHECK(motion_start_move(&m, z) == MOVE_NOT_HOMED, "Z move refused");
  CHECK(s.hwSteps[AXIS_Z] == 0, "Z still never stepped");
  // E-stop + keep restores X and Y
  s.estop = true; sim_tick(&s); s.estop = false;
  CHECK(motion_reset(&m, true) == KEEP_OK && m.ax[AXIS_X].homed && m.ax[AXIS_Y].homed && !m.ax[AXIS_Z].homed,
        "keep after E-stop restores X and Y only");
}

// One path: the carriage must visit every waypoint in order, in straight lines,
// stopping at each (the S-curve of each segment ends at zero speed).
static void test_paths(void) {
  motion_t m; sim_t s;
  const int32_t start[3] = { 1000, 700, 400 };
  sim_init(&s, &m, start, RAIL_MIN, RAIL_MAX);
  home_all(&s, &m);                                  // at 0,0
  const int32_t L[4][2] = { { 2048, 0 }, { 2048, 2048 }, { 4096, 2048 }, { 4096, 4096 } };   // grid-line path
  const int32_t steps[4][2] = { { 1450, 0 }, { 1450, 1050 }, { 2900, 1050 }, { 2900, 2100 } };
  CHECK(motion_start_path(&m, L, 4) == MOVE_OK, "start path");
  CHECK(m.pathLen == 4 && m.pathIdx == 1, "first waypoint started");
  CHECK(motion_start_path(&m, L, 4) == MOVE_BUSY, "second path refused while busy");
  int visited = 0, ticks = 0;
  int32_t prev[2] = { 0, 0 };
  while (m.mode == MM_MOVE && ticks++ < 200000) {
    int seg = m.pathIdx - 1;                         // segment being driven
    sim_tick(&s);
    // between waypoints only one axis changes (grid-line segments)
    int32_t from[2] = { seg == 0 ? 0 : steps[seg - 1][0], seg == 0 ? 0 : steps[seg - 1][1] };
    if (from[0] == steps[seg][0]) CHECK(s.abs[AXIS_X] == from[0] || m.pathIdx - 1 != seg, "seg %d: X moved on a Y segment", seg);
    if (from[1] == steps[seg][1]) CHECK(s.abs[AXIS_Y] == from[1] || m.pathIdx - 1 != seg, "seg %d: Y moved on an X segment", seg);
    if (s.abs[AXIS_X] == steps[visited][0] && s.abs[AXIS_Y] == steps[visited][1]) visited++;
    if (visited > 3) visited = 3;
    CHECK(abs(s.abs[AXIS_X] - prev[0]) <= 1 && abs(s.abs[AXIS_Y] - prev[1]) <= 1, "at most 1 step per tick");
    prev[0] = s.abs[AXIS_X]; prev[1] = s.abs[AXIS_Y];
  }
  CHECK(m.mode == MM_IDLE && m.pathLen == 0, "path done, queue empty");
  CHECK(s.abs[AXIS_X] == 2900 && s.abs[AXIS_Y] == 2100, "ends at the last waypoint (%ld,%ld)",
        (long)s.abs[AXIS_X], (long)s.abs[AXIS_Y]);
  CHECK(visited == 3, "passed through every corner");
  CHECK(s.abs[AXIS_Z] == RAIL_MAX[AXIS_Z], "Z untouched by a path");

  // One point, and a point equal to where we are
  const int32_t one[2][2] = { { 0, 4096 }, { 0, 4096 } };
  CHECK(motion_start_path(&m, one, 2) == MOVE_OK, "repeated point");
  run_while(&s, MM_MOVE, 100000);
  CHECK(m.mode == MM_IDLE && s.abs[AXIS_X] == 0 && s.abs[AXIS_Y] == 2100, "repeated point ends there");

  // Bad paths
  CHECK(motion_start_path(&m, one, 0) == MOVE_BAD_PATH, "empty path");
  CHECK(motion_start_path(&m, one, MOTION_PATH_MAX + 1) == MOVE_BAD_PATH, "too many points");
  const int32_t off[1][2] = { { 4097, 0 } }, neg[1][2] = { { 0, -1 } };
  CHECK(motion_start_path(&m, off, 1) == MOVE_BAD_PATH && motion_start_path(&m, neg, 1) == MOVE_BAD_PATH,
        "coordinates outside 0..4096");
  CHECK(m.mode == MM_IDLE, "bad paths change nothing");

  // The longest path: 64 waypoints, a zigzag
  int32_t zig[MOTION_PATH_MAX][2];
  for (int i = 0; i < MOTION_PATH_MAX; i++) { zig[i][0] = (i / 2) * 128; zig[i][1] = ((i + 1) / 2) % 2 ? 512 : 0; }
  CHECK(motion_start_path(&m, (const int32_t (*)[2])zig, MOTION_PATH_MAX) == MOVE_OK, "64 waypoints accepted");
  run_while(&s, MM_MOVE, 400000);
  CHECK(m.mode == MM_IDLE && s.abs[AXIS_X] == rail_coord_to_steps(zig[63][0], m.ax[AXIS_X].span) &&
        s.abs[AXIS_Y] == rail_coord_to_steps(zig[63][1], m.ax[AXIS_Y].span),
        "64-waypoint path ends at its last point (%ld,%ld)", (long)s.abs[AXIS_X], (long)s.abs[AXIS_Y]);

  // Stop mid-path drops the rest
  CHECK(motion_start_path(&m, L, 4) == MOVE_OK, "path again");
  for (int i = 0; i < 1500; i++) sim_tick(&s);
  motion_stop(&m);
  int32_t at[2] = { s.abs[AXIS_X], s.abs[AXIS_Y] };
  for (int i = 0; i < 3000; i++) sim_tick(&s);
  CHECK(m.mode == MM_IDLE && m.pathLen == 0 && s.abs[AXIS_X] == at[0] && s.abs[AXIS_Y] == at[1], "x stops the path");

  // A fault mid-path drops the rest; reset doesn't resume it
  CHECK(motion_start_path(&m, L, 4) == MOVE_OK, "path again");
  for (int i = 0; i < 1500; i++) sim_tick(&s);
  s.estop = true; sim_tick(&s); s.estop = false;
  CHECK(m.mode == MM_FAULT && m.pathLen == 0, "E-stop drops the path");
  CHECK(motion_reset(&m, true) == KEEP_OK, "keep homing");
  at[0] = s.abs[AXIS_X]; at[1] = s.abs[AXIS_Y];
  for (int i = 0; i < 3000; i++) sim_tick(&s);
  CHECK(m.mode == MM_IDLE && s.abs[AXIS_X] == at[0] && s.abs[AXIS_Y] == at[1], "nothing resumes after reset");

  // Not homed
  motion_reset(&m, false);
  CHECK(motion_start_path(&m, L, 1) == MOVE_NOT_HOMED, "path needs X and Y homed");
}

static void test_homing_errors(void) {
  motion_t m; sim_t s;
  // X MAX switch only 30 steps past MIN -> RAIL SHORT on X
  const int32_t start[3] = { 500, 700, 400 };
  const int32_t minAt[3] = { 490, 0, 0 }, maxShort[3] = { 520, 2100, 1500 };
  sim_init(&s, &m, start, minAt, maxShort);
  int x = AXIS_X;
  motion_start_homing(&m, &x, 1);
  run_while(&s, MM_HOMING, 100000);
  CHECK(m.fault == MF_RAIL_SHORT && m.faultAxis == AXIS_X, "RAIL SHORT X (fault %d axis %d)", m.fault, m.faultAxis);

  // Z MIN never reached -> HOME TIMEOUT on Z, first in the queue
  const int32_t farMin[3] = { 0, 0, -99999 };
  sim_init(&s, &m, start, farMin, RAIL_MAX);
  motion_start_homing(&m, ALL, 3);
  run_while(&s, MM_HOMING, 200000);
  CHECK(m.fault == MF_HOME_TIMEOUT && m.faultAxis == AXIS_Z, "HOME TIMEOUT Z (fault %d axis %d)", m.fault, m.faultAxis);
  CHECK(s.abs[AXIS_X] == 500 && s.abs[AXIS_Y] == 700, "X, Y never started");

  // Start on a MIN switch: found at once
  const int32_t onMin[3] = { 0, 0, 0 };
  sim_init(&s, &m, onMin, RAIL_MIN, RAIL_MAX);
  home_all(&s, &m);
  CHECK(m.mode == MM_IDLE && m.ax[AXIS_Y].span == RAIL_MAX[AXIS_Y], "homing from the MIN switches");
}

int main(void) {
  test_profile();
  test_names();
  test_homing_all();
  test_home_one_axis();
  test_moves();
  test_limit_and_skip();
  test_estop_keep();
  test_homing_errors();
  test_xy_only();
  test_paths();
  printf("%s (%d failures)\n", failures ? "FAILED" : "ALL TESTS PASSED", failures);
  return failures ? 1 : 0;
}
