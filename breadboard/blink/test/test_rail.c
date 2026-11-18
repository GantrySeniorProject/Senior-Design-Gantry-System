/*
  test_rail.c - PC test for main/rail.h (coordinates and two-ended homing).
  Build and run from breadboard/blink:
    gcc -Wall -Wextra -Werror -std=c11 -I main test/test_rail.c -o test_rail && ./test_rail

  Part 3 replays the motion_tick homing loop (same ramp, same events) against
  a simulated rail whose MIN / MAX switches sit at chosen step positions.
*/
#include <stdio.h>
#include <math.h>
#include "rail.h"

static int failures;
#define CHECK(cond, ...) do { if (!(cond)) { failures++; if (failures <= 20) { \
  printf("FAIL line %d: ", __LINE__); printf(__VA_ARGS__); printf("\n"); } } } while (0)

// Mirrors the M_HOMING case of motion_tick. The carriage starts at absolute
// step `start`; switches are pressed while the carriage is at or past them.
typedef struct { home_event_t result; int32_t span, pos, ticks; } home_run_t;

static home_run_t simulate_homing(int32_t start, int32_t minAt, int32_t maxAt, int32_t maxSteps) {
  const float dt = 0.001f, AMAX = 1200.0f, HOME_SPEED = 300.0f;
  home_phase_t phase = HOME_SEEK_MIN;
  int32_t abs = start, pos = start, steps = 0;
  float vel = 0, acc = 0;
  home_run_t r = { HOME_CONTINUE, 0, 0, 0 };
  for (r.ticks = 0; r.ticks < 200000; r.ticks++) {
    bool limMin = abs <= minAt, limMax = abs >= maxAt;
    home_event_t ev = home_check(phase, limMin, limMax, pos, steps, maxSteps);
    if (ev == HOME_FOUND_MIN) { pos = 0; vel = acc = 0; steps = 0; phase = HOME_SEEK_MAX; continue; }
    if (ev != HOME_CONTINUE) { r.result = ev; r.span = pos; r.pos = abs; return r; }
    int dir = (phase == HOME_SEEK_MIN) ? -1 : 1;
    vel = fminf(vel + AMAX * dt, HOME_SPEED);
    acc += vel * dt;
    while (acc >= 1.0f) { abs += dir; pos += dir; acc -= 1.0f; steps++; }
  }
  return r;
}

int main(void) {
  // 1) Coordinate <-> step mapping
  const int32_t spans[] = { 64, 100, 777, 2900, 4096, 5000, 12288 };
  for (size_t i = 0; i < sizeof spans / sizeof spans[0]; i++) {
    int32_t span = spans[i], prev = -1;
    CHECK(rail_coord_to_steps(0, span) == 0, "span %ld: coord 0 -> %ld", (long)span, (long)rail_coord_to_steps(0, span));
    CHECK(rail_coord_to_steps(RAIL_COORD_MAX, span) == span, "span %ld: coord max -> %ld", (long)span,
          (long)rail_coord_to_steps(RAIL_COORD_MAX, span));
    CHECK(rail_steps_to_coord(0, span) == 0, "span %ld: step 0 -> coord %ld", (long)span, (long)rail_steps_to_coord(0, span));
    CHECK(rail_steps_to_coord(span, span) == RAIL_COORD_MAX, "span %ld: last step -> coord %ld", (long)span,
          (long)rail_steps_to_coord(span, span));
    for (int32_t c = 0; c <= RAIL_COORD_MAX; c++) {
      int32_t s = rail_coord_to_steps(c, span);
      CHECK(s >= prev, "span %ld: not monotonic at coord %ld", (long)span, (long)c);
      CHECK(s >= 0 && s <= span, "span %ld: coord %ld -> step %ld outside rail", (long)span, (long)c, (long)s);
      // Converting back lands within half a step's worth of coordinate
      int32_t back = rail_steps_to_coord(s, span);
      int32_t tol = RAIL_COORD_MAX / span / 2 + 1;
      CHECK(back - c <= tol && c - back <= tol, "span %ld: coord %ld -> %ld -> %ld", (long)span, (long)c, (long)s, (long)back);
      prev = s;
    }
    // Step -> coord -> step: exact while steps <= coordinates; on longer rails one
    // coordinate covers several steps, so it lands within half a coordinate.
    int32_t stepTol = span <= RAIL_COORD_MAX ? 0 : span / RAIL_COORD_MAX / 2 + 1;
    for (int32_t s = 0; s <= span; s++) {
      int32_t d = rail_coord_to_steps(rail_steps_to_coord(s, span), span) - s;
      CHECK(d <= stepTol && -d <= stepTol, "span %ld: step %ld round trip off by %ld", (long)span, (long)s, (long)d);
    }
  }
  // Clamping and unhomed span
  CHECK(rail_coord_to_steps(-50, 2900) == 0 && rail_coord_to_steps(99999, 2900) == 2900, "coord clamp");
  CHECK(rail_steps_to_coord(-5, 2900) == 0 && rail_steps_to_coord(3500, 2900) == RAIL_COORD_MAX, "step clamp");
  CHECK(rail_coord_to_steps(2048, 0) == 0 && rail_steps_to_coord(10, 0) == 0, "span 0 must not divide by zero");
  CHECK(rail_coord_to_steps(2048, 2900) == 1450, "midpoint of 2900 -> %ld", (long)rail_coord_to_steps(2048, 2900));

  // 2) home_check decisions
  CHECK(home_check(HOME_SEEK_MIN, false, false, 500, 10, 100) == HOME_CONTINUE, "seek min, nothing pressed");
  CHECK(home_check(HOME_SEEK_MIN, true,  false, 500, 10, 100) == HOME_FOUND_MIN, "seek min, MIN pressed");
  CHECK(home_check(HOME_SEEK_MIN, false, true,  500, 10, 100) == HOME_CONTINUE, "seek min ignores MAX");
  CHECK(home_check(HOME_SEEK_MAX, true,  false, 5,   5,  100) == HOME_CONTINUE, "seek max ignores MIN still held");
  CHECK(home_check(HOME_SEEK_MAX, false, true,  RAIL_MIN_SPAN, 64, 9999) == HOME_FOUND_MAX, "MAX at minimum span");
  CHECK(home_check(HOME_SEEK_MAX, false, true,  RAIL_MIN_SPAN - 1, 63, 9999) == HOME_ERR_SHORT, "MAX too close");
  CHECK(home_check(HOME_SEEK_MIN, false, false, 0, 101, 100) == HOME_ERR_TIMEOUT, "timeout seeking MIN");
  CHECK(home_check(HOME_SEEK_MAX, false, false, 101, 101, 100) == HOME_ERR_TIMEOUT, "timeout seeking MAX");
  CHECK(home_check(HOME_SEEK_MIN, true,  false, 0, 999, 100) == HOME_FOUND_MIN, "switch wins over timeout");

  // 2b) keep_home_check: only an E-stop on a homed axis whose encoder hasn't moved
  CHECK(keep_home_check(true,  true,  500, 500, 500, 4) == KEEP_OK,        "E-stop, homed, still -> keep");
  CHECK(keep_home_check(true,  true,  500, 504, 504, 4) == KEEP_OK,        "drift within tolerance");
  CHECK(keep_home_check(false, true,  500, 500, 500, 4) == KEEP_NOT_ESTOP, "other fault -> no keep");
  CHECK(keep_home_check(true,  false, 500, 500, 500, 4) == KEEP_NOT_HOMED, "not homed (e.g. E-stop while homing)");
  CHECK(keep_home_check(true,  true,  500, 505, 505, 4) == KEEP_MOVED,     "encoder moved +5 since fault");
  CHECK(keep_home_check(true,  true,  500, 495, 495, 4) == KEEP_MOVED,     "encoder moved -5 since fault");
  CHECK(keep_home_check(true,  true,  500, 500, 505, 4) == KEEP_MOVED,     "commanded vs encoder off by 5");
  CHECK(keep_home_check(true,  true,  500, 500, 495, 4) == KEEP_MOVED,     "commanded vs encoder off by -5");
  CHECK(keep_home_check(false, false, 0, 99, 0, 4) == KEEP_NOT_ESTOP,      "not-E-stop reported first");

  // 3) Full homing runs through the motion_tick loop
  struct { int32_t start, minAt, maxAt; home_event_t want; int32_t span; } runs[] = {
    {  1000,     0,  2900, HOME_FOUND_MAX, 2900 },   // typical: start mid-rail
    {     0,     0,  4096, HOME_FOUND_MAX, 4096 },   // start on the MIN switch
    {  4096,     0,  4096, HOME_FOUND_MAX, 4096 },   // start on the MAX switch
    {   300,  -200,  1800, HOME_FOUND_MAX, 2000 },   // switches anywhere: span = distance
    {   500,   490,   520, HOME_ERR_SHORT,    0 },   // MAX only 30 steps past MIN
    { 30000,     0,  2900, HOME_ERR_TIMEOUT,  0 },   // MIN never reached
    {  1000,     0, 99999, HOME_ERR_TIMEOUT,  0 },   // MAX never reached
  };
  for (size_t i = 0; i < sizeof runs / sizeof runs[0]; i++) {
    home_run_t r = simulate_homing(runs[i].start, runs[i].minAt, runs[i].maxAt, 3 * 4096);
    CHECK(r.result == runs[i].want, "run %zu: result %d, want %d", i, r.result, runs[i].want);
    if (runs[i].want == HOME_FOUND_MAX) {
      CHECK(r.span == runs[i].span, "run %zu: span %ld, want %ld", i, (long)r.span, (long)runs[i].span);
      CHECK(r.pos == runs[i].maxAt, "run %zu: stopped at %ld, MAX switch at %ld", i, (long)r.pos, (long)runs[i].maxAt);
    }
  }

  printf("%s (%d failures)\n", failures ? "FAILED" : "ALL TESTS PASSED", failures);
  return failures ? 1 : 0;
}
