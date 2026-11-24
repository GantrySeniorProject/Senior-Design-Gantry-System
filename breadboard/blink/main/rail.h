/*
  rail.h - rail coordinates and two-ended homing. Hardware-independent, tested
  on the PC by test/test_rail.c.

  Homing finds both physical ends of the rail:
    1. SEEK_MIN: turn toward MIN until the MIN limit is pressed -> that spot is step 0.
    2. SEEK_MAX: turn toward MAX until the MAX limit is pressed -> the step count
       there is the rail span.
  Coordinates 0..RAIL_COORD_MAX are then spread evenly over 0..span steps, so
  coordinate 0 is always the MIN end and RAIL_COORD_MAX the MAX end, whatever
  the measured length.
*/
#pragma once
#include <stdbool.h>
#include <stdint.h>

#define RAIL_COORD_MAX  4096   // coordinate of the MAX end
#define RAIL_MIN_SPAN   64     // MAX pressed closer than this to MIN = bad homing

static inline int32_t rail_clamp(int32_t v, int32_t lo, int32_t hi) { return v < lo ? lo : (v > hi ? hi : v); }

// Coordinate (0..RAIL_COORD_MAX) -> motor step (0..span), rounded to nearest.
static inline int32_t rail_coord_to_steps(int32_t coord, int32_t span) {
  if (span <= 0) return 0;
  coord = rail_clamp(coord, 0, RAIL_COORD_MAX);
  return (int32_t)(((int64_t)coord * span + RAIL_COORD_MAX / 2) / RAIL_COORD_MAX);
}

// Motor step -> coordinate, rounded to nearest. Steps outside the rail clamp to the ends.
static inline int32_t rail_steps_to_coord(int32_t steps, int32_t span) {
  if (span <= 0) return 0;
  steps = rail_clamp(steps, 0, span);
  return (int32_t)(((int64_t)steps * RAIL_COORD_MAX + span / 2) / span);
}

// "Reset and keep home" after a fault. Only an E-stop qualifies: the controller
// stopped stepping on purpose, so it still knows where the carriage is. Every
// other fault (SKIP, LIMIT, homing faults) means the position can't be trusted.
// The encoder must still read what it read when the fault hit and agree with the
// commanded position. On the breadboard the encoder is simulated and can't see a
// hand-turned shaft, so the user confirms nothing moved; on the gantry, the real
// encoders make this check meaningful.
typedef enum { KEEP_OK, KEEP_NOT_ESTOP, KEEP_NOT_HOMED, KEEP_MOVED } keep_home_t;

static inline keep_home_t keep_home_check(bool faultIsEstop, bool homedAtFault,
                                          int32_t encAtFault, int32_t encNow,
                                          int32_t posCmd, int32_t tolerance) {
  if (!faultIsEstop) return KEEP_NOT_ESTOP;
  if (!homedAtFault) return KEEP_NOT_HOMED;
  int32_t drift = encNow - encAtFault, err = posCmd - encNow;
  if (drift > tolerance || -drift > tolerance || err > tolerance || -err > tolerance) return KEEP_MOVED;
  return KEEP_OK;
}

typedef enum { HOME_SEEK_MIN, HOME_SEEK_MAX } home_phase_t;
typedef enum { HOME_CONTINUE, HOME_FOUND_MIN, HOME_FOUND_MAX,
               HOME_ERR_SHORT, HOME_ERR_TIMEOUT } home_event_t;

// One homing decision per control tick. pos = current step (0 at MIN once found),
// phaseSteps = steps taken in this phase. The limit for the other end is ignored:
// right after MIN is found its switch is still pressed while the carriage leaves it.
// maxSteps <= 0 means no limit: homing keeps going until the switch is pressed.
static inline home_event_t home_check(home_phase_t phase, bool limMin, bool limMax,
                                      int32_t pos, int32_t phaseSteps, int32_t maxSteps) {
  if (phase == HOME_SEEK_MIN) {
    if (limMin) return HOME_FOUND_MIN;
  } else if (limMax) {
    return pos >= RAIL_MIN_SPAN ? HOME_FOUND_MAX : HOME_ERR_SHORT;
  }
  if (maxSteps > 0 && phaseSteps > maxSteps) return HOME_ERR_TIMEOUT;
  return HOME_CONTINUE;
}
