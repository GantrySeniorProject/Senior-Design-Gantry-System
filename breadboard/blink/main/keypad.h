/*
  keypad.h - E-stop and the MIN / MAX limits of three axes from the kit's 4x4
  membrane keypad. Hardware-independent: the GPIO calls are passed in, so the
  same code runs on the ESP32 and in the PC test (test/test_keypad.c).

  Wiring (see breadboard/pinplan.txt):
            C1   C2   C3   C4
      R1     1    2    3    A      R1 -> pinR1 (open-drain, pull-up)
      R2     4    5    6    B      R2, R3 not connected
      R3     7    8    9    C
      R4     *    0    #    D      R4 -> GND
    C1..C4 -> pinC1..pinC4 (inputs with pull-ups). pinC3 / pinC4 may be -1
    (not wired): then the Y / Z limits always read released.

    *  = E-stop
    0  = X MIN     1 = X MAX
    #  = Y MIN     3 = Y MAX      (column C3)
    D  = Z MIN     A = Z MAX      (column C4; Z MIN = attachment down, MAX = up)
    2 and the keys on R2 / R3 do nothing.

  Read sequence:
    1. R1 released: C1 low = E-stop, C2 low = X MIN, C3 low = Y MIN, C4 low = Z MIN
    2. R1 driven low: C1 low = X MAX, C3 low = Y MAX, C4 low = Z MAX.
       A column that was already low in step 1 can't tell its R1 key apart, so
       that MAX reads released: * masks X MAX (everything is stopped anyway),
       # masks Y MAX and D masks Z MAX (a real axis can't be at both ends).
    3. R1 released, wait for the columns to pull back up before the next read.
  The caller must keep two reads from overlapping (they share R1).
*/
#pragma once
#include <stdbool.h>
#include <stdint.h>

#define KEYPAD_SCAN_SETTLE_US     10   // after driving R1 low
#define KEYPAD_RELEASE_SETTLE_US  20   // after releasing R1 (pull-up recovery is slower)
#define KEYPAD_AXES               3    // X, Y, Z

typedef struct { bool estop; bool limMin[KEYPAD_AXES], limMax[KEYPAD_AXES]; } keypad_state_t;

typedef struct {
  int  (*get)(int pin);                // 0 = low
  void (*set)(int pin, int level);     // R1 only: 1 = released (hi-Z), 0 = low
  void (*delay_us)(uint32_t us);
  int  pinC1, pinC2, pinC3, pinC4, pinR1;   // pinC3 / pinC4 = -1: not wired
} keypad_io_t;

static inline bool keypad_low(const keypad_io_t *io, int pin) {
  return pin >= 0 && io->get(pin) == 0;
}

static inline keypad_state_t keypad_read(const keypad_io_t *io) {
  keypad_state_t s;
  s.estop     = keypad_low(io, io->pinC1);
  s.limMin[0] = keypad_low(io, io->pinC2);
  s.limMin[1] = keypad_low(io, io->pinC3);
  s.limMin[2] = keypad_low(io, io->pinC4);
  io->set(io->pinR1, 0);
  io->delay_us(KEYPAD_SCAN_SETTLE_US);
  bool c1 = keypad_low(io, io->pinC1);
  bool c3 = keypad_low(io, io->pinC3);
  bool c4 = keypad_low(io, io->pinC4);
  io->set(io->pinR1, 1);
  io->delay_us(KEYPAD_RELEASE_SETTLE_US);
  s.limMax[0] = c1 && !s.estop;        // column already low: its R1 key is unknown
  s.limMax[1] = c3 && !s.limMin[1];
  s.limMax[2] = c4 && !s.limMin[2];
  return s;
}
