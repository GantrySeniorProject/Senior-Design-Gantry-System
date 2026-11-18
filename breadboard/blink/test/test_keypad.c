/*
  test_keypad.c - PC test for main/keypad.h (no ESP-IDF needed).
  Build and run from breadboard/blink:
    gcc -Wall -Wextra -Werror -std=c11 -I main test/test_keypad.c -o test_keypad && ./test_keypad

  Models the keypad as a switch network: each held key joins its row and column.
  R4 is tied to GND, R1 is GND only while the firmware drives it low, C1..C4 and
  R1 have pull-ups, R2/R3 are unconnected. keypad_read() runs against this model
  for every one of the 2^16 key combinations, twice: with C3/C4 wired (3 axes)
  and with them not wired (pinC3 = pinC4 = -1, only E-stop and X).
*/
#include <stdio.h>
#include <string.h>
#include "keypad.h"

enum { PIN_C1 = 4, PIN_C2 = 32, PIN_C3 = 34, PIN_C4 = 35, PIN_R1 = 33 };
static const char LAYOUT[4][5] = {"123A", "456B", "789C", "*0#D"};

static uint32_t pressed;          // bit r*4+c = key at row r, column c held
static int r1Level = 1;           // 1 = released (hi-Z), 0 = driven low

// Sequence log, to check the order of operations
static char seq[64];
static int  seqLen;
static uint32_t lastDelay;
static int  failures;

#define CHECK(cond, ...) do { if (!(cond)) { failures++; if (failures <= 20) { \
  printf("FAIL line %d: ", __LINE__); printf(__VA_ARGS__); printf("\n"); } } } while (0)

// Nets 0..3 = R1..R4, 4..7 = C1..C4
static int parent[8];
static int find(int x) { while (parent[x] != x) x = parent[x]; return x; }

static int net_is_low(int net) {
  for (int i = 0; i < 8; i++) parent[i] = i;
  for (int r = 0; r < 4; r++)
    for (int c = 0; c < 4; c++)
      if (pressed >> (r * 4 + c) & 1) parent[find(r)] = find(4 + c);
  int n = find(net);
  return n == find(3) || (r1Level == 0 && n == find(0));
}

static int fake_get(int pin) {
  seq[seqLen++] = 'g';
  if (pin == PIN_C1) return !net_is_low(4);
  if (pin == PIN_C2) { CHECK(r1Level == 1, "C2 read while R1 is driven low"); return !net_is_low(5); }
  if (pin == PIN_C3) return !net_is_low(6);
  if (pin == PIN_C4) return !net_is_low(7);
  CHECK(0, "read of unexpected pin %d", pin);
  return 1;
}

static void fake_set(int pin, int level) {
  CHECK(pin == PIN_R1, "set on pin %d (only R1 may be driven)", pin);
  CHECK(level == 0 || level == 1, "bad level %d", level);
  seq[seqLen++] = level ? 'H' : 'L';
  r1Level = level;
}

static void fake_delay(uint32_t us) { seq[seqLen++] = 'd'; lastDelay = us; }

static const keypad_io_t ioFull = {
  .get = fake_get, .set = fake_set, .delay_us = fake_delay,
  .pinC1 = PIN_C1, .pinC2 = PIN_C2, .pinC3 = PIN_C3, .pinC4 = PIN_C4, .pinR1 = PIN_R1,
};
static const keypad_io_t ioXOnly = {
  .get = fake_get, .set = fake_set, .delay_us = fake_delay,
  .pinC1 = PIN_C1, .pinC2 = PIN_C2, .pinC3 = -1, .pinC4 = -1, .pinR1 = PIN_R1,
};

static int key_bit(char k) {
  for (int r = 0; r < 4; r++)
    for (int c = 0; c < 4; c++)
      if (LAYOUT[r][c] == k) return r * 4 + c;
  return -1;
}
static uint32_t keys(const char *s) { uint32_t m = 0; for (; *s; s++) m |= 1u << key_bit(*s); return m; }

static int popcount16(uint32_t v) { int n = 0; while (v) { n += v & 1; v >>= 1; } return n; }

static keypad_state_t read_with(const keypad_io_t *io, uint32_t m) {
  pressed = m; r1Level = 1; seqLen = 0;
  keypad_state_t s = keypad_read(io);
  seq[seqLen] = 0;
  return s;
}

static bool any_pressed(keypad_state_t s) {
  bool r = s.estop;
  for (int a = 0; a < KEYPAD_AXES; a++) r = r || s.limMin[a] || s.limMax[a];
  return r;
}

int main(void) {
  // Key for each reading: MIN / MAX per axis
  const char MINKEY[KEYPAD_AXES] = { '0', '#', 'D' }, MAXKEY[KEYPAD_AXES] = { '1', '3', 'A' };
  const uint32_t STAR = keys("*");
  int minGhostKeys = 99, combos = 0;

  // 1) Every key combination, C3 / C4 wired
  for (uint32_t m = 0; m < (1u << 16); m++) {
    keypad_state_t s = read_with(&ioFull, m);
    combos++;
    bool tEstop = m & STAR, ghost = s.estop && !tEstop;
    CHECK(!tEstop || s.estop, "mask %04x: * held but E-stop not read", m);
    for (int a = 0; a < KEYPAD_AXES; a++) {
      bool tMin = m & keys((char[]){MINKEY[a], 0}), tMax = m & keys((char[]){MAXKEY[a], 0});
      // MAX may only be masked by its own column's static key (* for X, MIN for Y / Z)
      bool masked = a == 0 ? s.estop : s.limMin[a];
      CHECK(!tMin || s.limMin[a], "mask %04x: axis %d MIN key held but not read", m, a);
      CHECK(!tMax || s.limMax[a] || masked, "mask %04x: axis %d MAX key held but not read", m, a);
      CHECK(!(s.limMax[a] && masked), "mask %04x: axis %d MAX reported while masked", m, a);
      ghost = ghost || (s.limMin[a] && !tMin) || (s.limMax[a] && !tMax);
    }
    if (ghost && popcount16(m) < minGhostKeys) minGhostKeys = popcount16(m);

    // Sequence: read C1..C4, R1 low, settle, read C1 C3 C4, release, settle
    CHECK(strcmp(seq, "ggggLdgggHd") == 0, "mask %04x: sequence %s", m, seq);
    CHECK(r1Level == 1, "mask %04x: R1 left driven low", m);
    CHECK(lastDelay == KEYPAD_RELEASE_SETTLE_US, "mask %04x: no settle after release", m);
  }
  CHECK(minGhostKeys >= 3, "a ghost read needs only %d keys", minGhostKeys);

  // 2) Every key combination, C3 / C4 not wired: X behaves as before, Y / Z never pressed
  int minGhostX = 99;
  for (uint32_t m = 0; m < (1u << 16); m++) {
    keypad_state_t s = read_with(&ioXOnly, m);
    bool tEstop = m & STAR, tMin = m & keys("0"), tMax = m & keys("1");
    CHECK(!tEstop || s.estop, "x-only %04x: E-stop missed", m);
    CHECK(!tMin || s.limMin[0], "x-only %04x: X MIN missed", m);
    CHECK(!tMax || s.limMax[0] || s.estop, "x-only %04x: X MAX missed", m);
    CHECK(!s.limMin[1] && !s.limMax[1] && !s.limMin[2] && !s.limMax[2], "x-only %04x: Y/Z read pressed", m);
    if ((s.estop && !tEstop) || (s.limMin[0] && !tMin) || (s.limMax[0] && !tMax))
      if (popcount16(m) < minGhostX) minGhostX = popcount16(m);
    CHECK(strcmp(seq, "ggLdgHd") == 0, "x-only %04x: sequence %s", m, seq);
  }
  CHECK(minGhostX >= 3, "x-only: a ghost read needs only %d keys", minGhostX);

  // 3) Single keys: exact mapping, the other keys do nothing
  for (int b = 0; b < 16; b++) {
    keypad_state_t s = read_with(&ioFull, 1u << b);
    char k = LAYOUT[b / 4][b % 4];
    bool ok = s.estop == (k == '*');
    for (int a = 0; a < KEYPAD_AXES; a++)
      ok = ok && s.limMin[a] == (k == MINKEY[a]) && s.limMax[a] == (k == MAXKEY[a]);
    CHECK(ok, "key %c alone gives the wrong reading", k);
    if (strchr("2456789BC", k)) CHECK(!any_pressed(s), "key %c should do nothing", k);
  }

  // 4) Named cases: keys -> estop, X min/max, Y min/max, Z min/max
  struct { const char *keys; bool e, xn, xx, yn, yx, zn, zx; } cases[] = {
    {"",     0, 0,0, 0,0, 0,0},
    {"*",    1, 0,0, 0,0, 0,0}, {"0", 0, 1,0, 0,0, 0,0}, {"1", 0, 0,1, 0,0, 0,0},
    {"#",    0, 0,0, 1,0, 0,0}, {"3", 0, 0,0, 0,1, 0,0},
    {"D",    0, 0,0, 0,0, 1,0}, {"A", 0, 0,0, 0,0, 0,1},
    {"01",   0, 1,1, 0,0, 0,0},             // both X limits held
    {"*1",   1, 0,0, 0,0, 0,0},             // X MAX masked by the E-stop
    {"#3",   0, 0,0, 1,0, 0,0},             // Y MAX masked by Y MIN (same column)
    {"DA",   0, 0,0, 0,0, 1,0},             // Z MAX masked by Z MIN
    {"13A",  0, 0,1, 0,1, 0,1},             // all three MAX at once
    {"0#D",  0, 1,0, 1,0, 1,0},             // all three MIN at once
    {"3D",   0, 0,0, 0,1, 1,0}, {"#A", 0, 0,0, 1,0, 0,1},   // different axes don't mask
    {"*3A",  1, 0,0, 0,1, 0,1},             // E-stop doesn't hide Y / Z MAX
    {"2",    0, 0,0, 0,0, 0,0}, {"5BC", 0, 0,0, 0,0, 0,0},
  };
  for (size_t i = 0; i < sizeof cases / sizeof cases[0]; i++) {
    keypad_state_t s = read_with(&ioFull, keys(cases[i].keys));
    CHECK(s.estop == cases[i].e &&
          s.limMin[0] == cases[i].xn && s.limMax[0] == cases[i].xx &&
          s.limMin[1] == cases[i].yn && s.limMax[1] == cases[i].yx &&
          s.limMin[2] == cases[i].zn && s.limMax[2] == cases[i].zx,
          "keys \"%s\" -> e=%d X=%d%d Y=%d%d Z=%d%d", cases[i].keys, s.estop,
          s.limMin[0], s.limMax[0], s.limMin[1], s.limMax[1], s.limMin[2], s.limMax[2]);
  }

  printf("combinations checked: %d (x2 wirings)\n", combos);
  printf("fewest keys for a ghost read: %d with C3/C4, %d without\n",
         minGhostKeys, minGhostX);
  printf("%s (%d failures)\n", failures ? "FAILED" : "ALL TESTS PASSED", failures);
  return failures ? 1 : 0;
}
