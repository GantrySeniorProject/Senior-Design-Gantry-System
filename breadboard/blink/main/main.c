/*
  main.c - 3-axis gantry test bench (X, Y carriage + Z attachment), native ESP-IDF v6.0 (C)
  Board : Freenove ESP32-WROVER (target esp32). Leave the camera UNPLUGGED.
  Console: ESP-IDF monitor (115200). Type a command and press Enter.

  Only the steppers and the keypad are used. Three 28BYJ-48 + ULN2003 boards
  stand in for the X, Y and Z axes. The controller itself is in motion.h
  (tested on the PC); this file drives the coils, reads the keypad and runs
  the console.

    - motion_tick  (esp_timer, 1 kHz): keypad + simulated presses -> motion.h
    - report_task  (core 1): fault messages, backup E-stop check, telemetry, @S status
    - console_task (core 1): serial commands

  Commands:
    h            home all axes: Z first (ends raised; only with USE_Z_AXIS), then X, then Y.
                 Each axis: toward MIN until MIN is pressed, then toward MAX until MAX.
                 Then X and Y drive back to 0,0, the starting position.
    h x|y|z      home one axis
    g <x> <y>    straight-line S-curve move to X/Y coordinates 0..4096
    w x1 y1 x2 y2 ...  drive a path: one straight move per waypoint, in order (up to
                 64 waypoints; the web page sends these from its A* planner)
    z <c>        move the attachment: 0 = down (Z MIN) .. 4096 = up (Z MAX)
    k [x|y|z] <n> simulate n skipped steps on an axis (default X)
    x            stop now
    e            software E-stop (latches ESTOP, same as keypad *)
    p<axis><0|1> simulated MIN (0) / MAX (1) press, e.g. px0, py1, pz0 (p0 / p1 = X)
    r            reset a fault (then home again)
    r k          after an E-stop only: reset and KEEP homing (only if nothing moved)
    t            toggle 1 s telemetry printout
    q1 / q0      start / stop the 20 Hz "@S" status line for the web page
    ?            help

  PIN MAP (full plan in breadboard/pinplan.txt)
    X stepper IN1..IN4 : GPIO 13, 14, 27, 26
    Y stepper IN1..IN4 : GPIO 23, 22, 21, 19
    Z stepper IN1..IN4 : GPIO 18, 25, 15, 2   (GPIO 2 = on-board LED, it flickers with Z)
    Keypad C1 / C2 / R1 : GPIO 4 / 32 / 33     Keypad R4 : GND
    Keypad C3 / C4      : GPIO 34 / 35, each with a 10k pull-up to 3.3V (USE_YZ_KEYS)
      * = E-stop, 0/1 = X MIN/MAX, #/3 = Y MIN/MAX, D/A = Z MIN/MAX (see keypad.h)
*/

#include <stdio.h>
#include <stdint.h>
#include <stdbool.h>
#include <stdlib.h>
#include <string.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_timer.h"
#include "esp_rom_sys.h"
#include "driver/gpio.h"
#include "driver/uart.h"
#include "keypad.h"
#include "motion.h"

// ----------------------------- Pins ---------------------------------------
static const gpio_num_t COIL_PINS[MOTION_AXES][4] = {
  { GPIO_NUM_13, GPIO_NUM_14, GPIO_NUM_27, GPIO_NUM_26 },   // X
  { GPIO_NUM_23, GPIO_NUM_22, GPIO_NUM_21, GPIO_NUM_19 },   // Y
  { GPIO_NUM_18, GPIO_NUM_25, GPIO_NUM_15, GPIO_NUM_2  },   // Z
};
#define PIN_KP_C1      GPIO_NUM_4    // keypad column 1: * (E-stop) and 1 (X MAX)
#define PIN_KP_C2      GPIO_NUM_32   // keypad column 2: 0 (X MIN)
#define PIN_KP_C3      GPIO_NUM_34   // keypad column 3: # (Y MIN) and 3 (Y MAX), external pull-up
#define PIN_KP_C4      GPIO_NUM_35   // keypad column 4: D (Z MIN) and A (Z MAX), external pull-up
#define PIN_KP_R1      GPIO_NUM_33   // keypad row 1, open-drain scan line for 1 / 3 / A

// GPIO 34 / 35 have no internal pull-ups. Keep 0 until the two 10k pull-ups and
// the C3 / C4 wires are in, or the floating pins read as random limit presses.
// With 0, the Y / Z limits come only from px/py/pz (web page MIN / MAX buttons).
#define USE_YZ_KEYS       0

// Z axis (attachment). 0 = only X and Y are tested: "h" homes X then Y, Z can't be
// homed or moved, its coils stay off and keypad C4 (GPIO 35) isn't read.
// Set to 1 once the Z motor is wired: "h" then homes Z first (it ends raised).
#define USE_Z_AXIS        0

// ----------------------------- Tuning -------------------------------------
// Kit stepper (28BYJ-48), half-step mode: about 4096 half-steps per output turn.
#define VMAX              600.0f   // half-steps per second (per motor)
#define AMAX              1200.0f  // peak acceleration, half-steps/s^2
#define HOME_SPEED        300.0f
#define HOME_MAX_STEPS    (3 * 4096) // homing gives up after this many steps per end
#define SKIP_THRESHOLD    4        // allowed commanded-vs-encoder error
#define VIRT_PRESS_MS     150      // how long a p.. simulated limit press lasts
#define STATUS_PERIOD_MS  50       // "@S" status line rate for the web page (20 Hz)
#define COIL_OFF_DELAY_MS 500      // power a motor down after it hasn't stepped this long

// ----------------------------- State --------------------------------------
static portMUX_TYPE motionMux = portMUX_INITIALIZER_UNLOCKED;
static motion_t mc;                                     // guarded by motionMux
static volatile uint32_t virtMinUntil[MOTION_AXES], virtMaxUntil[MOTION_AXES];  // p.. presses (ms)
static volatile motion_inputs_t lastInputs;             // last inputs seen by motion_tick
static volatile uint32_t motionTicks = 0;               // proves the 1 kHz controller is running
static volatile bool telemetry = false, statusStream = false;

static inline uint32_t millis32(void) { return (uint32_t)(esp_timer_get_time() / 1000); }

// ----------------------------- Steppers -----------------------------------
static const uint8_t HALF_STEP[8][4] = {
  {1,0,0,0},{1,1,0,0},{0,1,0,0},{0,1,1,0},{0,0,1,0},{0,0,1,1},{0,0,0,1},{1,0,0,1}};
static uint8_t  phaseIdx[MOTION_AXES];
static bool     coilsOn[MOTION_AXES];
static uint32_t lastStepMs[MOTION_AXES];

static void coils_write(int a, uint8_t i) {
  for (int c = 0; c < 4; c++) gpio_set_level(COIL_PINS[a][c], HALF_STEP[i][c]);
  coilsOn[a] = true;
}

static void coils_off(int a) {
  for (int c = 0; c < 4; c++) gpio_set_level(COIL_PINS[a][c], 0);
  coilsOn[a] = false;
}

static void hw_step(void *ctx, int a, int dir) {         // called by motion.h, under motionMux
  (void)ctx;
  phaseIdx[a] = (uint8_t)((phaseIdx[a] + (dir > 0 ? 1 : 7)) & 7);
  coils_write(a, phaseIdx[a]);
  lastStepMs[a] = millis32();
}

// ----------------------------- Keypad -------------------------------------
// motion_tick and report_task read it from different cores; the lock keeps
// one read's R1 scan from overlapping the other's static reads.
static portMUX_TYPE keypadMux = portMUX_INITIALIZER_UNLOCKED;
static int  kp_get(int pin)            { return gpio_get_level((gpio_num_t)pin); }
static void kp_set(int pin, int level) { gpio_set_level((gpio_num_t)pin, (uint32_t)level); }
static const keypad_io_t kpIo = {
  .get = kp_get, .set = kp_set, .delay_us = esp_rom_delay_us,
  .pinC1 = PIN_KP_C1, .pinC2 = PIN_KP_C2, .pinR1 = PIN_KP_R1,
  .pinC3 = USE_YZ_KEYS ? PIN_KP_C3 : -1,
  .pinC4 = (USE_YZ_KEYS && USE_Z_AXIS) ? PIN_KP_C4 : -1,   // Z keys only with the Z axis on
};

static keypad_state_t kp_read(void) {
  portENTER_CRITICAL(&keypadMux);
  keypad_state_t s = keypad_read(&kpIo);
  portEXIT_CRITICAL(&keypadMux);
  return s;
}

// ----------------------------- 1 kHz motion controller --------------------
static void motion_timer_cb(void *arg) {
  (void)arg;
  keypad_state_t kp = kp_read();
  uint32_t now = millis32();
  motion_inputs_t in = { .estop = kp.estop };
  for (int a = 0; a < MOTION_AXES; a++) {               // keypad or a p.. press
    in.limMin[a] = kp.limMin[a] || (int32_t)(virtMinUntil[a] - now) > 0;
    in.limMax[a] = kp.limMax[a] || (int32_t)(virtMaxUntil[a] - now) > 0;
  }
  lastInputs = in;
  motionTicks++;

  portENTER_CRITICAL(&motionMux);
  motion_tick(&mc, &in, 0.001f);
  for (int a = 0; a < MOTION_AXES; a++)
    if (coilsOn[a] && (mc.mode == MM_FAULT || now - lastStepMs[a] > COIL_OFF_DELAY_MS)) coils_off(a);
  portEXIT_CRITICAL(&motionMux);
}

static void raise_estop(void) {
  portENTER_CRITICAL(&motionMux);
  motion_fault(&mc, MF_ESTOP, -1);
  for (int a = 0; a < MOTION_AXES; a++) if (coilsOn[a]) coils_off(a);
  portEXIT_CRITICAL(&motionMux);
}

// ----------------------------- Commands -----------------------------------
static int axis_from_char(char c) {
  switch (c) { case 'x': case 'X': return AXIS_X; case 'y': case 'Y': return AXIS_Y;
               case 'z': case 'Z': return AXIS_Z; }
  return -1;
}

static char *skip_spaces(char *s) { while (*s == ' ') s++; return s; }

static void start_move(int32_t x, int32_t y, int32_t z) {
  int32_t c[MOTION_AXES] = { x, y, z };
  portENTER_CRITICAL(&motionMux);
  move_result_t r = motion_start_move(&mc, c);
  portEXIT_CRITICAL(&motionMux);
  if (r == MOVE_NOT_HOMED) printf("Home that axis first (h).\n");
  else if (r == MOVE_BUSY) printf("Busy or faulted.\n");
}

static void start_homing(const int *axes, int n) {
  portENTER_CRITICAL(&motionMux);
  bool ok = motion_start_homing(&mc, axes, n);
  portEXIT_CRITICAL(&motionMux);
  if (ok) printf("Homing: for each axis press its MIN key, then its MAX key "
                 "(X: 0 then 1, Y: # then 3%s).\n", USE_Z_AXIS ? ", Z: D then A" : "");
  else printf("Can't home now. Fault? r to reset first; moving? x to stop.\n");
}

static void reset_fault(bool keep) {
  if (kp_read().estop) { printf("Release the e-stop (*) first.\n"); return; }
  portENTER_CRITICAL(&motionMux);
  keep_home_t k = motion_reset(&mc, keep);
  portEXIT_CRITICAL(&motionMux);
  if (!keep)             printf("Fault cleared. Home again (h) before moving.\n");
  else if (k == KEEP_OK) printf("Fault cleared, homing kept.\n");
  else printf("Can't keep homing: %s. Use r and home again.\n",
              k == KEEP_NOT_ESTOP ? "only after an E-stop" :
              k == KEEP_NOT_HOMED ? "no axis was homed" : "an encoder moved");
}

#if USE_Z_AXIS
static const int HOME_ALL[] = { AXIS_Z, AXIS_X, AXIS_Y };   // raise the attachment first
#define HOME_ORDER_TEXT "Z, X, Y"
#else
static const int HOME_ALL[] = { AXIS_X, AXIS_Y };
#define HOME_ORDER_TEXT "X, Y; Z disabled"
#endif
#define Z_DISABLED_MSG "Z axis is disabled (USE_Z_AXIS 0 in main.c).\n"

static void print_help(void) {
  printf("\nCommands: h [x|y|z] | g <x> <y> | w x1 y1 x2 y2.. | z <c> | k [axis] <n> | x | e | p<axis><0|1> | r | r k | t | q1 | q0 | ?\n"
         "  h home (" HOME_ORDER_TEXT ", then back to 0,0), g X/Y move 0..4096, w path,\n   z attachment 0 (down)..4096 (up), k simulate skips,\n"
         "  x stop, e E-stop, px0/px1/py0/py1/pz0/pz1 press MIN/MAX, r reset fault (r k: keep homing),\n"
         "  t telemetry, q1/q0 web status\n");
}

static void run_command(char *buf) {
  char cmd = buf[0];
  char *arg = skip_spaces(buf + 1);
  switch (cmd) {
    case 'h': {
      int a = axis_from_char(arg[0]);
      if (a == AXIS_Z && !USE_Z_AXIS) { printf(Z_DISABLED_MSG); break; }
      if (a >= 0) { start_homing(&a, 1); break; }
      start_homing(HOME_ALL, (int)(sizeof HOME_ALL / sizeof HOME_ALL[0]));
    } break;
    case 'g': {
      char *endX, *endY;
      long x = strtol(arg, &endX, 10), y = strtol(endX, &endY, 10);
      if (endX == arg || endY == endX) { printf("g <x> <y>, each 0..4096\n"); break; }
      start_move((int32_t)rail_clamp((int32_t)x, 0, RAIL_COORD_MAX),
                 (int32_t)rail_clamp((int32_t)y, 0, RAIL_COORD_MAX), -1);
    } break;
    case 'w': {                                // path: x1 y1 x2 y2 ...
      static int32_t pts[MOTION_PATH_MAX][2];
      int n = 0;
      char *p = arg, *end;
      bool bad = false;
      for (;;) {
        long x = strtol(p, &end, 10);
        if (end == p) break;                   // no more numbers
        p = end;
        long y = strtol(p, &end, 10);
        if (end == p || n >= MOTION_PATH_MAX) { bad = true; break; }   // odd count / too many
        p = end;
        pts[n][0] = (int32_t)x; pts[n][1] = (int32_t)y; n++;
      }
      if (*skip_spaces(p)) bad = true;         // trailing junk
      if (bad || n == 0) { printf("w x1 y1 x2 y2 ... (pairs 0..4096, up to %d)\n", MOTION_PATH_MAX); break; }
      portENTER_CRITICAL(&motionMux);
      move_result_t r = motion_start_path(&mc, (const int32_t (*)[2])pts, n);
      portEXIT_CRITICAL(&motionMux);
      if (r == MOVE_OK) printf("Path: %d waypoint(s).\n", n);
      else printf("%s\n", r == MOVE_NOT_HOMED ? "Home X and Y first (h)." :
                          r == MOVE_BUSY ? "Busy or faulted." : "Bad path (coordinates 0..4096).");
    } break;
    case 'z':
      if (!USE_Z_AXIS) { printf(Z_DISABLED_MSG); break; }
      if (!*arg) { printf("z <coord>, 0 = down .. 4096 = up\n"); break; }
      start_move(-1, -1, rail_clamp((int32_t)atol(arg), 0, RAIL_COORD_MAX));
      break;
    case 'k': {
      int a = axis_from_char(arg[0]);
      if (a >= 0) arg = skip_spaces(arg + 1); else a = AXIS_X;
      long n = atol(arg);
      int32_t steps = (int32_t)(n > 0 ? n : 8);
      portENTER_CRITICAL(&motionMux);
      mc.ax[a].skipInject = steps;
      portEXIT_CRITICAL(&motionMux);
      printf("%c axis will skip %ld step(s).\n", motion_axis_name(a), (long)steps);
    } break;
    case 'x':
      portENTER_CRITICAL(&motionMux);
      motion_stop(&mc);
      portEXIT_CRITICAL(&motionMux);
      break;
    case 'e': raise_estop(); break;
    case 'p': {
      int a = axis_from_char(arg[0]);
      if (a >= 0) arg++; else a = AXIS_X;
      if (arg[0] != '0' && arg[0] != '1') { printf("px0 = press X MIN, px1 = press X MAX (also py.., pz..)\n"); break; }
      uint32_t until = millis32() + VIRT_PRESS_MS;
      if (arg[0] == '1') virtMaxUntil[a] = until; else virtMinUntil[a] = until;
    } break;
    case 'r': reset_fault(arg[0] == 'k'); break;
    case 't': telemetry = !telemetry; break;
    case 'q': statusStream = (arg[0] == '1'); break;
    default: print_help();
  }
}

static void console_task(void *arg) {
  (void)arg;
  static char buf[1024];                    // a whole path fits on one line
  int len = 0;
  for (;;) {
    uint8_t c;
    if (uart_read_bytes(UART_NUM_0, &c, 1, portMAX_DELAY) != 1) continue;
    if (c == '\r' || c == '\n') {
      printf("\n");
      if (len > 0) { buf[len] = 0; run_command(buf); }
      len = 0;
    } else if ((c == 8 || c == 127) && len > 0) {         // backspace
      len--; printf("\b \b"); fflush(stdout);
    } else if (c >= 32 && len < (int)sizeof buf - 1) {
      buf[len++] = (char)c; putchar(c); fflush(stdout);    // echo what you type
    }
  }
}

// ----------------------------- Report task (core 1) -----------------------
static void fault_text(char *out, size_t n, motion_fault_t f, int axis) {
  if (axis >= 0) snprintf(out, n, "%s %c", motion_fault_name(f), motion_axis_name(axis));
  else           snprintf(out, n, "%s", motion_fault_name(f));
}

static void report_task(void *arg) {
  (void)arg;
  motion_fault_t lastFault = MF_NONE;
  uint32_t tTel = 0, tStat = 0, lastTicks = 0;

  for (;;) {
    uint32_t now = millis32();

    // Backup e-stop check from this core, in case the motion timer is starved
    if (kp_read().estop) raise_estop();

    // One consistent snapshot of the controller
    portENTER_CRITICAL(&motionMux);
    motion_t s = mc;
    bool keep = motion_keep_check(&mc) == KEEP_OK;   // "r k" would keep homing
    portEXIT_CRITICAL(&motionMux);

    char ft[32];
    fault_text(ft, sizeof ft, s.fault, s.faultAxis);
    if (s.fault != lastFault) {
      if (s.fault != MF_NONE) printf("\n*** FAULT: %s  (r to reset, then home)\n", ft);
      lastFault = s.fault;
    }

    // Coordinate per axis: -1 until homed; stays shown during an E-stop that can keep homing
    long coord[MOTION_AXES];
    for (int a = 0; a < MOTION_AXES; a++) {
      bool show = s.ax[a].homed || (keep && s.ax[a].homedAtFault);
      coord[a] = show ? (long)rail_steps_to_coord(s.ax[a].pos, s.ax[a].span) : -1L;
    }

    if (telemetry && now - tTel >= 1000) {
      tTel = now;
      printf("%-5s fault=%s controller=%lu Hz\n", motion_mode_name(s.mode), ft,
             (unsigned long)(motionTicks - lastTicks));
      lastTicks = motionTicks;
      for (int a = 0; a < MOTION_AXES; a++)
        printf("  %c coord=%ld steps=%ld span=%ld enc=%ld err=%ld homed=%d | MIN=%d MAX=%d (1 = pressed)\n",
               motion_axis_name(a), coord[a], (long)s.ax[a].pos, (long)s.ax[a].span, (long)s.ax[a].enc,
               (long)(s.ax[a].pos - s.ax[a].enc), (int)s.ax[a].homed,
               (int)lastInputs.limMin[a], (int)lastInputs.limMax[a]);
      printf("  ESTOP(*)=%d\n", (int)lastInputs.estop);
    }

    // Machine-readable status for the web page (web-control/index.html)
    if (statusStream && now - tStat >= STATUS_PERIOD_MS) {
      tStat = now;
      motion_inputs_t in = lastInputs;
      printf("@S mode=%s axis=%c phase=%s pos=%ld,%ld,%ld steps=%ld,%ld,%ld span=%ld,%ld,%ld "
             "homed=%d,%d,%d estop=%d min=%d,%d,%d max=%d,%d,%d keep=%d zon=%d path=%d/%d fault=%s\n",
             motion_mode_name(s.mode), motion_axis_name(motion_homing_axis(&s)),
             s.homePhase == HOME_SEEK_MIN ? "MIN" : "MAX",
             coord[0], coord[1], coord[2],
             (long)s.ax[0].pos, (long)s.ax[1].pos, (long)s.ax[2].pos,
             (long)s.ax[0].span, (long)s.ax[1].span, (long)s.ax[2].span,
             (int)s.ax[0].homed, (int)s.ax[1].homed, (int)s.ax[2].homed,
             (int)in.estop, (int)in.limMin[0], (int)in.limMin[1], (int)in.limMin[2],
             (int)in.limMax[0], (int)in.limMax[1], (int)in.limMax[2], (int)keep, USE_Z_AXIS, s.pathIdx, s.pathLen, ft);
    }

    vTaskDelay(pdMS_TO_TICKS(20));            // 50 Hz
  }
}

// ----------------------------- app_main -----------------------------------
void app_main(void) {
  // Stepper coils
  uint64_t coilMask = 0;
  for (int a = 0; a < MOTION_AXES; a++)
    for (int c = 0; c < 4; c++) coilMask |= 1ULL << COIL_PINS[a][c];
  gpio_config_t out = { .pin_bit_mask = coilMask, .mode = GPIO_MODE_OUTPUT };
  ESP_ERROR_CHECK(gpio_config(&out));
  for (int a = 0; a < MOTION_AXES; a++) coils_off(a);

  // Keypad columns C1 / C2 (internal pull-ups)
  gpio_config_t in = {
    .pin_bit_mask = (1ULL << PIN_KP_C1) | (1ULL << PIN_KP_C2),
    .mode = GPIO_MODE_INPUT, .pull_up_en = GPIO_PULLUP_ENABLE,
  };
  ESP_ERROR_CHECK(gpio_config(&in));
#if USE_YZ_KEYS
  gpio_config_t in34 = {                     // input-only pins: external 10k pull-ups
    .pin_bit_mask = (1ULL << PIN_KP_C3) | (USE_Z_AXIS ? (1ULL << PIN_KP_C4) : 0), .mode = GPIO_MODE_INPUT,
  };
  ESP_ERROR_CHECK(gpio_config(&in34));
#endif
  // Keypad row 1: open-drain so it can never drive high into the GND row through a key pair
  gpio_set_level(PIN_KP_R1, 1);              // released before the pin becomes an output
  gpio_config_t row = {
    .pin_bit_mask = 1ULL << PIN_KP_R1, .mode = GPIO_MODE_INPUT_OUTPUT_OD,
    .pull_up_en = GPIO_PULLUP_ENABLE,
  };
  ESP_ERROR_CHECK(gpio_config(&row));

  motion_config_t cfg = { .vmax = VMAX, .amax = AMAX, .homeSpeed = HOME_SPEED,
                          .homeMaxSteps = HOME_MAX_STEPS, .skipThreshold = SKIP_THRESHOLD,
                          .homeReturn = true };     // homing ends at 0,0
  motion_init(&mc, &cfg, hw_step, NULL);

  // Console input on UART0 (output still goes through printf)
  ESP_ERROR_CHECK(uart_driver_install(UART_NUM_0, 2048, 0, 0, NULL, 0));   // room for a whole path line

  // 1 kHz motion controller
  esp_timer_create_args_t targs = { .callback = motion_timer_cb, .name = "motion" };
  esp_timer_handle_t motionTimer;
  ESP_ERROR_CHECK(esp_timer_create(&targs, &motionTimer));
  ESP_ERROR_CHECK(esp_timer_start_periodic(motionTimer, 1000));

  xTaskCreatePinnedToCore(report_task,  "report",  6144, NULL, 2, NULL, 1);
  xTaskCreatePinnedToCore(console_task, "console", 4096, NULL, 3, NULL, 1);

  print_help();
  printf("Z axis %s. Y/Z limit keys %s.\n", USE_Z_AXIS ? "on" : "OFF (USE_Z_AXIS 0)",
         USE_YZ_KEYS ? "on (# 3, D A)" : "OFF: use py0/py1 (pz0/pz1) or the web page");
  printf("Start with: h  (%sX: 0 then 1, Y: # then 3)\n", USE_Z_AXIS ? "Z: D then A, " : "");
}
