/*
  main.c - one-axis gantry test bench, native ESP-IDF v6.0 (C)
  Board : Freenove ESP32-WROVER (target esp32). Leave the camera UNPLUGGED.
  Console: ESP-IDF monitor (115200). Type a command and press Enter.

  Runs at the same time:
    - 1 kHz motion controller (esp_timer): S-curve moves, homing, joystick jog,
      limit switches, e-stop, skip detection against a "virtual encoder"
    - sensor_task  (core 0): MPU6050 + Kalman filter, ultrasonic, vibration switch
    - ui_task      (core 1): joystick, servo, thermistor, LCD, fault LED + buzzer
    - console_task (core 1): serial commands

  Commands:
    h            home (motor turns backward until you press the MIN limit button)
    g <steps>    S-curve move to an absolute position, 0..4096 (home first)
    w a b c ...  visit up to 8 waypoints, ordered by nearest neighbour
    j            toggle joystick jog mode (pressing the joystick also toggles it)
    s <deg>      servo angle 0..180
    k <n>        simulate n skipped steps on the next moves (tests skip detection)
    x            stop now
    r            reset a fault (then home again); joystick press also resets
    t            toggle 1 s telemetry printout
    ?            help

  PIN MAP (same as the Arduino version)
    Stepper driver IN1..IN4 : GPIO 13, 14, 27, 26     I2C SDA/SCL : GPIO 21 / 22
    Ultrasonic TRIG / ECHO  : GPIO 23 / 35 (divider)  Servo       : GPIO 25
    Limit MIN / MAX         : GPIO 32 / 33 (to GND)   E-stop      : GPIO 4 (to GND)
    Joystick X / Y / SW     : GPIO 36 / 39 / 18       Vibration   : GPIO 19 (to GND)
    Thermistor              : GPIO 34                 Fault LED   : GPIO 2
    Buzzer (via NPN)        : GPIO 15
*/

#include <stdio.h>
#include <stdint.h>
#include <stdbool.h>
#include <stdlib.h>
#include <string.h>
#include <math.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/semphr.h"
#include "esp_timer.h"
#include "esp_rom_sys.h"
#include "esp_attr.h"
#include "driver/gpio.h"
#include "driver/i2c_master.h"
#include "driver/ledc.h"
#include "driver/uart.h"
#include "esp_adc/adc_oneshot.h"
#include "esp_adc/adc_cali.h"
#include "esp_adc/adc_cali_scheme.h"

// ----------------------------- Pins ---------------------------------------
#define PIN_IN1        GPIO_NUM_13
#define PIN_IN2        GPIO_NUM_14
#define PIN_IN3        GPIO_NUM_27
#define PIN_IN4        GPIO_NUM_26
#define PIN_SDA        GPIO_NUM_21
#define PIN_SCL        GPIO_NUM_22
#define PIN_US_TRIG    GPIO_NUM_23
#define PIN_US_ECHO    GPIO_NUM_35
#define PIN_SERVO      GPIO_NUM_25
#define PIN_LIMIT_MIN  GPIO_NUM_32
#define PIN_LIMIT_MAX  GPIO_NUM_33
#define PIN_ESTOP      GPIO_NUM_4
#define PIN_JOY_SW     GPIO_NUM_18
#define PIN_VIBRATION  GPIO_NUM_19
#define PIN_LED_FAULT  GPIO_NUM_2
#define PIN_BUZZER     GPIO_NUM_15
// ADC1 channels on the ESP32: GPIO36 = CH0, GPIO39 = CH3, GPIO34 = CH6
#define ADC_JOY_X      ADC_CHANNEL_0
#define ADC_JOY_Y      ADC_CHANNEL_3
#define ADC_THERM      ADC_CHANNEL_6

// ----------------------------- Tuning -------------------------------------
// Kit stepper (28BYJ-48), half-step mode: about 4096 half-steps per output turn.
#define VMAX              600.0f   // half-steps per second
#define AMAX              1200.0f  // peak acceleration, half-steps/s^2
#define HOME_SPEED        300.0f
#define AXIS_MAX          4096     // soft travel limit (one output turn)
#define HOME_BACKOFF      64
#define SKIP_THRESHOLD    4        // allowed commanded-vs-encoder error
#define COIL_OFF_DELAY_MS 500
#define TEMP_FAULT_C      50.0f
#define OBSTACLE_CM       8.0f
#define VIB_EVENTS_PER_S  8
#define THERM_R_FIXED     10000.0f
#define THERM_R0          10000.0f
#define THERM_BETA        3950.0f  // check your thermistor
#define PI_F              3.14159265f
#define RAD2DEG           57.2957795f

// ----------------------------- State --------------------------------------
typedef enum { M_IDLE, M_HOMING, M_MOVE, M_JOG, M_FAULT } mode_t_;
typedef enum { F_NONE, F_ESTOP, F_LIMIT, F_SKIP, F_OVERTEMP,
               F_OBSTACLE, F_VIBRATION, F_HOME_TIMEOUT } fault_t_;
static const char *MODE_NAMES[]  = {"IDLE", "HOME", "MOVE", "JOG", "FAULT"};
static const char *FAULT_NAMES[] = {"NONE", "ESTOP", "LIMIT", "SKIP", "OVERTEMP",
                                    "OBSTACLE", "VIBRATION", "HOME TIMEOUT"};

static portMUX_TYPE motionMux = portMUX_INITIALIZER_UNLOCKED;
static volatile mode_t_  mode        = M_IDLE;
static volatile fault_t_ fault       = F_NONE;
static volatile int32_t  posCmd      = 0;   // where the controller thinks the motor is
static volatile int32_t  encVirtual  = 0;   // "encoder": steps the motor actually made
static volatile int32_t  skipInject  = 0;
static volatile bool     homed       = false;
static volatile float    jogSetpoint = 0;

static volatile float sensRoll = NAN, sensPitch = NAN, sensTempC = NAN, sensDistCm = NAN;
static volatile uint32_t vibCount = 0;
static volatile int64_t  echoRise = 0;
static volatile int32_t  echoWidthUs = -1;

static i2c_master_bus_handle_t i2cBus;
static i2c_master_dev_handle_t lcdDev, mpuDev;
static SemaphoreHandle_t i2cMutex;
static bool lcdOk = false, mpuOk = false;

static adc_oneshot_unit_handle_t adc1;
static adc_cali_handle_t adcCali = NULL;
static float servoAngle = 90.0f;
static bool telemetry = false;

static inline uint32_t millis32(void) { return (uint32_t)(esp_timer_get_time() / 1000); }
static inline int32_t iabs32(int32_t v) { return v < 0 ? -v : v; }
static inline int32_t clampi(int32_t v, int32_t lo, int32_t hi) { return v < lo ? lo : (v > hi ? hi : v); }
static inline float clampf(float v, float lo, float hi) { return v < lo ? lo : (v > hi ? hi : v); }

// ----------------------------- S-curve profile ----------------------------
// Sine-shaped acceleration: accel starts and ends at zero, so jerk stays finite.
typedef struct { int32_t start, dist; float vp, ta, tc, total, dacc, t; } profile_t;
static profile_t prof;

static void plan_profile(profile_t *p, int32_t start, int32_t target) {
  p->start = start;
  p->dist  = target - start;
  float D  = fabsf((float)p->dist);
  float vp = VMAX;
  float ta = PI_F * vp / (2.0f * AMAX);
  float dacc = vp * ta / 2.0f;
  if (2.0f * dacc > D) {                       // too short to reach VMAX
    vp   = sqrtf(2.0f * AMAX * D / PI_F);
    ta   = PI_F * vp / (2.0f * AMAX);
    dacc = vp * ta / 2.0f;
  }
  p->vp = vp; p->ta = ta; p->dacc = dacc;
  p->tc = (vp > 0.0f) ? (D - 2.0f * dacc) / vp : 0.0f;
  if (p->tc < 0.0f) p->tc = 0.0f;
  p->total = 2.0f * ta + p->tc;
  p->t = 0.0f;
}

static float s_accel(const profile_t *p, float t) {
  if (p->ta <= 0.0f) return 0.0f;
  return 0.5f * p->vp * (t - p->ta / PI_F * sinf(PI_F * t / p->ta));
}

static float profile_pos(const profile_t *p, float t) {
  float D = fabsf((float)p->dist);
  if (t <= 0.0f) return 0.0f;
  if (t >= p->total) return D;
  if (t < p->ta) return s_accel(p, t);
  if (t < p->ta + p->tc) return p->dacc + p->vp * (t - p->ta);
  return D - s_accel(p, p->total - t);
}

// ----------------------------- Stepper ------------------------------------
static const uint8_t HALF_STEP[8][4] = {
  {1,0,0,0},{1,1,0,0},{0,1,0,0},{0,1,1,0},{0,0,1,0},{0,0,1,1},{0,0,0,1},{1,0,0,1}};
static uint8_t  phaseIdx   = 0;
static bool     coilsOn    = false;
static uint32_t lastStepMs = 0;

static void coils_write(uint8_t i) {
  gpio_set_level(PIN_IN1, HALF_STEP[i][0]);
  gpio_set_level(PIN_IN2, HALF_STEP[i][1]);
  gpio_set_level(PIN_IN3, HALF_STEP[i][2]);
  gpio_set_level(PIN_IN4, HALF_STEP[i][3]);
  coilsOn = true;
}

static void coils_off(void) {
  gpio_set_level(PIN_IN1, 0); gpio_set_level(PIN_IN2, 0);
  gpio_set_level(PIN_IN3, 0); gpio_set_level(PIN_IN4, 0);
  coilsOn = false;
}

// A simulated skip advances the command but not the motor or the encoder.
static void do_step(int dir) {
  posCmd += dir;
  if (skipInject > 0) { skipInject--; return; }
  phaseIdx = (uint8_t)((phaseIdx + (dir > 0 ? 1 : 7)) & 7);
  coils_write(phaseIdx);
  encVirtual += dir;
  lastStepMs = millis32();
}

static void raise_fault_locked(fault_t_ f) {     // call only while holding motionMux
  if (fault == F_NONE) { fault = f; mode = M_FAULT; homed = false; }
}

static void raise_fault(fault_t_ f) {
  portENTER_CRITICAL(&motionMux);
  raise_fault_locked(f);
  portEXIT_CRITICAL(&motionMux);
}

// ----------------------------- 1 kHz motion controller --------------------
static float jogVel = 0, jogAcc = 0, homeVel = 0, homeAcc = 0;
static int32_t homeSteps = 0;

static void motion_tick(void *arg) {
  (void)arg;
  const float dt = 0.001f;
  bool estop  = gpio_get_level(PIN_ESTOP) == 0;
  bool limMin = gpio_get_level(PIN_LIMIT_MIN) == 0;
  bool limMax = gpio_get_level(PIN_LIMIT_MAX) == 0;
  uint32_t now = millis32();

  portENTER_CRITICAL(&motionMux);
  if (estop) raise_fault_locked(F_ESTOP);
  if (iabs32(posCmd - encVirtual) > SKIP_THRESHOLD) raise_fault_locked(F_SKIP);

  switch (mode) {
    case M_FAULT:
      jogVel = jogAcc = homeVel = homeAcc = 0;
      break;

    case M_IDLE:
      if (coilsOn && now - lastStepMs > COIL_OFF_DELAY_MS) coils_off();
      break;

    case M_MOVE: {
      prof.t += dt;
      int32_t s = (int32_t)lroundf(profile_pos(&prof, prof.t));
      int32_t target = prof.start + (prof.dist >= 0 ? s : -s);
      for (int i = 0; i < 2 && posCmd != target; i++) {
        int dir = (target > posCmd) ? 1 : -1;
        if ((dir < 0 && limMin) || (dir > 0 && limMax)) { raise_fault_locked(F_LIMIT); break; }
        do_step(dir);
      }
      if (mode == M_MOVE && prof.t >= prof.total && posCmd == target) mode = M_IDLE;
    } break;

    case M_JOG: {
      float sp = jogSetpoint, dv = AMAX * dt;
      jogVel = (jogVel < sp) ? fminf(jogVel + dv, sp) : fmaxf(jogVel - dv, sp);
      bool blockNeg = limMin || (homed && posCmd <= 0);
      bool blockPos = limMax || (homed && posCmd >= AXIS_MAX);
      if ((jogVel < 0 && blockNeg) || (jogVel > 0 && blockPos)) { jogVel = 0; jogAcc = 0; }
      jogAcc += jogVel * dt;
      while (jogAcc >= 1.0f)  { do_step(+1); jogAcc -= 1.0f; }
      while (jogAcc <= -1.0f) { do_step(-1); jogAcc += 1.0f; }
      if (jogVel == 0 && coilsOn && now - lastStepMs > COIL_OFF_DELAY_MS) coils_off();
    } break;

    case M_HOMING:
      if (limMin) {                       // found zero: reset counters, back off
        posCmd = 0; encVirtual = 0; homeVel = homeAcc = 0; homed = true;
        plan_profile(&prof, 0, HOME_BACKOFF);
        mode = M_MOVE;
        break;
      }
      homeVel = fminf(homeVel + AMAX * dt, HOME_SPEED);
      homeAcc -= homeVel * dt;
      while (homeAcc <= -1.0f) { do_step(-1); homeAcc += 1.0f; homeSteps++; }
      if (homeSteps > 3 * AXIS_MAX) raise_fault_locked(F_HOME_TIMEOUT);
      break;
  }
  if (mode == M_FAULT && coilsOn) coils_off();
  portEXIT_CRITICAL(&motionMux);
}

// Commands -----------------------------------------------------------------
static bool start_move(int32_t target) {
  if (!homed) { printf("Home first (h).\n"); return false; }
  target = clampi(target, 0, AXIS_MAX);
  bool ok = false;
  portENTER_CRITICAL(&motionMux);
  if (mode == M_IDLE) { plan_profile(&prof, posCmd, target); mode = M_MOVE; ok = true; }
  portEXIT_CRITICAL(&motionMux);
  if (!ok) printf("Busy or faulted.\n");
  return ok;
}

static void start_homing(void) {
  portENTER_CRITICAL(&motionMux);
  if (mode == M_IDLE || mode == M_JOG) {
    homed = false; homeSteps = 0; homeVel = homeAcc = 0; mode = M_HOMING;
  }
  portEXIT_CRITICAL(&motionMux);
}

static void toggle_jog(void) {
  portENTER_CRITICAL(&motionMux);
  if (mode == M_IDLE)     { jogVel = jogAcc = 0; mode = M_JOG; }
  else if (mode == M_JOG) { mode = M_IDLE; }
  portEXIT_CRITICAL(&motionMux);
}

static void stop_now(void) {
  portENTER_CRITICAL(&motionMux);
  if (mode != M_FAULT) mode = M_IDLE;
  portEXIT_CRITICAL(&motionMux);
}

static void reset_fault(void) {
  if (gpio_get_level(PIN_ESTOP) == 0) { printf("Release the e-stop first.\n"); return; }
  portENTER_CRITICAL(&motionMux);
  fault = F_NONE; mode = M_IDLE; homed = false;
  skipInject = 0; encVirtual = posCmd;   // true position unknown: re-home required
  portEXIT_CRITICAL(&motionMux);
  printf("Fault cleared. Home again (h) before moving.\n");
}

// ----------------------------- I2C: LCD + MPU6050 -------------------------
static bool i2c_has(uint8_t addr) {
  return i2c_master_probe(i2cBus, addr, 50) == ESP_OK;
}

static i2c_master_dev_handle_t i2c_add(uint8_t addr) {
  i2c_device_config_t cfg = {
    .dev_addr_length = I2C_ADDR_BIT_LEN_7,
    .device_address  = addr,
    .scl_speed_hz    = 100000,
  };
  i2c_master_dev_handle_t h = NULL;
  ESP_ERROR_CHECK(i2c_master_bus_add_device(i2cBus, &cfg, &h));
  return h;
}

// HD44780 through a PCF8574 backpack (RS=P0, EN=P2, BL=P3, D4-D7=P4-P7).
// Each nibble is one I2C write of two bytes: EN high, then EN low.
static void lcd_nibble(uint8_t nib, uint8_t rs) {
  uint8_t d = (uint8_t)((nib << 4) | (rs ? 0x01 : 0x00) | 0x08);
  uint8_t buf[2] = { (uint8_t)(d | 0x04), d };
  i2c_master_transmit(lcdDev, buf, 2, 50);
  esp_rom_delay_us(50);
}
static void lcd_send(uint8_t v, uint8_t rs) { lcd_nibble(v >> 4, rs); lcd_nibble(v & 0x0F, rs); }
static void lcd_cmd(uint8_t c) { lcd_send(c, 0); if (c <= 0x03) vTaskDelay(pdMS_TO_TICKS(2)); }

static void lcd_init(void) {
  uint8_t bl = 0x08;
  vTaskDelay(pdMS_TO_TICKS(50));
  i2c_master_transmit(lcdDev, &bl, 1, 50);
  vTaskDelay(pdMS_TO_TICKS(5));
  lcd_nibble(0x03, 0); vTaskDelay(pdMS_TO_TICKS(5));
  lcd_nibble(0x03, 0); esp_rom_delay_us(150);
  lcd_nibble(0x03, 0); esp_rom_delay_us(150);
  lcd_nibble(0x02, 0);                 // 4-bit mode
  lcd_cmd(0x28);                       // 2 lines, 5x8 font
  lcd_cmd(0x0C);                       // display on, cursor off
  lcd_cmd(0x06);                       // entry mode
  lcd_cmd(0x01);                       // clear
}

static void lcd_line(uint8_t row, const char *s) {   // writes exactly 16 characters
  if (!lcdOk) return;
  if (xSemaphoreTake(i2cMutex, pdMS_TO_TICKS(50)) != pdTRUE) return;
  lcd_cmd(0x80 | (row ? 0x40 : 0x00));
  for (int i = 0; i < 16; i++) lcd_send((uint8_t)(*s ? *s++ : ' '), 1);
  xSemaphoreGive(i2cMutex);
}

#define MPU_ADDR 0x68
static void mpu_write(uint8_t reg, uint8_t val) {
  uint8_t b[2] = { reg, val };
  i2c_master_transmit(mpuDev, b, 2, 50);
}
static void mpu_init(void) {
  mpu_write(0x6B, 0x00);   // wake up
  mpu_write(0x1A, 0x03);   // low-pass filter ~44 Hz
  mpu_write(0x1B, 0x00);   // gyro +/-250 deg/s (131 LSB per deg/s)
  mpu_write(0x1C, 0x00);   // accel +/-2 g     (16384 LSB per g)
}
static bool mpu_read(float *ax, float *ay, float *az, float *gx, float *gy) {
  uint8_t reg = 0x3B, d[14];
  if (i2c_master_transmit_receive(mpuDev, &reg, 1, d, 14, 50) != ESP_OK) return false;
  int16_t r[7];
  for (int i = 0; i < 7; i++) r[i] = (int16_t)((d[2 * i] << 8) | d[2 * i + 1]);
  *ax = r[0] / 16384.0f; *ay = r[1] / 16384.0f; *az = r[2] / 16384.0f;
  *gx = r[4] / 131.0f;   *gy = r[5] / 131.0f;      // r[3] is temperature
  return true;
}

// Two-state Kalman filter (angle + gyro bias) --------------------------------
typedef struct { float angle, bias, P00, P01, P10, P11, Qa, Qb; } kalman_t;

static float kalman_update(kalman_t *k, float meas, float rate, float dt, float R) {
  k->angle += dt * (rate - k->bias);                    // predict
  k->P00 += dt * (dt * k->P11 - k->P01 - k->P10 + k->Qa);
  k->P01 -= dt * k->P11;
  k->P10 -= dt * k->P11;
  k->P11 += k->Qb * dt;
  float S = k->P00 + R, K0 = k->P00 / S, K1 = k->P10 / S;   // update
  float y = meas - k->angle;
  k->angle += K0 * y;
  k->bias  += K1 * y;
  float p00 = k->P00, p01 = k->P01;
  k->P00 -= K0 * p00; k->P01 -= K0 * p01;
  k->P10 -= K1 * p00; k->P11 -= K1 * p01;
  return k->angle;
}

// ----------------------------- Interrupts ---------------------------------
static void IRAM_ATTR vib_isr(void *arg) { (void)arg; vibCount++; }

static void IRAM_ATTR echo_isr(void *arg) {
  (void)arg;
  int64_t now = esp_timer_get_time();
  if (gpio_get_level(PIN_US_ECHO)) echoRise = now;
  else if (echoRise) { echoWidthUs = (int32_t)(now - echoRise); echoRise = 0; }
}

// ----------------------------- Sensor task (core 0) -----------------------
static void sensor_task(void *arg) {
  (void)arg;
  kalman_t kRoll = { .Qa = 0.001f, .Qb = 0.003f }, kPitch = { .Qa = 0.001f, .Qb = 0.003f };
  int64_t lastUs = esp_timer_get_time();
  uint32_t tSonic = 0, tVib = 0, lastVib = vibCount;
  int closeCount = 0;
  bool kInit = false;

  for (;;) {
    uint32_t nowMs = millis32();

    if (mpuOk && xSemaphoreTake(i2cMutex, pdMS_TO_TICKS(20)) == pdTRUE) {
      float ax, ay, az, gx, gy;
      bool ok = mpu_read(&ax, &ay, &az, &gx, &gy);
      xSemaphoreGive(i2cMutex);
      int64_t nowUs = esp_timer_get_time();
      float dt = (float)(nowUs - lastUs) / 1e6f;
      lastUs = nowUs;
      if (ok) {
        float accRoll  = atan2f(ay, az) * RAD2DEG;
        float accPitch = atanf(-ax / sqrtf(ay * ay + az * az)) * RAD2DEG;
        float amag = sqrtf(ax * ax + ay * ay + az * az);
        // Trust the accelerometer less while something other than gravity acts on it
        float R = (fabsf(amag - 1.0f) > 0.15f) ? 0.5f : 0.03f;
        if (!kInit) { kRoll.angle = accRoll; kPitch.angle = accPitch; kInit = true; }
        else {
          sensRoll  = kalman_update(&kRoll, accRoll, gx, dt, R);
          sensPitch = kalman_update(&kPitch, accPitch, gy, dt, R);
        }
      }
    }

    bool moving = (mode == M_MOVE || mode == M_JOG || mode == M_HOMING);

    if (nowMs - tSonic >= 100) {              // read the last ping, then fire a new one
      tSonic = nowMs;
      int32_t w = echoWidthUs;
      float d = (w > 0 && w < 25000) ? (float)w / 58.0f : NAN;
      sensDistCm = d;
      if (!isnan(d) && d < OBSTACLE_CM) closeCount++; else closeCount = 0;
      if (moving && closeCount >= 3) raise_fault(F_OBSTACLE);
      echoWidthUs = -1; echoRise = 0;
      gpio_set_level(PIN_US_TRIG, 1); esp_rom_delay_us(10); gpio_set_level(PIN_US_TRIG, 0);
    }

    if (nowMs - tVib >= 1000) {
      tVib = nowMs;
      uint32_t c = vibCount;
      if ((int)(c - lastVib) >= VIB_EVENTS_PER_S) raise_fault(F_VIBRATION);
      lastVib = c;
    }

    vTaskDelay(pdMS_TO_TICKS(10));            // ~100 Hz IMU loop
  }
}

// ----------------------------- Servo (LEDC) -------------------------------
static void servo_init(void) {
  ledc_timer_config_t tc = {
    .speed_mode = LEDC_LOW_SPEED_MODE, .duty_resolution = LEDC_TIMER_14_BIT,
    .timer_num = LEDC_TIMER_0, .freq_hz = 50, .clk_cfg = LEDC_AUTO_CLK,
  };
  ESP_ERROR_CHECK(ledc_timer_config(&tc));
  ledc_channel_config_t cc = {
    .gpio_num = PIN_SERVO, .speed_mode = LEDC_LOW_SPEED_MODE, .channel = LEDC_CHANNEL_0,
    .timer_sel = LEDC_TIMER_0, .duty = 0, .hpoint = 0,
  };
  ESP_ERROR_CHECK(ledc_channel_config(&cc));
}

static void servo_write(float deg) {
  servoAngle = clampf(deg, 0.0f, 180.0f);
  uint32_t us = 500 + (uint32_t)(servoAngle / 180.0f * 2000.0f);    // 0.5-2.5 ms
  uint32_t duty = us * ((1u << 14) - 1) / 20000u;
  ledc_set_duty(LEDC_LOW_SPEED_MODE, LEDC_CHANNEL_0, duty);
  ledc_update_duty(LEDC_LOW_SPEED_MODE, LEDC_CHANNEL_0);
}

// ----------------------------- ADC ----------------------------------------
static int adc_raw(adc_channel_t ch) {
  int v = 0;
  adc_oneshot_read(adc1, ch, &v);
  return v;
}

static float read_thermistor_c(void) {
  int raw = adc_raw(ADC_THERM), mv = 0;
  if (adcCali) adc_cali_raw_to_voltage(adcCali, raw, &mv);
  else mv = raw * 3300 / 4095;
  if (mv < 50 || mv > 3250) return NAN;                // open or shorted
  float r = THERM_R_FIXED * (float)mv / (3300.0f - (float)mv);
  float invT = 1.0f / 298.15f + logf(r / THERM_R0) / THERM_BETA;
  return 1.0f / invT - 273.15f;
}

static void adc_init(void) {
  adc_oneshot_unit_init_cfg_t ucfg = { .unit_id = ADC_UNIT_1 };
  ESP_ERROR_CHECK(adc_oneshot_new_unit(&ucfg, &adc1));
  adc_oneshot_chan_cfg_t ccfg = { .atten = ADC_ATTEN_DB_12, .bitwidth = ADC_BITWIDTH_12 };
  ESP_ERROR_CHECK(adc_oneshot_config_channel(adc1, ADC_JOY_X, &ccfg));
  ESP_ERROR_CHECK(adc_oneshot_config_channel(adc1, ADC_JOY_Y, &ccfg));
  ESP_ERROR_CHECK(adc_oneshot_config_channel(adc1, ADC_THERM, &ccfg));
#if ADC_CALI_SCHEME_LINE_FITTING_SUPPORTED
  adc_cali_line_fitting_config_t cal = {
    .unit_id = ADC_UNIT_1, .atten = ADC_ATTEN_DB_12, .bitwidth = ADC_BITWIDTH_12 };
  if (adc_cali_create_scheme_line_fitting(&cal, &adcCali) != ESP_OK) adcCali = NULL;
#endif
}

// ----------------------------- Waypoints (nearest neighbour) --------------
static int32_t wq[8];
static volatile int wqLen = 0, wqIdx = 0;

static void plan_waypoints(const int32_t *pts, int n) {
  int32_t cur = posCmd;
  bool used[8] = {false};
  int len = 0;
  printf("Visit order:");
  for (int k = 0; k < n; k++) {
    int best = 0; int32_t bestD = INT32_MAX;
    for (int i = 0; i < n; i++) {
      if (used[i]) continue;
      int32_t d = iabs32(pts[i] - cur);
      if (d < bestD) { bestD = d; best = i; }
    }
    used[best] = true;
    wq[len++] = pts[best];
    cur = pts[best];
    printf(" %ld", (long)pts[best]);
  }
  printf("\n");
  wqIdx = 0;
  wqLen = len;
}

// ----------------------------- Console task -------------------------------
static void print_help(void) {
  printf("\nCommands: h | g <steps> | w a b c.. | j | s <deg> | k <n> | x | r | t | ?\n"
         "  h home, g move 0..4096, w waypoints, j jog, s servo, k simulate skips,\n"
         "  x stop, r reset fault, t telemetry\n");
}

static void run_command(char *buf) {
  char cmd = buf[0];
  char *arg = buf + 1;
  switch (cmd) {
    case 'h': start_homing(); printf("Homing: press the MIN button to mark zero.\n"); break;
    case 'g': start_move((int32_t)atol(arg)); break;
    case 'w': {
      if (!homed) { printf("Home first (h).\n"); break; }
      int32_t pts[8]; int n = 0;
      char *tok = strtok(arg, " ,");
      while (tok && n < 8) { pts[n++] = clampi((int32_t)atol(tok), 0, AXIS_MAX); tok = strtok(NULL, " ,"); }
      if (n) plan_waypoints(pts, n);
    } break;
    case 'j': toggle_jog(); printf("%s\n", mode == M_JOG ? "Jog ON" : "Jog OFF"); break;
    case 's': servo_write((float)atof(arg)); break;
    case 'k': { long n = atol(arg); skipInject = (int32_t)(n > 0 ? n : 8);
                printf("Will skip %ld step(s).\n", (long)skipInject); } break;
    case 'x': wqLen = 0; wqIdx = 0; stop_now(); break;
    case 'r': reset_fault(); break;
    case 't': telemetry = !telemetry; break;
    case '?': print_help(); break;
    default: print_help();
  }
}

static void console_task(void *arg) {
  (void)arg;
  char buf[64];
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
    } else if (c >= 32 && len < 63) {
      buf[len++] = (char)c; putchar(c); fflush(stdout);    // echo what you type
    }
  }
}

// ----------------------------- UI task (core 1) ---------------------------
static void fmt_val(char *out, size_t n, float v, const char *fmt) {
  if (isnan(v)) snprintf(out, n, "--"); else snprintf(out, n, fmt, v);
}

static void ui_task(void *arg) {
  (void)arg;
  // Joystick centre calibration (leave it untouched at power-up)
  long sx = 0, sy = 0;
  for (int i = 0; i < 32; i++) { sx += adc_raw(ADC_JOY_X); sy += adc_raw(ADC_JOY_Y); vTaskDelay(1); }
  int cx = (int)(sx / 32), cy = (int)(sy / 32);

  fault_t_ lastFault = F_NONE;
  int beeps = 0, lastSw = 1;
  uint32_t tBeep = 0, tLcd = 0, tTel = 0, tTemp = 0, lastSwMs = 0;

  for (;;) {
    uint32_t now = millis32();

    // Joystick: X jogs the stepper, Y nudges the servo, press toggles jog / resets
    float nx = (adc_raw(ADC_JOY_X) - cx) / 2048.0f, ny = (adc_raw(ADC_JOY_Y) - cy) / 2048.0f;
    if (fabsf(nx) < 0.12f) nx = 0; else nx = clampf(nx, -1, 1);
    if (fabsf(ny) < 0.12f) ny = 0; else ny = clampf(ny, -1, 1);
    if (mode == M_JOG) {
      jogSetpoint = nx * VMAX;
      if (ny != 0) servo_write(servoAngle + ny * 90.0f * 0.02f);
    } else {
      jogSetpoint = 0;
    }
    int sw = gpio_get_level(PIN_JOY_SW);
    if (sw == 0 && lastSw == 1 && now - lastSwMs > 250) {
      lastSwMs = now;
      if (mode == M_FAULT) reset_fault(); else toggle_jog();
    }
    lastSw = sw;

    // Waypoint queue
    if (mode == M_FAULT) { wqLen = 0; wqIdx = 0; }
    else if (wqIdx < wqLen && mode == M_IDLE) { int i = wqIdx; wqIdx = i + 1; start_move(wq[i]); }

    // Thermistor (all ADC reads stay in this task)
    if (now - tTemp >= 500) {
      tTemp = now;
      float t = read_thermistor_c();
      sensTempC = t;
      if (!isnan(t) && t > TEMP_FAULT_C) raise_fault(F_OVERTEMP);
    }

    // Fault LED + 3 beeps on a new fault
    fault_t_ f = fault;
    if (f != lastFault) {
      if (f != F_NONE) {
        printf("\n*** FAULT: %s  (r to reset, then home)\n", FAULT_NAMES[f]);
        beeps = 6; tBeep = now;
      }
      lastFault = f;
    }
    gpio_set_level(PIN_LED_FAULT, (f != F_NONE && (now / 250) % 2) ? 1 : 0);
    if (beeps > 0 && now - tBeep >= 200) { tBeep = now; beeps--; gpio_set_level(PIN_BUZZER, beeps % 2); }
    if (beeps == 0) gpio_set_level(PIN_BUZZER, 0);

    // LCD (buffers are oversized; lcd_line writes the first 16 characters)
    if (now - tLcd >= 250) {
      tLcd = now;
      char l0[40], l1[40], a[16], b[16];
      snprintf(l0, sizeof l0, "%-5s P:%5ld %c", MODE_NAMES[mode], (long)posCmd, homed ? 'H' : ' ');
      if (f != F_NONE) {
        snprintf(l1, sizeof l1, "F:%s", FAULT_NAMES[f]);
      } else if ((now / 2000) % 2) {
        fmt_val(a, sizeof a, sensTempC, "%.1f");
        fmt_val(b, sizeof b, sensDistCm, "%.0f");
        snprintf(l1, sizeof l1, "T:%sC D:%scm", a, b);
      } else {
        fmt_val(a, sizeof a, sensRoll, "%+.1f");
        fmt_val(b, sizeof b, sensPitch, "%+.1f");
        snprintf(l1, sizeof l1, "R:%s P:%s", a, b);
      }
      lcd_line(0, l0);
      lcd_line(1, l1);
    }

    if (telemetry && now - tTel >= 1000) {
      tTel = now;
      printf("%-5s pos=%ld enc=%ld err=%ld homed=%d | T=%.1fC dist=%.1fcm roll=%.1f pitch=%.1f servo=%.0f\n",
             MODE_NAMES[mode], (long)posCmd, (long)encVirtual, (long)(posCmd - encVirtual), (int)homed,
             sensTempC, sensDistCm, sensRoll, sensPitch, servoAngle);
    }

    vTaskDelay(pdMS_TO_TICKS(20));            // 50 Hz
  }
}

// ----------------------------- app_main -----------------------------------
void app_main(void) {
  // Outputs
  gpio_config_t out = {
    .pin_bit_mask = (1ULL << PIN_IN1) | (1ULL << PIN_IN2) | (1ULL << PIN_IN3) | (1ULL << PIN_IN4) |
                    (1ULL << PIN_US_TRIG) | (1ULL << PIN_LED_FAULT) | (1ULL << PIN_BUZZER),
    .mode = GPIO_MODE_OUTPUT,
  };
  ESP_ERROR_CHECK(gpio_config(&out));
  coils_off();
  gpio_set_level(PIN_US_TRIG, 0); gpio_set_level(PIN_LED_FAULT, 0); gpio_set_level(PIN_BUZZER, 0);

  // Buttons and switches (to GND, internal pull-ups)
  gpio_config_t in = {
    .pin_bit_mask = (1ULL << PIN_LIMIT_MIN) | (1ULL << PIN_LIMIT_MAX) | (1ULL << PIN_ESTOP) |
                    (1ULL << PIN_JOY_SW),
    .mode = GPIO_MODE_INPUT, .pull_up_en = GPIO_PULLUP_ENABLE,
  };
  ESP_ERROR_CHECK(gpio_config(&in));
  gpio_config_t vib = {
    .pin_bit_mask = 1ULL << PIN_VIBRATION, .mode = GPIO_MODE_INPUT,
    .pull_up_en = GPIO_PULLUP_ENABLE, .intr_type = GPIO_INTR_NEGEDGE,
  };
  ESP_ERROR_CHECK(gpio_config(&vib));
  gpio_config_t echo = {                     // GPIO35 is input-only, no pull-ups
    .pin_bit_mask = 1ULL << PIN_US_ECHO, .mode = GPIO_MODE_INPUT, .intr_type = GPIO_INTR_ANYEDGE,
  };
  ESP_ERROR_CHECK(gpio_config(&echo));
  ESP_ERROR_CHECK(gpio_install_isr_service(0));
  ESP_ERROR_CHECK(gpio_isr_handler_add(PIN_VIBRATION, vib_isr, NULL));
  ESP_ERROR_CHECK(gpio_isr_handler_add(PIN_US_ECHO, echo_isr, NULL));

  // I2C bus, LCD (0x27 or 0x3F) and MPU6050 (0x68)
  i2c_master_bus_config_t bus = {
    .i2c_port = I2C_NUM_0, .sda_io_num = PIN_SDA, .scl_io_num = PIN_SCL,
    .clk_source = I2C_CLK_SRC_DEFAULT, .glitch_ignore_cnt = 7,
    .flags.enable_internal_pullup = true,
  };
  ESP_ERROR_CHECK(i2c_new_master_bus(&bus, &i2cBus));
  i2cMutex = xSemaphoreCreateMutex();
  uint8_t lcdAddr = i2c_has(0x27) ? 0x27 : (i2c_has(0x3F) ? 0x3F : 0);
  if (lcdAddr) { lcdDev = i2c_add(lcdAddr); lcdOk = true; lcd_init();
                 lcd_line(0, "Gantry testbench"); lcd_line(1, "Starting..."); }
  if (i2c_has(MPU_ADDR)) { mpuDev = i2c_add(MPU_ADDR); mpuOk = true; mpu_init(); }
  printf("LCD %s, MPU6050 %s\n", lcdOk ? "found" : "NOT found", mpuOk ? "found" : "NOT found");

  servo_init();
  servo_write(90);
  adc_init();

  // Console input on UART0 (output still goes through printf)
  ESP_ERROR_CHECK(uart_driver_install(UART_NUM_0, 256, 0, 0, NULL, 0));

  // 1 kHz motion controller
  esp_timer_create_args_t targs = { .callback = motion_tick, .name = "motion" };
  esp_timer_handle_t motionTimer;
  ESP_ERROR_CHECK(esp_timer_create(&targs, &motionTimer));
  ESP_ERROR_CHECK(esp_timer_start_periodic(motionTimer, 1000));

  xTaskCreatePinnedToCore(sensor_task,  "sensors", 4096, NULL, 2, NULL, 0);
  xTaskCreatePinnedToCore(ui_task,      "ui",      6144, NULL, 2, NULL, 1);
  xTaskCreatePinnedToCore(console_task, "console", 4096, NULL, 3, NULL, 1);

  print_help();
  printf("Start with: h  (then press the MIN button by hand)\n");
}
