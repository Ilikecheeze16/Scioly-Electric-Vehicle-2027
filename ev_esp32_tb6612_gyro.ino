// ============================================================================
//  EVENT SETTINGS  -  EDIT THESE THREE NUMBERS AT THE COMPETITION, THEN UPLOAD
// ============================================================================
const float EVENT_DIST_CM   = 500.0f;   // Start Point  -> Target Point   (centimetres)
const float EVENT_TIME_S    = 15.0f;    // Target Time                    (seconds)
const float EVENT_BOTTLE_CM = 100.0f;   // Target Point -> Bottle Line    (centimetres)
// After you change any of these and upload, the car uses the new values.
// You can also change them over serial (d / t / b); those stay saved across
// power cycles until you upload new numbers here or type 'evt'.
// ============================================================================

/*
  Science Olympiad 2027 - Electric Vehicle C
  ESP32 DevKit V1 + 2x TB6612FNG (4 motors) + MPU-9250 (gyro heading hold)
  Open-loop, time-based distance; gyro keeps the car driving straight.
  The speed table is now edited over serial and saved to flash - no re-upload.

  WIRING (both TB6612 drivers share these signal wires)
    GPIO25 -> PWMA (both)   GPIO26 -> AIN1 (both)   GPIO27 -> AIN2 (both)
    GPIO33 -> PWMB (both)   GPIO18 -> BIN1 (both)   GPIO19 -> BIN2 (both)
    Channel A = LEFT side motors, channel B = RIGHT side motors
    3V3 -> VCC and STBY of both drivers     Pack + (after switch) -> VM of both
    One common ground: pack -, buck, ESP32, both drivers, MPU-9250
    GPIO4 -> START button -> GND
    MPU-9250: VCC -> 3V3, GND -> GND, SDA -> GPIO21, SCL -> GPIO22

  SERIAL COMMANDS (115200 baud, line ending: Newline)
    d <cm>  t <sec>  b <cm>   target distance / target time / bottle line
    fs <x>  rs <x>            forward / reverse distance scale
    kp <x>                    heading-hold gain
    gyro <0|1>                gyro heading hold off / on
    zero                      re-zero the gyro (keep the car still!)
    gtest                     print heading for 6 s (turn the car by hand)
    p                         print the plan
    cal <pwm> <ms>            test drive forward at a fixed PWM
    evt                       reload the EVENT SETTINGS from the top of the sketch

  SPEED TABLE COMMANDS
    meas <pwm> <d1> <d2>      easiest: d1 = cm travelled in a 2000 ms cal run,
                              d2 = cm travelled in a 4000 ms cal run (same PWM).
                              Speed = (d2 - d1) / 2 is worked out and stored.
    add <pwm> <cm/s>          store a speed you already know
    del <pwm>                 remove a row
    tab                       show the table
    tabrst                    back to the placeholder table
  The first add/meas clears the placeholder rows. Up to 10 rows, saved to flash.
*/

#include <Arduino.h>
#include <Preferences.h>
#include <Wire.h>

// Types must be declared before any function so the Arduino IDE's
// auto-generated prototypes can see them.
enum Dir { DIR_FWD, DIR_REV };
struct Plan {
  float fwd;    // cm
  float rev;    // cm
  float v;      // cm/s needed to hit the target time
  bool  ok;
};

// ======================= PINS =======================
const int PIN_ENA   = 25;   // motor A PWM (TB6612 PWMA)
const int PIN_IN1   = 26;   // AIN1
const int PIN_IN2   = 27;   // AIN2
const int PIN_ENB   = 33;   // motor B PWM (TB6612 PWMB)
const int PIN_IN3   = 18;   // BIN1
const int PIN_IN4   = 19;   // BIN2
const int PIN_START = 4;    // button to GND
const int PIN_LED   = 2;    // onboard LED on most DevKit V1 boards
const int PIN_SDA   = 21;   // MPU-9250
const int PIN_SCL   = 22;   // MPU-9250

// ======================= TRACK VALUES (announced after impound) =======================
float targetDistCm = EVENT_DIST_CM;     // set from EVENT SETTINGS at the top
float targetTimeS  = EVENT_TIME_S;
float bottleLineCm = EVENT_BOTTLE_CM;
bool  eventFromSketch = false;          // true if this boot used the sketch's EVENT SETTINGS

// ======================= VEHICLE / TUNING =======================
const float VEHICLE_LEN_CM    = 30.0f;  // MP (rear) to front edge of pusher: MEASURE THIS
const float LINE_OVERSHOOT_CM = 2.0f;   // how far past the line the pusher front should end up

// Baseline motor trims (values 0.80 - 1.00). With the gyro on, leave both near 1.00.
const float MOTOR_A_TRIM = 1.00f;
const float MOTOR_B_TRIM = 1.00f;

// Fine-tune scales (changeable over serial). >1 = go farther, <1 = go shorter
float fwdScale = 1.00f;
float revScale = 1.00f;

const int MIN_PWM        = 80;    // lowest duty that reliably moves the vehicle (tune)
const int MAX_DUTY       = 175;   // ~69% cap: keeps ~9 V pack near 6 V at the motors
const int RAMP_MS        = 400;   // soft start / soft stop time
const int BRAKE_MS       = 120;   // active brake time at each stop
const int TURN_PAUSE_MS  = 350;   // pause between forward and reverse
const int START_DELAY_MS = 250;   // delay after button press (lets the pencil lift)

// ======================= GYRO / HEADING HOLD =======================
const uint8_t MPU_ADDR          = 0x68;    // 0x69 if AD0 is high
const float   GYRO_LSB_PER_DPS  = 131.0f;  // +/-250 dps range
const float   GYRO_SIGN         = 1.0f;    // set to -1.0f if heading goes NEGATIVE when you turn the car LEFT
const bool    MOTOR_A_IS_LEFT   = true;    // set to false if motor A is the car's RIGHT side
float         KP                = 3.0f;    // PWM counts per degree of heading error
const float   KD                = 0.2f;    // PWM counts per deg/s of turn rate
const float   MAX_CORR          = 50.0f;   // largest correction in PWM counts
bool  useGyro = true;
bool  gyroOk  = false;
float gyroBias = 0.0f;       // raw counts
float heading  = 0.0f;       // degrees, + = turned left (counter-clockwise)
float rateDps  = 0.0f;
uint32_t lastGyroUs = 0;

// ======================= SPEED TABLE (edited over serial, saved to flash) =======================
// PWM -> cm/s. Both columns must rise from top to bottom.
const int   CAL_MAX         = 10;
const int   CAL_T_SHORT_MS  = 2000;   // the two cal run lengths used by 'meas'
const int   CAL_T_LONG_MS   = 4000;
const float DEF_PWM[5]      = { 100, 120, 140, 160, 175 };
const float DEF_SPEED[5]    = {   8,  13,  18,  23,  27 };   // PLACEHOLDERS
float calPwm[CAL_MAX]       = { 100, 120, 140, 160, 175 };
float calSpeed[CAL_MAX]     = {   8,  13,  18,  23,  27 };
int   nCal                  = 5;
bool  tableCustom           = false;  // false = still the placeholder table

// ======================= PWM (works on Arduino-ESP32 core 2.x and 3.x) =======================
const int PWM_FREQ = 5000;
const int PWM_BITS = 8;   // duty 0-255

#if defined(ESP_ARDUINO_VERSION_MAJOR) && ESP_ARDUINO_VERSION_MAJOR >= 3
  void pwmSetup() {
    ledcAttach(PIN_ENA, PWM_FREQ, PWM_BITS);
    ledcAttach(PIN_ENB, PWM_FREQ, PWM_BITS);
  }
  void pwmWrite(int pin, int duty) { ledcWrite(pin, duty); }
#else
  const int CH_A = 0;
  const int CH_B = 1;
  void pwmSetup() {
    ledcSetup(CH_A, PWM_FREQ, PWM_BITS);
    ledcSetup(CH_B, PWM_FREQ, PWM_BITS);
    ledcAttachPin(PIN_ENA, CH_A);
    ledcAttachPin(PIN_ENB, CH_B);
  }
  void pwmWrite(int pin, int duty) { ledcWrite(pin == PIN_ENA ? CH_A : CH_B, duty); }
#endif

Preferences prefs;

// ======================= MPU-9250 (library-free I2C) =======================
bool mpuWrite(uint8_t reg, uint8_t val) {
  Wire.beginTransmission(MPU_ADDR);
  Wire.write(reg);
  Wire.write(val);
  return Wire.endTransmission() == 0;
}

int mpuReadReg(uint8_t reg) {
  Wire.beginTransmission(MPU_ADDR);
  Wire.write(reg);
  if (Wire.endTransmission(false) != 0) return -1;
  if (Wire.requestFrom((int)MPU_ADDR, 1, 1) != 1) return -1;
  return Wire.read();
}

bool mpuReadGyroZ(int16_t &gz) {
  Wire.beginTransmission(MPU_ADDR);
  Wire.write((uint8_t)0x47);              // GYRO_ZOUT_H
  if (Wire.endTransmission(false) != 0) return false;
  if (Wire.requestFrom((int)MPU_ADDR, 2, 1) != 2) return false;
  int hi = Wire.read();
  int lo = Wire.read();
  gz = (int16_t)((hi << 8) | lo);
  return true;
}

bool mpuInit() {
  Wire.begin(PIN_SDA, PIN_SCL);
  Wire.setClock(400000);
  int who = mpuReadReg(0x75);             // WHO_AM_I (0x71 = 9250, 0x70 = 6500, 0x73 = 9255)
  if (who < 0 || who == 0x00 || who == 0xFF) return false;
  Serial.printf("MPU WHO_AM_I = 0x%02X\n", who);
  mpuWrite(0x6B, 0x80);                   // reset
  delay(100);
  mpuWrite(0x6B, 0x01);                   // wake, clock = gyro PLL
  delay(50);
  mpuWrite(0x1A, 0x03);                   // low-pass filter ~41 Hz
  mpuWrite(0x19, 0x00);                   // sample rate divider
  mpuWrite(0x1B, 0x00);                   // gyro range +/-250 dps
  delay(20);
  return true;
}

void resetHeading() {
  heading = 0.0f;
  rateDps = 0.0f;
  lastGyroUs = micros();
}

void gyroCalibrate() {
  if (!gyroOk) return;
  digitalWrite(PIN_LED, HIGH);
  delay(300);
  double sum = 0;
  int n = 0;
  for (int i = 0; i < 400; i++) {
    int16_t raw;
    if (mpuReadGyroZ(raw)) { sum += raw; n++; }
    delay(3);
  }
  if (n > 0) gyroBias = (float)(sum / n);
  resetHeading();
  digitalWrite(PIN_LED, LOW);
  float biasDps = gyroBias / GYRO_LSB_PER_DPS;
  Serial.printf("Gyro zeroed: bias %.1f counts (%.3f deg/s)\n", gyroBias, biasDps);
  if (fabsf(biasDps) > 2.0f) Serial.println("WARNING: large bias - was the car moving? Run 'zero' again with the car still.");
}

void updateHeading() {
  if (!gyroOk) return;
  int16_t raw;
  if (!mpuReadGyroZ(raw)) return;
  uint32_t now = micros();
  float dt = (now - lastGyroUs) * 1e-6f;
  lastGyroUs = now;
  if (dt <= 0.0f || dt > 0.1f) return;    // skip glitches / long gaps
  rateDps = GYRO_SIGN * ((raw - gyroBias) / GYRO_LSB_PER_DPS);
  heading += rateDps * dt;
}

// ======================= MOTOR HELPERS =======================
int  baseDuty   = 0;       // requested duty before trims / correction
int  runDir     = 1;       // +1 forward, -1 reverse
bool holdActive = false;   // true while a leg is running

// If the vehicle drives the wrong way, swap the two wires of BOTH motors
// (or swap the true/false logic below).
void setDirection(Dir d) {
  bool f = (d == DIR_FWD);
  digitalWrite(PIN_IN1, f ? HIGH : LOW);
  digitalWrite(PIN_IN2, f ? LOW : HIGH);
  digitalWrite(PIN_IN3, f ? HIGH : LOW);
  digitalWrite(PIN_IN4, f ? LOW : HIGH);
}

void applyDrive() {
  float corr = 0.0f;
  if (useGyro && gyroOk && holdActive) {
    corr = KP * heading + KD * rateDps;      // > 0 : car has turned left
    corr = constrain(corr, -MAX_CORR, MAX_CORR);
    corr *= runDir;                           // reverse flips the steering effect
  }
  // corr > 0 : need a right turn (going forward) -> slow the right wheel
  // corr < 0 : need a left turn  (going forward) -> slow the left wheel
  float left = baseDuty, right = baseDuty;
  if (corr > 0) right -= corr; else left += corr;

  float a = MOTOR_A_IS_LEFT ? left : right;
  float b = MOTOR_A_IS_LEFT ? right : left;
  int da = constrain((int)(a * MOTOR_A_TRIM + 0.5f), 0, MAX_DUTY);
  int db = constrain((int)(b * MOTOR_B_TRIM + 0.5f), 0, MAX_DUTY);
  pwmWrite(PIN_ENA, da);
  pwmWrite(PIN_ENB, db);
}

void drive(int duty) {
  baseDuty = constrain(duty, 0, MAX_DUTY);
  applyDrive();
}

void coast() {
  baseDuty = 0;
  pwmWrite(PIN_ENA, 0);
  pwmWrite(PIN_ENB, 0);
}

// Wait while still tracking heading (and steering if a leg is active)
void servicePause(uint32_t ms) {
  uint32_t t0 = millis();
  while (millis() - t0 < ms) {
    updateHeading();
    if (holdActive) applyDrive();
    delay(2);
  }
}

// TB6612 short brake: both inputs HIGH shorts the motor terminals
void brake() {
  holdActive = false;
  digitalWrite(PIN_IN1, HIGH);
  digitalWrite(PIN_IN2, HIGH);
  digitalWrite(PIN_IN3, HIGH);
  digitalWrite(PIN_IN4, HIGH);
  pwmWrite(PIN_ENA, 255);
  pwmWrite(PIN_ENB, 255);
  servicePause(BRAKE_MS);
  coast();
}

void rampPWM(int fromDuty, int toDuty, int ms) {
  int steps = max(1, ms / 10);
  for (int i = 1; i <= steps; i++) {
    int duty = fromDuty + (int)((long)(toDuty - fromDuty) * i / steps);
    drive(duty);
    servicePause(10);
  }
}

// ======================= SPEED TABLE =======================
float minSpeed() { return nCal > 0 ? calSpeed[0] : 0.0f; }
float maxSpeed() { return nCal > 0 ? calSpeed[nCal - 1] : 0.0f; }

int speedToPWM(float v) {
  if (nCal <= 0) return MIN_PWM;
  if (nCal == 1 || v <= calSpeed[0]) return (int)calPwm[0];
  for (int i = 1; i < nCal; i++) {
    if (v <= calSpeed[i]) {
      float f = (v - calSpeed[i - 1]) / (calSpeed[i] - calSpeed[i - 1]);
      return (int)(calPwm[i - 1] + f * (calPwm[i] - calPwm[i - 1]) + 0.5f);
    }
  }
  return (int)calPwm[nCal - 1];
}

void printTable() {
  Serial.printf("---- SPEED TABLE (%s, %d rows) ----\n", tableCustom ? "measured" : "PLACEHOLDER", nCal);
  for (int i = 0; i < nCal; i++) {
    Serial.printf("PWM %3d -> %.1f cm/s\n", (int)(calPwm[i] + 0.5f), calSpeed[i]);
  }
  Serial.println("--------------");
}

void saveTable() {
  prefs.begin("ev", false);
  prefs.putInt("tcust", tableCustom ? 1 : 0);
  prefs.putInt("ncal", nCal);
  prefs.putBytes("cpwm", calPwm, sizeof(calPwm));
  prefs.putBytes("cspd", calSpeed, sizeof(calSpeed));
  prefs.end();
}

void resetTableToDefaults() {
  nCal = 5;
  for (int i = 0; i < CAL_MAX; i++) {
    calPwm[i]   = (i < 5) ? DEF_PWM[i]   : 0.0f;
    calSpeed[i] = (i < 5) ? DEF_SPEED[i] : 0.0f;
  }
  tableCustom = false;
}

bool tableOrderOk(const float *p, const float *s, int n) {
  for (int i = 1; i < n; i++) {
    if (p[i] <= p[i - 1] || s[i] <= s[i - 1]) return false;
  }
  return true;
}

// Add a row, or update the row with the same PWM
bool tableSet(float pwm, float speed) {
  pwm = roundf(pwm);
  if (pwm < MIN_PWM || pwm > MAX_DUTY) {
    Serial.printf("PWM must be between %d and %d.\n", MIN_PWM, MAX_DUTY);
    return false;
  }
  if (speed <= 0.0f) {
    Serial.println("Speed must be above 0 cm/s - check the distances you entered.");
    return false;
  }
  if (!tableCustom) {
    nCal = 0;
    tableCustom = true;
    Serial.println("Placeholder rows cleared - building your own table.");
  }

  float p[CAL_MAX + 1], s[CAL_MAX + 1];
  int n = nCal;
  for (int i = 0; i < n; i++) { p[i] = calPwm[i]; s[i] = calSpeed[i]; }

  int idx = -1;
  for (int i = 0; i < n; i++) if (fabsf(p[i] - pwm) < 0.5f) idx = i;

  if (idx >= 0) {
    s[idx] = speed;
  } else {
    if (n >= CAL_MAX) {
      Serial.println("Table is full (10 rows). Remove one with 'del <pwm>' first.");
      return false;
    }
    int pos = 0;
    while (pos < n && p[pos] < pwm) pos++;
    for (int i = n; i > pos; i--) { p[i] = p[i - 1]; s[i] = s[i - 1]; }
    p[pos] = pwm;
    s[pos] = speed;
    n++;
  }

  if (!tableOrderOk(p, s, n)) {
    Serial.println("Rejected: speed must rise as PWM rises. Recheck this measurement or its neighbours (see 'tab').");
    return false;
  }

  for (int i = 0; i < n; i++) { calPwm[i] = p[i]; calSpeed[i] = s[i]; }
  nCal = n;
  saveTable();
  printTable();
  return true;
}

bool tableDel(float pwm) {
  pwm = roundf(pwm);
  int idx = -1;
  for (int i = 0; i < nCal; i++) if (fabsf(calPwm[i] - pwm) < 0.5f) idx = i;
  if (idx < 0) {
    Serial.printf("No row with PWM %d.\n", (int)pwm);
    return false;
  }
  for (int i = idx; i < nCal - 1; i++) { calPwm[i] = calPwm[i + 1]; calSpeed[i] = calSpeed[i + 1]; }
  nCal--;
  tableCustom = true;
  saveTable();
  printTable();
  return true;
}

// ======================= PLAN =======================
Plan makePlan() {
  Plan p;
  p.fwd = targetDistCm + bottleLineCm + LINE_OVERSHOOT_CM - VEHICLE_LEN_CM;
  p.rev = p.fwd - targetDistCm;
  float fixedS = (START_DELAY_MS + TURN_PAUSE_MS + 2 * BRAKE_MS + 2 * RAMP_MS) / 1000.0f;
  float moveS  = targetTimeS - fixedS;
  p.ok = (p.fwd > 0 && p.rev > 0 && moveS > 0);
  p.v  = (moveS > 0) ? (p.fwd + p.rev) / moveS : 0;
  return p;
}

void printPlan() {
  Plan p = makePlan();
  Serial.println();
  Serial.println("---- PLAN ----");
  Serial.printf("Target %.1f cm | Time %.2f s | Bottle line %.1f cm\n", targetDistCm, targetTimeS, bottleLineCm);
  Serial.printf("Forward %.1f cm | Reverse %.1f cm | Needed speed %.1f cm/s\n", p.fwd, p.rev, p.v);
  Serial.printf("Scales: fwd %.3f rev %.3f\n", fwdScale, revScale);
  Serial.printf("Gyro: %s | KP %.2f\n", (gyroOk && useGyro) ? "ON" : (gyroOk ? "OFF (disabled)" : "NOT FOUND"), KP);
  if (!tableCustom) Serial.println("WARNING: speed table is the PLACEHOLDER default - measure your car and use 'meas' or 'add'.");
  if (nCal < 2) Serial.println("WARNING: speed table needs at least 2 rows.");
  if (!p.ok) Serial.println("WARNING: plan invalid (check vehicle length, distances, time).");
  if (nCal >= 2 && p.v > maxSpeed()) Serial.printf("WARNING: needed speed above calibrated max (%.1f cm/s). Will run slower than target time.\n", maxSpeed());
  if (nCal >= 2 && p.v < minSpeed() && p.ok) Serial.printf("WARNING: needed speed below calibrated min (%.1f cm/s). Will run faster than target time.\n", minSpeed());
  Serial.println("--------------");
}

// ======================= RUN =======================
void runLeg(Dir d, float distCm, float v, float scale) {
  int cruise = speedToPWM(v);
  float cruiseS = (distCm / v) - (RAMP_MS / 1000.0f);
  if (cruiseS < 0) cruiseS = 0;
  cruiseS *= scale;

  setDirection(d);
  runDir = (d == DIR_FWD) ? 1 : -1;
  holdActive = true;
  rampPWM(MIN_PWM, cruise, RAMP_MS);
  drive(cruise);
  servicePause((uint32_t)(cruiseS * 1000.0f));
  rampPWM(cruise, MIN_PWM, RAMP_MS);
  brake();
}

void runSequence() {
  Plan p = makePlan();
  if (!p.ok) {
    Serial.println("Plan invalid - not running.");
    return;
  }
  if (nCal < 2) {
    Serial.println("Speed table needs at least 2 rows - use 'meas' or 'add'. Not running.");
    return;
  }
  float v = constrain(p.v, minSpeed(), maxSpeed());

  digitalWrite(PIN_LED, HIGH);
  uint32_t t0 = millis();
  delay(START_DELAY_MS);
  resetHeading();                      // heading 0 = the direction the car is pointing now

  runLeg(DIR_FWD, p.fwd, v, fwdScale);
  float hFwd = heading;
  servicePause(TURN_PAUSE_MS);
  runLeg(DIR_REV, p.rev, v, revScale);
  servicePause(300);
  float hEnd = heading;

  uint32_t elapsed = millis() - t0;
  digitalWrite(PIN_LED, LOW);

  // Do not look at this until the vehicle has been retrieved.
  Serial.printf("Run done. Elapsed %.2f s (target %.2f s). Heading: after fwd %.2f deg, end %.2f deg\n",
                elapsed / 1000.0f, targetTimeS, hFwd, hEnd);
}

void calibrationRun(int pwm, int ms) {
  pwm = constrain(pwm, MIN_PWM, MAX_DUTY);
  ms  = constrain(ms, 500, 8000);
  Serial.printf("CAL: PWM %d for %d ms. Measure how far it travelled.\n", pwm, ms);
  setDirection(DIR_FWD);
  runDir = 1;
  resetHeading();
  holdActive = true;
  rampPWM(MIN_PWM, pwm, 200);
  drive(pwm);
  servicePause(ms);
  brake();
  servicePause(300);
  Serial.printf("CAL done. End heading %.2f deg\n", heading);
  Serial.printf("Speed table: run this PWM at %d ms (d1) and %d ms (d2), then type: meas %d <d1> <d2>\n",
                CAL_T_SHORT_MS, CAL_T_LONG_MS, pwm);
}

void gyroTest() {
  if (!gyroOk) { Serial.println("Gyro not found."); return; }
  Serial.println("Gyro test (6 s): turn the car LEFT by hand - heading should go POSITIVE.");
  resetHeading();
  uint32_t t0 = millis();
  uint32_t lastPrint = 0;
  while (millis() - t0 < 6000) {
    updateHeading();
    if (millis() - lastPrint >= 200) {
      lastPrint = millis();
      Serial.printf("heading %.2f deg  rate %.2f deg/s\n", heading, rateDps);
    }
    delay(2);
  }
  Serial.println("Gyro test done.");
}

// ======================= SETTINGS STORAGE =======================
void saveParams() {
  prefs.begin("ev", false);
  prefs.putFloat("dist", targetDistCm);
  prefs.putFloat("time", targetTimeS);
  prefs.putFloat("line", bottleLineCm);
  prefs.putFloat("fs", fwdScale);
  prefs.putFloat("rs", revScale);
  prefs.putFloat("kp", KP);
  prefs.putFloat("ug", useGyro ? 1.0f : 0.0f);
  prefs.end();
}

// Remember which EVENT SETTINGS this sketch was last uploaded with, so a new
// upload with different numbers is detected and used.
void saveEventStamp() {
  prefs.begin("ev", false);
  prefs.putFloat("sd", EVENT_DIST_CM);
  prefs.putFloat("st", EVENT_TIME_S);
  prefs.putFloat("sb", EVENT_BOTTLE_CM);
  prefs.end();
}

void loadParams() {
  prefs.begin("ev", true);
  float sd = prefs.getFloat("sd", -1.0f);
  float st = prefs.getFloat("st", -1.0f);
  float sb = prefs.getFloat("sb", -1.0f);
  eventFromSketch = (sd != EVENT_DIST_CM || st != EVENT_TIME_S || sb != EVENT_BOTTLE_CM);
  if (eventFromSketch) {
    targetDistCm = EVENT_DIST_CM;
    targetTimeS  = EVENT_TIME_S;
    bottleLineCm = EVENT_BOTTLE_CM;
  } else {
    targetDistCm = prefs.getFloat("dist", EVENT_DIST_CM);
    targetTimeS  = prefs.getFloat("time", EVENT_TIME_S);
    bottleLineCm = prefs.getFloat("line", EVENT_BOTTLE_CM);
  }
  fwdScale     = prefs.getFloat("fs", fwdScale);
  revScale     = prefs.getFloat("rs", revScale);
  KP           = prefs.getFloat("kp", KP);
  useGyro      = prefs.getFloat("ug", 1.0f) > 0.5f;

  if (prefs.getInt("tcust", 0) == 1) {
    int n = prefs.getInt("ncal", 0);
    if (n >= 0 && n <= CAL_MAX &&
        prefs.getBytesLength("cpwm") == sizeof(calPwm) &&
        prefs.getBytesLength("cspd") == sizeof(calSpeed)) {
      prefs.getBytes("cpwm", calPwm, sizeof(calPwm));
      prefs.getBytes("cspd", calSpeed, sizeof(calSpeed));
      nCal = n;
      tableCustom = true;
    }
  }
  prefs.end();

  if (eventFromSketch) {      // new numbers uploaded: stamp them and save
    saveEventStamp();
    saveParams();
  }
}

void printHelp() {
  Serial.println("Commands: d <cm> | t <s> | b <cm> | fs <x> | rs <x> | kp <x> | gyro <0|1> | zero | gtest | p | cal <pwm> <ms> | evt");
  Serial.println("Table:    meas <pwm> <d1> <d2> | add <pwm> <cm/s> | del <pwm> | tab | tabrst | help");
}

void handleSerial() {
  if (!Serial.available()) return;
  String line = Serial.readStringUntil('\n');
  line.trim();
  if (line.length() == 0) return;

  char cmd[8] = {0};
  float a = 0, b = 0, c = 0;
  int n = sscanf(line.c_str(), "%7s %f %f %f", cmd, &a, &b, &c);
  if (n < 1) return;

  if (strcmp(cmd, "d") == 0 && n >= 2)         { targetDistCm = a; saveParams(); printPlan(); }
  else if (strcmp(cmd, "t") == 0 && n >= 2)    { targetTimeS  = a; saveParams(); printPlan(); }
  else if (strcmp(cmd, "b") == 0 && n >= 2)    { bottleLineCm = a; saveParams(); printPlan(); }
  else if (strcmp(cmd, "fs") == 0 && n >= 2)   { fwdScale = a; saveParams(); printPlan(); }
  else if (strcmp(cmd, "rs") == 0 && n >= 2)   { revScale = a; saveParams(); printPlan(); }
  else if (strcmp(cmd, "kp") == 0 && n >= 2)   { KP = a; saveParams(); printPlan(); }
  else if (strcmp(cmd, "gyro") == 0 && n >= 2) { useGyro = (a != 0); saveParams(); printPlan(); }
  else if (strcmp(cmd, "zero") == 0)           { gyroCalibrate(); }
  else if (strcmp(cmd, "gtest") == 0)          { gyroTest(); }
  else if (strcmp(cmd, "evt") == 0)            { targetDistCm = EVENT_DIST_CM; targetTimeS = EVENT_TIME_S; bottleLineCm = EVENT_BOTTLE_CM; saveParams(); printPlan(); }
  else if (strcmp(cmd, "p") == 0)              { printPlan(); }
  else if (strcmp(cmd, "cal") == 0 && n >= 3)  { calibrationRun((int)a, (int)b); }
  else if (strcmp(cmd, "meas") == 0 && n >= 4) {
    float dt = (CAL_T_LONG_MS - CAL_T_SHORT_MS) / 1000.0f;
    float speed = (c - b) / dt;             // a = pwm, b = d1 (short run), c = d2 (long run)
    Serial.printf("PWM %d: (%.1f - %.1f) / %.1f s = %.2f cm/s\n", (int)a, c, b, dt, speed);
    tableSet(a, speed);
  }
  else if (strcmp(cmd, "add") == 0 && n >= 3)  { tableSet(a, b); }
  else if (strcmp(cmd, "del") == 0 && n >= 2)  { tableDel(a); }
  else if (strcmp(cmd, "tab") == 0)            { printTable(); }
  else if (strcmp(cmd, "tabrst") == 0)         { resetTableToDefaults(); saveTable(); printTable(); }
  else                                         { printHelp(); }
}

// ======================= ARDUINO =======================
bool startPressed() {
  if (digitalRead(PIN_START) == LOW) {
    delay(30);
    if (digitalRead(PIN_START) == LOW) return true;
  }
  return false;
}

void setup() {
  // Motor pins low first so nothing twitches at boot
  pinMode(PIN_IN1, OUTPUT); digitalWrite(PIN_IN1, LOW);
  pinMode(PIN_IN2, OUTPUT); digitalWrite(PIN_IN2, LOW);
  pinMode(PIN_IN3, OUTPUT); digitalWrite(PIN_IN3, LOW);
  pinMode(PIN_IN4, OUTPUT); digitalWrite(PIN_IN4, LOW);
  pinMode(PIN_LED, OUTPUT); digitalWrite(PIN_LED, LOW);
  pinMode(PIN_START, INPUT_PULLUP);

  pwmSetup();
  coast();

  Serial.begin(115200);
  delay(300);
  loadParams();
  Serial.println("Electric Vehicle controller ready.");
  Serial.println(eventFromSketch
    ? "Event values: loaded from EVENT SETTINGS at the top of the sketch."
    : "Event values: restored from saved settings (type 'evt' to reload the sketch's values).");

  gyroOk = mpuInit();
  if (gyroOk) {
    Serial.println("MPU found. Keep the car STILL - zeroing gyro...");
    gyroCalibrate();
  } else {
    Serial.println("MPU NOT found (check wiring / address). Running without heading hold.");
  }

  printHelp();
  printPlan();
}

void loop() {
  handleSerial();

  // Slow heartbeat blink while armed and waiting
  digitalWrite(PIN_LED, (millis() / 500) % 2);

  if (startPressed()) {
    runSequence();
    // wait for button release and give time to retrieve the vehicle
    while (digitalRead(PIN_START) == LOW) delay(10);
    delay(1500);
  }
}