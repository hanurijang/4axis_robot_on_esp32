#pragma once

#include <Arduino.h>
#include <ESP32Servo.h>

// 4x MG996R: ax1..ax4
// Avoid GPIO21 (ESP32 default I2C SDA; broke after CSG Wire).
// GPIO18 also failed PWM on this board with a known-good servo → use 23.
static const int SERVO_PINS[4] = {13, 14, 23, 27};
static const char* SERVO_LABELS[4] = {"ax1", "ax2", "ax3", "ax4"};

// Default / fallback soft limits (ax1, ax4)
static const int SERVO_MIN_DEG = 60;
static const int SERVO_MAX_DEG = 120;
// Per-axis jog/goto limits: [ax1, ax2, ax3, ax4]
// ax1 85..130; ax2 80..130; ax3 30..170; ax4 60..95
static const int SERVO_AXIS_MIN_DEG[4] = {85, 80, 30, 60};
static const int SERVO_AXIS_MAX_DEG[4] = {130, 130, 170, 95};
static const int SERVO_CENTER_DEG = 90;
static const int SERVO_US_MIN = 500;
static const int SERVO_US_MAX = 2500;

inline int servoAxisMinDeg(int axis) {
  if (axis < 0 || axis > 3) return SERVO_MIN_DEG;
  return SERVO_AXIS_MIN_DEG[axis];
}
inline int servoAxisMaxDeg(int axis) {
  if (axis < 0 || axis > 3) return SERVO_MAX_DEG;
  return SERVO_AXIS_MAX_DEG[axis];
}
inline int clampServoDeg(int axis, int deg) {
  int lo = servoAxisMinDeg(axis);
  int hi = servoAxisMaxDeg(axis);
  if (deg < lo) return lo;
  if (deg > hi) return hi;
  return deg;
}

static const uint32_t SERVO_TICK_MS = 10;
static const uint32_t SERVO_LOG_MS = 250;
// Soft keep-alive: LEDC/PWM can drop while idle → limp until next write.
static const uint32_t SERVO_HOLD_MS = 1500;
// Loop can stall (UART/BLE); large dt must not become a jump.
static const uint32_t SERVO_DT_CLAMP_MS = 40;
static const float SERVO_ACCEL_DEG_S2 = 90.0f;   // accel/decel for smooth start/stop
static const float SERVO_MAX_STEP_DEG = 2.2f;    // allow level9 (~195°/s @ 10ms tick)
static const float SERVO_GOTO_ARRIVE_DEG = 1.5f; // declare arrived (avoid endless crawl)
static const float SERVO_GOTO_MIN_DEG_S = 18.0f; // keep moving until near target
static const float SERVO_GOTO_SLOW_DEG = 6.0f;   // start taper inside this window

// Runtime tour/jog speed level 1..9 (default 5)
static const int SPEED_LEVEL_MAX = 9;
static int gSpeedLevel = 5;
static const float SPEED_DEG_S_BY_LEVEL[10] = {
    0.f,
    8.f, 14.f, 22.f, 30.f, 40.f,   // 1..5
    55.f, 70.f, 85.f, 195.f       // 6..9 (level9 = 130 * 1.5)
};

inline int getSpeedLevel() { return gSpeedLevel; }

inline float getServoSpeedDegS() {
  if (gSpeedLevel < 1) return SPEED_DEG_S_BY_LEVEL[1];
  if (gSpeedLevel > SPEED_LEVEL_MAX) return SPEED_DEG_S_BY_LEVEL[SPEED_LEVEL_MAX];
  return SPEED_DEG_S_BY_LEVEL[gSpeedLevel];
}

inline bool setSpeedLevel(int level) {
  if (level < 1) level = 1;
  if (level > SPEED_LEVEL_MAX) level = SPEED_LEVEL_MAX;
  if (level == gSpeedLevel) return false;
  gSpeedLevel = level;
  return true;
}

inline bool bumpSpeedLevel(int delta) {
  return setSpeedLevel(gSpeedLevel + delta);
}

class ServoAxis {
 public:
  void begin(int pin, const char* label, int minDeg, int maxDeg, int initialDeg) {
    _pin = pin;
    _label = label;
    _minDeg = minDeg;
    _maxDeg = maxDeg;
    if (_minDeg > _maxDeg) {
      int t = _minDeg;
      _minDeg = _maxDeg;
      _maxDeg = t;
    }
    if (initialDeg < _minDeg) initialDeg = _minDeg;
    if (initialDeg > _maxDeg) initialDeg = _maxDeg;
    _servo.setPeriodHertz(50);
    int ch = _servo.attach(_pin, SERVO_US_MIN, SERVO_US_MAX);
    // First PWM is always an absolute seek; start at last-known pose (not forced HOME)
    // so a later slew to HOME can actually be gradual.
    _us = degToUs(initialDeg);
    _cmdDir = 0;
    _velDegS = 0.f;
    _targetUs = -1;
    _lastTickMs = millis();
    _lastLogMs = 0;
    _lastHoldMs = millis();
    _limitLogged = false;
    if (!_servo.attached()) {
      Serial.printf("[SERVO] %s gpio=%d ATTACH FAIL\n", _label, _pin);
    } else {
      _servo.writeMicroseconds(_us);
      Serial.printf("[SERVO] %s gpio=%d OK init=%d range=%d..%d ch=%d\n",
                    _label, _pin, initialDeg, _minDeg, _maxDeg, ch);
    }
  }

  int angle() const { return usToDeg(_us); }
  int pin() const { return _pin; }
  int minDeg() const { return _minDeg; }
  int maxDeg() const { return _maxDeg; }

  void setDirection(int dir) {
    if (dir < -1) dir = -1;
    if (dir > 1) dir = 1;
    int minUs = degToUs(_minDeg);
    int maxUs = degToUs(_maxDeg);
    if (dir < 0 && _us <= minUs) dir = 0;
    if (dir > 0 && _us >= maxUs) dir = 0;
    if (dir != 0) _targetUs = -1;  // jog cancels goto
    if (dir != _cmdDir) _limitLogged = false;
    _cmdDir = dir;
    // Keep velocity for soft decelerate when dir→0; reverse: damp quickly
    if (dir != 0 && ((_velDegS > 0 && dir < 0) || (_velDegS < 0 && dir > 0))) {
      _velDegS *= 0.35f;
    }
  }

  void setTargetDeg(int deg) {
    if (deg < _minDeg) deg = _minDeg;
    if (deg > _maxDeg) deg = _maxDeg;
    _cmdDir = 0;
    _targetUs = degToUs(deg);
    _limitLogged = false;
  }

  void clearTarget() { _targetUs = -1; }

  void snapToTarget() {
    if (_targetUs < 0) return;
    _us = _targetUs;
    _targetUs = -1;
    _cmdDir = 0;
    _velDegS = 0.f;
    _servo.writeMicroseconds(_us);
    _lastHoldMs = millis();
  }

  // Ensure PWM is attached and holding current pulse (no detach).
  void reassert() {
    _cmdDir = 0;
    _targetUs = -1;
    _velDegS = 0.f;
    _limitLogged = false;
    if (!_servo.attached()) {
      int ch = _servo.attach(_pin, SERVO_US_MIN, SERVO_US_MAX);
      (void)ch;
      if (!_servo.attached()) {
        Serial.printf("[SERVO] %s gpio=%d REATTACH FAIL\n", _label, _pin);
        return;
      }
    }
    _servo.writeMicroseconds(_us);
    _lastHoldMs = millis();
    Serial.printf("[SERVO] %s gpio=%d reassert deg=%d us=%d\n",
                  _label, _pin, angle(), _us);
  }

  bool busy() const {
    if (_targetUs >= 0) return true;
    if (_cmdDir != 0) return true;
    if (_velDegS > 0.8f || _velDegS < -0.8f) return true;
    return false;
  }

  void service() {
    uint32_t now = millis();
    uint32_t dt = now - _lastTickMs;
    if (dt < SERVO_TICK_MS) return;
    _lastTickMs = now;
    if (dt > SERVO_DT_CLAMP_MS) dt = SERVO_DT_CLAMP_MS;
    const float dtSec = (float)dt / 1000.0f;

    if (!_servo.attached()) {
      int ch = _servo.attach(_pin, SERVO_US_MIN, SERVO_US_MAX);
      (void)ch;
      if (_servo.attached()) {
        _servo.writeMicroseconds(_us);
        _lastHoldMs = now;
        Serial.printf("[SERVO] %s gpio=%d auto-reattach deg=%d\n",
                      _label, _pin, angle());
      }
    }

    const float usPerDeg = (float)(SERVO_US_MAX - SERVO_US_MIN) / 180.0f;
    const float maxSpd = getServoSpeedDegS();
    int minUs = degToUs(_minDeg);
    int maxUs = degToUs(_maxDeg);

    float desiredVel = 0.f;
    bool inGoto = (_targetUs >= 0);

    if (inGoto) {
      float errDeg = (float)(_targetUs - _us) / usPerDeg;
      float absErr = errDeg >= 0.f ? errDeg : -errDeg;
      // Close enough → stop (old P-only crawl made tour dwell start very late)
      if (absErr <= SERVO_GOTO_ARRIVE_DEG) {
        _us = _targetUs;
        _targetUs = -1;
        _velDegS = 0.f;
        _servo.writeMicroseconds(_us);
        _lastHoldMs = now;
        _lastLogMs = now;
        // Always log arrive (not throttled) so tour handoff is visible.
        Serial.printf("[SERVO] %s gpio=%d deg=%d (arrived)\n", _label, _pin, angle());
        return;
      }
      float sign = (errDeg > 0.f) ? 1.f : -1.f;
      float spd = maxSpd;
      if (absErr < SERVO_GOTO_SLOW_DEG) {
        spd = maxSpd * (absErr / SERVO_GOTO_SLOW_DEG);
        if (spd < SERVO_GOTO_MIN_DEG_S) spd = SERVO_GOTO_MIN_DEG_S;
        if (spd > maxSpd) spd = maxSpd;
      }
      desiredVel = sign * spd;
    } else if (_cmdDir != 0) {
      desiredVel = (float)_cmdDir * maxSpd;
    } else {
      desiredVel = 0.f;
    }

    // Accel / decel toward desired velocity
    float dv = SERVO_ACCEL_DEG_S2 * dtSec;
    if (_velDegS < desiredVel) {
      _velDegS += dv;
      if (_velDegS > desiredVel) _velDegS = desiredVel;
    } else if (_velDegS > desiredVel) {
      _velDegS -= dv;
      if (_velDegS < desiredVel) _velDegS = desiredVel;
    }

    float stepDeg = _velDegS * dtSec;
    if (stepDeg > SERVO_MAX_STEP_DEG) stepDeg = SERVO_MAX_STEP_DEG;
    if (stepDeg < -SERVO_MAX_STEP_DEG) stepDeg = -SERVO_MAX_STEP_DEG;

    if (stepDeg < 0.02f && stepDeg > -0.02f && desiredVel == 0.f && !inGoto) {
      _velDegS = 0.f;
      if (_servo.attached() && (now - _lastHoldMs >= SERVO_HOLD_MS)) {
        _lastHoldMs = now;
        _servo.writeMicroseconds(_us);
      }
      return;
    }

    int next = _us + (int)(stepDeg * usPerDeg + (stepDeg >= 0.f ? 0.5f : -0.5f));
    if (next < minUs) next = minUs;
    if (next > maxUs) next = maxUs;

    if (!inGoto && _cmdDir != 0 && next == _us &&
        ((_cmdDir < 0 && _us <= minUs) || (_cmdDir > 0 && _us >= maxUs))) {
      _cmdDir = 0;
      _velDegS = 0.f;
      _servo.writeMicroseconds(_us);
      _lastHoldMs = now;
      if (!_limitLogged) {
        _limitLogged = true;
        _lastLogMs = now;
        Serial.printf("[SERVO] %s gpio=%d deg=%d (limit)\n", _label, _pin, angle());
      }
      return;
    }

    if (next != _us) {
      _us = next;
      _servo.writeMicroseconds(_us);
      _lastHoldMs = now;
      if (now - _lastLogMs >= SERVO_LOG_MS) {
        _lastLogMs = now;
        logAngle();
      }
    } else if (!busy() && _servo.attached() && (now - _lastHoldMs >= SERVO_HOLD_MS)) {
      _lastHoldMs = now;
      _servo.writeMicroseconds(_us);
    }
  }

  void logAngle() const {
    Serial.printf("[SERVO] %s gpio=%d deg=%d us=%d\n", _label, _pin, angle(), _us);
  }

 private:
  static int degToUs(int deg) {
    long num = (long)deg * (SERVO_US_MAX - SERVO_US_MIN);
    return SERVO_US_MIN + (int)((num + 90) / 180);
  }
  static int usToDeg(int us) {
    long num = (long)(us - SERVO_US_MIN) * 180L;
    long den = (long)(SERVO_US_MAX - SERVO_US_MIN);
    return (int)((num + den / 2) / den);
  }

  Servo _servo;
  int _pin = -1;
  const char* _label = "?";
  int _minDeg = SERVO_MIN_DEG;
  int _maxDeg = SERVO_MAX_DEG;
  int _us = 0;
  int _cmdDir = 0;
  float _velDegS = 0.f;
  int _targetUs = -1;
  bool _limitLogged = false;
  uint32_t _lastTickMs = 0;
  uint32_t _lastLogMs = 0;
  uint32_t _lastHoldMs = 0;
};

class ServoBank {
 public:
  void begin(const int initialDeg[4]) {
    ESP32PWM::allocateTimer(0);
    ESP32PWM::allocateTimer(1);
    ESP32PWM::allocateTimer(2);
    ESP32PWM::allocateTimer(3);
    Serial.println("[SERVO] WARNING: use external 5-6V; common GND with ESP");
#ifdef SERVO_SINGLE_PIN
    Serial.printf("[SERVO] SINGLE-TEST mode: ALL keyboard axes -> GPIO%d @ level%d\n",
                  SERVO_SINGLE_PIN, getSpeedLevel());
    int d0 = initialDeg ? initialDeg[0] : SERVO_CENTER_DEG;
    _axes[0].begin(SERVO_SINGLE_PIN, "test27",
                   SERVO_AXIS_MIN_DEG[0], SERVO_AXIS_MAX_DEG[0], d0);
    _count = 1;
#else
    Serial.printf("[SERVO] 4-axis pins=%d,%d,%d,%d speedLevel=%d (%.0fdeg/s)\n",
                  SERVO_PINS[0], SERVO_PINS[1], SERVO_PINS[2], SERVO_PINS[3],
                  getSpeedLevel(), (double)getServoSpeedDegS());
    for (int i = 0; i < 4; i++) {
      int d = initialDeg ? initialDeg[i] : SERVO_CENTER_DEG;
      _axes[i].begin(SERVO_PINS[i], SERVO_LABELS[i],
                     SERVO_AXIS_MIN_DEG[i], SERVO_AXIS_MAX_DEG[i], d);
      Serial.printf("[SERVO] %s limit %d..%d\n",
                    SERVO_LABELS[i], SERVO_AXIS_MIN_DEG[i], SERVO_AXIS_MAX_DEG[i]);
    }
    _count = 4;
#endif
  }

  void setDirection(int axisIndex, int dir) {
#ifdef SERVO_SINGLE_PIN
    (void)axisIndex;
    _axes[0].setDirection(dir);
#else
    if (axisIndex < 0 || axisIndex > 3) return;
    _axes[axisIndex].setDirection(dir);
#endif
  }

  void stopJogOnly() {
    for (int i = 0; i < _count; i++) {
      _axes[i].setDirection(0);
    }
  }

  void stopAll() {
    for (int i = 0; i < _count; i++) {
      _axes[i].setDirection(0);
      _axes[i].clearTarget();
    }
  }

  void getAngles(int out[4]) const {
    for (int i = 0; i < 4; i++) {
      out[i] = (i < _count) ? _axes[i].angle() : SERVO_CENTER_DEG;
    }
  }

  void gotoAngles(const int deg[4]) {
    stopJogOnly();
#ifdef SERVO_SINGLE_PIN
    _axes[0].setTargetDeg(deg[0]);
#else
    for (int i = 0; i < 4; i++) _axes[i].setTargetDeg(deg[i]);
#endif
  }

  void setAnglesImmediate(const int deg[4]) {
    stopAll();
#ifdef SERVO_SINGLE_PIN
    _axes[0].setTargetDeg(deg[0]);
    _axes[0].snapToTarget();
#else
    for (int i = 0; i < 4; i++) {
      _axes[i].setTargetDeg(deg[i]);
      _axes[i].snapToTarget();
    }
#endif
  }

  // Recovery: reassert PWM at current pose, then slew smoothly to target.
  void reinitToAngles(const int deg[4]) {
    stopAll();
#ifdef SERVO_SINGLE_PIN
    _axes[0].reassert();
#else
    for (int i = 0; i < _count; i++) _axes[i].reassert();
#endif
    gotoAngles(deg);
  }

  bool busy() const {
    for (int i = 0; i < _count; i++) {
      if (_axes[i].busy()) return true;
    }
    return false;
  }

  void service() {
    for (int i = 0; i < _count; i++) _axes[i].service();
  }

 private:
  ServoAxis _axes[4];
  int _count = 4;
};
