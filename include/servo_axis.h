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
// ax2-1(-)=80, ax2-2(+)=130; ax3-2(+)=140; ax4-2(+)=90
static const int SERVO_AXIS_MIN_DEG[4] = {60, 80, 60, 60};
static const int SERVO_AXIS_MAX_DEG[4] = {120, 130, 140, 90};
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

static const float SERVO_SPEED_DEG_S = 50.0f;  // legacy default (= level 5)
static const uint32_t SERVO_TICK_MS = 10;
static const uint32_t SERVO_LOG_MS = 200;

// Runtime tour/jog speed level 1..9 (default 5)
static const int SPEED_LEVEL_MAX = 9;
static int gSpeedLevel = 5;
static const float SPEED_DEG_S_BY_LEVEL[10] = {
    0.f,
    12.f, 20.f, 30.f, 40.f, 50.f,  // 1..5
    70.f, 90.f, 110.f, 140.f       // 6..9
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
  void begin(int pin, const char* label, int minDeg, int maxDeg) {
    _pin = pin;
    _label = label;
    _minDeg = minDeg;
    _maxDeg = maxDeg;
    if (_minDeg > _maxDeg) {
      int t = _minDeg;
      _minDeg = _maxDeg;
      _maxDeg = t;
    }
    _servo.setPeriodHertz(50);
    int ch = _servo.attach(_pin, SERVO_US_MIN, SERVO_US_MAX);
    _us = degToUs(SERVO_CENTER_DEG);
    _dir = 0;
    _targetUs = -1;
    _lastTickMs = millis();
    _lastLogMs = 0;
    if (ch == 0 && !_servo.attached()) {
      // some versions return channel index; rely on attached()
    }
    if (!_servo.attached()) {
      Serial.printf("[SERVO] %s gpio=%d ATTACH FAIL\n", _label, _pin);
    } else {
      _servo.writeMicroseconds(_us);
      Serial.printf("[SERVO] %s gpio=%d OK center=%d range=%d..%d ch=%d\n",
                    _label, _pin, SERVO_CENTER_DEG, _minDeg, _maxDeg, ch);
    }
  }

  int angle() const { return usToDeg(_us); }
  int pin() const { return _pin; }
  int minDeg() const { return _minDeg; }
  int maxDeg() const { return _maxDeg; }

  void setDirection(int dir) {
    if (dir < -1) dir = -1;
    if (dir > 1) dir = 1;
    if (dir != 0) _targetUs = -1;  // jog cancels goto
    _dir = dir;
  }

  void setTargetDeg(int deg) {
    if (deg < _minDeg) deg = _minDeg;
    if (deg > _maxDeg) deg = _maxDeg;
    _dir = 0;
    _targetUs = degToUs(deg);
  }

  void clearTarget() { _targetUs = -1; }

  void snapToTarget() {
    if (_targetUs < 0) return;
    _us = _targetUs;
    _targetUs = -1;
    _dir = 0;
    _servo.writeMicroseconds(_us);
  }

  bool busy() const { return _dir != 0 || _targetUs >= 0; }

  void service() {
    uint32_t now = millis();
    uint32_t dt = now - _lastTickMs;
    if (dt < SERVO_TICK_MS) return;
    _lastTickMs = now;

    const float usPerDeg = (float)(SERVO_US_MAX - SERVO_US_MIN) / 180.0f;
    int deltaUs = (int)(getServoSpeedDegS() * usPerDeg * (float)dt / 1000.0f + 0.5f);
    if (deltaUs < 1) deltaUs = 1;

    int minUs = degToUs(_minDeg);
    int maxUs = degToUs(_maxDeg);
    int next = _us;

    if (_targetUs >= 0) {
      if (_us < _targetUs) {
        next = _us + deltaUs;
        if (next > _targetUs) next = _targetUs;
      } else if (_us > _targetUs) {
        next = _us - deltaUs;
        if (next < _targetUs) next = _targetUs;
      }
      if (next == _targetUs) {
        _targetUs = -1;
        if (now - _lastLogMs >= SERVO_LOG_MS) {
          _lastLogMs = now;
          Serial.printf("[SERVO] %s gpio=%d deg=%d (arrived)\n", _label, _pin, usToDeg(next));
        }
      }
    } else if (_dir != 0) {
      next = _us + _dir * deltaUs;
      if (next < minUs) next = minUs;
      if (next > maxUs) next = maxUs;
      if (next == _us && (now - _lastLogMs >= SERVO_LOG_MS)) {
        _lastLogMs = now;
        Serial.printf("[SERVO] %s gpio=%d deg=%d (limit)\n", _label, _pin, angle());
      }
    } else {
      return;
    }

    if (next != _us) {
      _us = next;
      _servo.writeMicroseconds(_us);
      if (now - _lastLogMs >= SERVO_LOG_MS) {
        _lastLogMs = now;
        logAngle();
      }
    }
  }

  void logAngle() const {
    Serial.printf("[SERVO] %s gpio=%d deg=%d us=%d\n", _label, _pin, angle(), _us);
  }

 private:
  static int degToUs(int deg) {
    return map(deg, 0, 180, SERVO_US_MIN, SERVO_US_MAX);
  }
  static int usToDeg(int us) {
    return map(us, SERVO_US_MIN, SERVO_US_MAX, 0, 180);
  }

  Servo _servo;
  int _pin = -1;
  const char* _label = "?";
  int _minDeg = SERVO_MIN_DEG;
  int _maxDeg = SERVO_MAX_DEG;
  int _us = 0;
  int _dir = 0;
  int _targetUs = -1;
  uint32_t _lastTickMs = 0;
  uint32_t _lastLogMs = 0;
};

class ServoBank {
 public:
  void begin() {
    ESP32PWM::allocateTimer(0);
    ESP32PWM::allocateTimer(1);
    ESP32PWM::allocateTimer(2);
    ESP32PWM::allocateTimer(3);
    Serial.println("[SERVO] WARNING: use external 5-6V; common GND with ESP");
#ifdef SERVO_SINGLE_PIN
    Serial.printf("[SERVO] SINGLE-TEST mode: ALL keyboard axes -> GPIO%d @ level%d\n",
                  SERVO_SINGLE_PIN, getSpeedLevel());
    _axes[0].begin(SERVO_SINGLE_PIN, "test27",
                   SERVO_AXIS_MIN_DEG[0], SERVO_AXIS_MAX_DEG[0]);
    _count = 1;
#else
    Serial.printf("[SERVO] 4-axis pins=%d,%d,%d,%d speedLevel=%d (%.0fdeg/s)\n",
                  SERVO_PINS[0], SERVO_PINS[1], SERVO_PINS[2], SERVO_PINS[3],
                  getSpeedLevel(), (double)getServoSpeedDegS());
    for (int i = 0; i < 4; i++) {
      _axes[i].begin(SERVO_PINS[i], SERVO_LABELS[i],
                     SERVO_AXIS_MIN_DEG[i], SERVO_AXIS_MAX_DEG[i]);
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

  // Stop jog only — keep in-flight goto/recall targets.
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
    stopAll();
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
