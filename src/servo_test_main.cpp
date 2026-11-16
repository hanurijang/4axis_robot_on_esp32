/**
 * MG996R individual test on GPIO27
 * Upload:  pio run -e servo_test -t upload
 * Monitor: python3 -m serial.tools.miniterm PORT 115200
 *
 * Plug ONE servo signal into GPIO27 (VCC=ext 5-6V, GND common).
 * Commands: a/d nudge | A/D slew | s stop | c center | w sweep | p status | h help
 */
#include <Arduino.h>
#include <ESP32Servo.h>
#include "soc/soc.h"
#include "soc/rtc_cntl_reg.h"

static const int PIN = 27;
static const int MIN_DEG = 60;
static const int MAX_DEG = 120;
static const int CENTER = 90;
static const int US_MIN = 500;
static const int US_MAX = 2500;
static const float SPEED_DEG_S = 45.0f;

static Servo gServo;
static int gDeg = CENTER;
static int gDir = 0;
static int gSweepPhase = 0;  // 0 idle, 1->max, 2->min, 3->center
static uint32_t gLastMs = 0;
static uint32_t gLastStatusMs = 0;

static int degToUs(int deg) { return map(deg, 0, 180, US_MIN, US_MAX); }

static void writeDeg(int deg) {
  if (deg < MIN_DEG) deg = MIN_DEG;
  if (deg > MAX_DEG) deg = MAX_DEG;
  gDeg = deg;
  gServo.writeMicroseconds(degToUs(gDeg));
}

static void printStatus(const char* why) {
  Serial.printf("[TEST] %s | gpio=%d deg=%d us=%d dir=%d phase=%d attached=%d\n",
                why, PIN, gDeg, degToUs(gDeg), gDir, gSweepPhase,
                gServo.attached() ? 1 : 0);
}

static void printHelp() {
  Serial.println();
  Serial.println("========================================");
  Serial.println("  SERVO INDIVIDUAL TEST  GPIO27");
  Serial.println("========================================");
  Serial.println("1) Power servo from external 5-6V, GND common with ESP");
  Serial.println("2) Connect ONLY signal (orange) to GPIO27");
  Serial.println("3) Type 'w' = sweep, or a/d = step, A/D = hold move");
  Serial.println("4) If deg changes in log but horn does not move -> bad servo/cable/power");
  Serial.println("5) Repeat for each of 4 servos (swap signal wire only)");
  Serial.println("Commands: a d A D s c w p h");
  Serial.println("========================================");
}

void setup() {
  WRITE_PERI_REG(RTC_CNTL_BROWN_OUT_REG, 0);
  Serial.begin(115200);
  delay(800);

  ESP32PWM::allocateTimer(0);
  ESP32PWM::allocateTimer(1);
  ESP32PWM::allocateTimer(2);
  ESP32PWM::allocateTimer(3);

  gServo.setPeriodHertz(50);
  gServo.attach(PIN, US_MIN, US_MAX);
  writeDeg(CENTER);
  gLastMs = millis();

  printHelp();
  printStatus("boot");
  Serial.println(gServo.attached() ? "[TEST] PWM OK" : "[TEST] FAIL: not attached");
}

void loop() {
  while (Serial.available()) {
    char c = (char)Serial.read();
    switch (c) {
      case 'a':
        gDir = 0; gSweepPhase = 0;
        writeDeg(gDeg - 1);
        printStatus("nudge-");
        break;
      case 'd':
        gDir = 0; gSweepPhase = 0;
        writeDeg(gDeg + 1);
        printStatus("nudge+");
        break;
      case 'A':
        gSweepPhase = 0; gDir = -1;
        printStatus("slew-");
        break;
      case 'D':
        gSweepPhase = 0; gDir = +1;
        printStatus("slew+");
        break;
      case 's':
      case ' ':
        gDir = 0; gSweepPhase = 0;
        printStatus("stop");
        break;
      case 'c':
        gDir = 0; gSweepPhase = 0;
        writeDeg(CENTER);
        printStatus("center");
        break;
      case 'w':
        gDir = 0;
        gSweepPhase = 1;
        printStatus("sweep->120");
        break;
      case 'p':
        printStatus("poll");
        break;
      case 'h':
      case '?':
        printHelp();
        break;
      default:
        break;
    }
  }

  uint32_t now = millis();
  if (now - gLastMs >= 10) {
    float dt = (now - gLastMs) / 1000.0f;
    gLastMs = now;
    int step = (int)(SPEED_DEG_S * dt + 0.5f);
    if (step < 1) step = 1;

    if (gSweepPhase == 1) {
      writeDeg(gDeg + step);
      if (gDeg >= MAX_DEG) {
        gSweepPhase = 2;
        printStatus("sweep->60");
      }
    } else if (gSweepPhase == 2) {
      writeDeg(gDeg - step);
      if (gDeg <= MIN_DEG) {
        gSweepPhase = 3;
        printStatus("sweep->90");
      }
    } else if (gSweepPhase == 3) {
      if (gDeg < CENTER) writeDeg(gDeg + step);
      else if (gDeg > CENTER) writeDeg(gDeg - step);
      if (gDeg == CENTER) {
        gSweepPhase = 0;
        printStatus("sweep-done OK");
      }
    } else if (gDir != 0) {
      writeDeg(gDeg + gDir * step);
      if ((gDir < 0 && gDeg <= MIN_DEG) || (gDir > 0 && gDeg >= MAX_DEG)) {
        gDir = 0;
        printStatus("limit");
      }
    }
  }

  if (now - gLastStatusMs >= 2000) {
    gLastStatusMs = now;
    printStatus("hb");
  }

  delay(2);
}
