#pragma once

#include <Arduino.h>
#include <Preferences.h>
#include "servo_axis.h"

static const int POSE_SLOTS = 10;

// Fixed home (not user-configurable): ax1..ax3=90, ax4(D)=85
static const int HOME_DEG[4] = {
    SERVO_CENTER_DEG, SERVO_CENTER_DEG, SERVO_CENTER_DEG, 85};

struct PoseSlot {
  bool valid = false;
  int deg[4] = {SERVO_CENTER_DEG, SERVO_CENTER_DEG, SERVO_CENTER_DEG, SERVO_CENTER_DEG};
};

inline char poseSlotDigitChar(int slot1to10) {
  if (slot1to10 >= 1 && slot1to10 <= 9) return (char)('0' + slot1to10);
  if (slot1to10 == 10) return '0';  // F10 / key 0
  return '-';
}

class PoseMemory {
 public:
  void begin() {
    _prefs.begin("poses", false);
    for (int i = 0; i < POSE_SLOTS; i++) {
      char key[8];
      snprintf(key, sizeof(key), "s%d", i + 1);
      PoseSlot slot;
      size_t n = _prefs.getBytes(key, &slot, sizeof(slot));
      if (n == sizeof(slot) && slot.valid) {
        _slots[i] = slot;
        Serial.printf("[MEM] F%d loaded %d,%d,%d,%d\n", i + 1,
                      slot.deg[0], slot.deg[1], slot.deg[2], slot.deg[3]);
      } else {
        _slots[i].valid = false;
        Serial.printf("[MEM] F%d empty\n", i + 1);
      }
    }
    Serial.printf("[MEM] HOME fixed %d,%d,%d,%d\n",
                  HOME_DEG[0], HOME_DEG[1], HOME_DEG[2], HOME_DEG[3]);
  }

  bool save(int slot1to10, const int deg[4]) {
    if (slot1to10 < 1 || slot1to10 > POSE_SLOTS) return false;
    PoseSlot& s = _slots[slot1to10 - 1];
    s.valid = true;
    for (int i = 0; i < 4; i++) {
      s.deg[i] = clampServoDeg(i, deg[i]);
    }
    char key[8];
    snprintf(key, sizeof(key), "s%d", slot1to10);
    _prefs.putBytes(key, &s, sizeof(s));
    Serial.printf("[MEM] F%d SAVE %d,%d,%d,%d\n", slot1to10,
                  s.deg[0], s.deg[1], s.deg[2], s.deg[3]);
    return true;
  }

  bool recall(int slot1to10, int outDeg[4]) const {
    if (slot1to10 < 1 || slot1to10 > POSE_SLOTS) return false;
    const PoseSlot& s = _slots[slot1to10 - 1];
    if (!s.valid) {
      Serial.printf("[MEM] F%d empty - nothing to recall\n", slot1to10);
      return false;
    }
    for (int i = 0; i < 4; i++) outDeg[i] = s.deg[i];
    Serial.printf("[MEM] %d RECALL F%d -> %d,%d,%d,%d\n", slot1to10, slot1to10,
                  outDeg[0], outDeg[1], outDeg[2], outDeg[3]);
    return true;
  }

  bool hasSlot(int slot1to10) const {
    if (slot1to10 < 1 || slot1to10 > POSE_SLOTS) return false;
    return _slots[slot1to10 - 1].valid;
  }

  int nextValidSlot(int after) const {
    for (int n = 1; n <= POSE_SLOTS; n++) {
      int s = ((after + n - 1) % POSE_SLOTS) + 1;
      if (_slots[s - 1].valid) return s;
    }
    return 0;
  }

  void getHome(int outDeg[4]) const {
    for (int i = 0; i < 4; i++) outDeg[i] = HOME_DEG[i];
  }

  // Last commanded pose (for gentle boot: restore PWM here, then slew to HOME)
  bool loadLast(int outDeg[4]) {
    PoseSlot slot;
    size_t n = _prefs.getBytes("last", &slot, sizeof(slot));
    if (n != sizeof(slot) || !slot.valid) return false;
    for (int i = 0; i < 4; i++) outDeg[i] = clampServoDeg(i, slot.deg[i]);
    return true;
  }

  void saveLast(const int deg[4]) {
    PoseSlot slot;
    slot.valid = true;
    for (int i = 0; i < 4; i++) slot.deg[i] = clampServoDeg(i, deg[i]);
    _prefs.putBytes("last", &slot, sizeof(slot));
  }

  void clearAll() {
    for (int i = 0; i < POSE_SLOTS; i++) {
      _slots[i].valid = false;
      for (int a = 0; a < 4; a++) _slots[i].deg[a] = SERVO_CENTER_DEG;
      char key[8];
      snprintf(key, sizeof(key), "s%d", i + 1);
      _prefs.remove(key);
    }
    Serial.println("[MEM] ALL slots cleared (F1-F10)");
  }

 private:
  Preferences _prefs;
  PoseSlot _slots[POSE_SLOTS];
};

inline int hidUsageToFnSlot(uint8_t usage) {
  // F1..F10 = 0x3A..0x43
  if (usage >= 0x3A && usage <= 0x43) return (int)(usage - 0x3A + 1);
  return 0;
}

inline int hidUsageToDigitSlot(uint8_t usage) {
  // 1..9 = 0x1E..0x26 -> slots 1..9; 0 = 0x27 -> slot 10
  if (usage >= 0x1E && usage <= 0x26) return (int)(usage - 0x1E + 1);
  if (usage == 0x27) return 10;
  return 0;
}

static const uint16_t HID_CONSUMER_PLAY_PAUSE = 0x00CD;
static const uint16_t HID_CONSUMER_HOME = 0x0223;   // goto fixed HOME
static const uint16_t HID_CONSUMER_CLEAR = 0x0221;  // clear F1-F10 memory
static const uint16_t HID_CONSUMER_TOUR_STOP = 0x0196;  // AL Internet Browser / WWW
