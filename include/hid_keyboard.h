#pragma once

#include <Arduino.h>

// Axis IDs for hold/repeat
enum AxisId : uint8_t {
  AX1_1 = 0,
  AX1_2,
  AX2_1,
  AX2_2,
  AX3_1,
  AX3_2,
  AX4_1,
  AX4_2,
  AXIS_COUNT
};

static const uint8_t AXIS_KB_MASK =
    (1u << AX3_1) | (1u << AX3_2) | (1u << AX4_1) | (1u << AX4_2);
static const uint8_t AXIS_CONSUMER_MASK =
    (1u << AX1_1) | (1u << AX1_2) | (1u << AX2_1) | (1u << AX2_2);

inline const char* axisName(uint8_t id) {
  switch (id) {
    case AX1_1: return "ax1-1";
    case AX1_2: return "ax1-2";
    case AX2_1: return "ax2-1";
    case AX2_2: return "ax2-2";
    case AX3_1: return "ax3-1";
    case AX3_2: return "ax3-2";
    case AX4_1: return "ax4-1";
    case AX4_2: return "ax4-2";
    default: return "?";
  }
}

inline int mapKeyboardUsageToAxisId(uint8_t usage) {
  switch (usage) {
    case 0x50: return AX3_2;  // LEFT  -> + (방향 반전)
    case 0x4F: return AX3_1;  // RIGHT -> -
    case 0x52: return AX4_2;  // UP    -> + (방향 반전)
    case 0x51: return AX4_1;  // DOWN  -> -
    default: return -1;
  }
}

inline int mapConsumerUsageToAxisId(uint16_t usage) {
  switch (usage) {
    case 0x00B6: return AX1_1;  // Prev
    case 0x00B5: return AX1_2;  // Next
    case 0x00E9: return AX2_1;  // Vol+
    case 0x00EA: return AX2_2;  // Vol-
    default: return -1;
  }
}

inline const char* mapKeyboardUsageToAxis(uint8_t usage) {
  int id = mapKeyboardUsageToAxisId(usage);
  return id >= 0 ? axisName((uint8_t)id) : nullptr;
}

inline const char* mapConsumerUsageToAxis(uint16_t usage) {
  int id = mapConsumerUsageToAxisId(usage);
  return id >= 0 ? axisName((uint8_t)id) : nullptr;
}

// USB HID Usage IDs for keyboard (partial)
inline char hidUsageToAscii(uint8_t usage, bool shift) {
  if (usage >= 0x04 && usage <= 0x1d) {
    char c = 'a' + (usage - 0x04);
    return shift ? (char)(c - 32) : c;
  }
  if (usage >= 0x1e && usage <= 0x26) {
    const char* n = shift ? "!@#$%^&*(" : "123456789";
    return n[usage - 0x1e];
  }
  if (usage == 0x27) return shift ? ')' : '0';
  if (usage == 0x28) return '\n';
  if (usage == 0x2c) return ' ';
  if (usage == 0x2d) return shift ? '_' : '-';
  if (usage == 0x2e) return shift ? '+' : '=';
  if (usage == 0x2f) return shift ? '{' : '[';
  if (usage == 0x30) return shift ? '}' : ']';
  if (usage == 0x31) return shift ? '|' : '\\';
  if (usage == 0x33) return shift ? ':' : ';';
  if (usage == 0x34) return shift ? '"' : '\'';
  if (usage == 0x35) return shift ? '~' : '`';
  if (usage == 0x36) return shift ? '<' : ',';
  if (usage == 0x37) return shift ? '>' : '.';
  if (usage == 0x38) return shift ? '?' : '/';
  if (usage == 0x2a) return 0x08;
  if (usage == 0x2b) return '\t';
  return 0;
}

inline const char* hidUsageName(uint8_t usage) {
  switch (usage) {
    case 0x28: return "ENTER";
    case 0x29: return "ESC";
    case 0x2a: return "BACKSPACE";
    case 0x2b: return "TAB";
    case 0x2c: return "SPACE";
    case 0x39: return "CAPS";
    case 0x3A: return "F1";
    case 0x3B: return "F2";
    case 0x3C: return "F3";
    case 0x3D: return "F4";
    case 0x3E: return "F5";
    case 0x3F: return "F6";
    case 0x40: return "F7";
    case 0x41: return "F8";
    case 0x42: return "F9";
    case 0x4f: return "RIGHT";
    case 0x50: return "LEFT";
    case 0x51: return "DOWN";
    case 0x52: return "UP";
    default: return nullptr;
  }
}

inline const char* consumerUsageName(uint16_t usage) {
  switch (usage) {
    case 0x00B5: return "NEXT_TRACK";
    case 0x00B6: return "PREV_TRACK";
    case 0x00CD: return "PLAY_PAUSE";
    case 0x00E2: return "MUTE";
    case 0x00E9: return "VOL_UP";
    case 0x00EA: return "VOL_DOWN";
    case 0x0196: return "WWW_TOUR_STOP";
    case 0x0221: return "CLEAR_MEM";
    case 0x0223: return "HOME";
    default: return nullptr;
  }
}

struct HidBootKeyboardReport {
  uint8_t modifiers;
  uint8_t reserved;
  uint8_t keys[6];
};

inline bool parseBootKeyboardReport(const uint8_t* data, size_t len, HidBootKeyboardReport* out) {
  if (len < 8 || out == nullptr) return false;
  out->modifiers = data[0];
  out->reserved = data[1];
  for (int i = 0; i < 6; i++) out->keys[i] = data[2 + i];
  return true;
}

// Replace keyboard-axis bits from full key state report
inline uint8_t keyboardAxesMask(const HidBootKeyboardReport& r) {
  uint8_t m = 0;
  for (int i = 0; i < 6; i++) {
    int id = mapKeyboardUsageToAxisId(r.keys[i]);
    if (id >= 0) m |= (1u << id);
  }
  return m;
}

inline uint8_t consumerAxisMask(uint16_t usage) {
  if (usage == 0) return 0;
  int id = mapConsumerUsageToAxisId(usage);
  return id >= 0 ? (uint8_t)(1u << id) : 0;
}

inline void printBootKeyboardReport(const HidBootKeyboardReport& r) {
  bool shift = (r.modifiers & 0x22) != 0;
  Serial.printf("  mods=0x%02X", r.modifiers);
  if (r.modifiers & 0x01) Serial.print(" LCtrl");
  if (r.modifiers & 0x02) Serial.print(" LShift");
  if (r.modifiers & 0x04) Serial.print(" LAlt");
  if (r.modifiers & 0x08) Serial.print(" LGui");
  if (r.modifiers & 0x10) Serial.print(" RCtrl");
  if (r.modifiers & 0x20) Serial.print(" RShift");
  if (r.modifiers & 0x40) Serial.print(" RAlt");
  if (r.modifiers & 0x80) Serial.print(" RGui");
  Serial.print(" | keys:");

  bool any = false;
  for (int i = 0; i < 6; i++) {
    uint8_t u = r.keys[i];
    if (u == 0) continue;
    any = true;
    Serial.printf(" 0x%02X", u);
    const char* axis = mapKeyboardUsageToAxis(u);
    const char* name = hidUsageName(u);
    char ascii = hidUsageToAscii(u, shift);
    if (axis) Serial.printf("(%s)", axis);
    else if (name) Serial.printf("(%s)", name);
    else if (ascii >= 32 && ascii < 127) Serial.printf("('%c')", ascii);
  }
  if (!any) Serial.print(" (none / key-up)");
  Serial.println();
}

inline void printConsumerReport(const uint8_t* data, size_t len) {
  if (len < 2) {
    Serial.println("  (raw report)");
    return;
  }
  uint16_t usage = (uint16_t)data[0] | ((uint16_t)data[1] << 8);
  if (usage == 0) {
    Serial.println("  consumer: (release / key-up)");
    return;
  }
  const char* axis = mapConsumerUsageToAxis(usage);
  const char* name = consumerUsageName(usage);
  Serial.printf("  consumer: 0x%04X", usage);
  if (axis) Serial.printf(" (%s)", axis);
  else if (name) Serial.printf(" (%s)", name);
  Serial.println();
}
