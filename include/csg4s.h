#pragma once

#include <Arduino.h>
#include <Wire.h>

// Comfile CSG-4S = NXP SAA1064 4-digit LED driver over I2C.
// Pins: GND, +5V, SDA, SCL. DIP → ADR → 7-bit addr 0x38..0x3B.
// ESP32 soft-I2C on free pins (servos use 13/14/21/27).
// Ref: tronixstuff SAA1064 tutorial (CSG-4S confirmed as SAA1064 under label).

#ifndef CSG_SDA_PIN
#define CSG_SDA_PIN 25
#endif
#ifndef CSG_SCL_PIN
#define CSG_SCL_PIN 26
#endif

static const uint8_t CSG_SAA_ADDRS[] = {0x38, 0x39, 0x3A, 0x3B};

// bit0=A .. bit6=G, bit7=DP
static const uint8_t CSG_SEG_DIGIT[16] = {
    0x3F, 0x06, 0x5B, 0x4F, 0x66, 0x6D, 0x7D, 0x07,  // 0-7
    0x7F, 0x6F, 0x77, 0x7C, 0x39, 0x5E, 0x79, 0x71   // 8-9 A-F
};

class Csg4s {
 public:
  bool begin(uint8_t preferAddr = 0) {
    // ESP32 default I2C is SDA=21/SCL=22 — never claim those (ax3 uses GPIO21).
    Wire.setPins(CSG_SDA_PIN, CSG_SCL_PIN);
    Wire.begin(CSG_SDA_PIN, CSG_SCL_PIN);
    Wire.setClock(100000);
    delay(50);

    Serial.println("[CSG] I2C scan (SAA1064)...");
    _addr = 0;
    for (uint8_t a = 1; a < 127; a++) {
      if (!probe(a)) continue;
      Serial.printf("[CSG] ACK 0x%02X\n", a);
      if (_addr == 0) _addr = a;
    }

    if (preferAddr && probe(preferAddr)) _addr = preferAddr;

    if (_addr == 0) {
      for (uint8_t a : CSG_SAA_ADDRS) {
        if (probe(a)) {
          _addr = a;
          break;
        }
      }
    }

    if (_addr == 0) {
      Serial.println("[CSG] NOT FOUND — 5V/GND/SDA25/SCL26 + DIP(0x38..0x3B)");
      _ok = false;
      return false;
    }

    if (!initChip()) {
      Serial.printf("[CSG] init failed addr=0x%02X\n", _addr);
      _ok = false;
      return false;
    }

    _ok = true;
    Serial.printf("[CSG] OK addr=0x%02X SDA=%d SCL=%d\n",
                  _addr, CSG_SDA_PIN, CSG_SCL_PIN);
    clear();
    delay(20);
    // self-test 8.8.8.8
    uint8_t eight = (uint8_t)(CSG_SEG_DIGIT[8] | 0x80);
    writeDigits(eight, eight, eight, eight);
    delay(300);
    clear();
    showAscii4("----");
    return true;
  }

  bool ok() const { return _ok; }
  uint8_t address() const { return _addr; }

  void clear() {
    writeDigits(0, 0, 0, 0);
  }

  void showDec(int value) {
    if (value < 0) value = 0;
    if (value > 9999) value = 9999;
    char buf[5];
    snprintf(buf, sizeof(buf), "%4d", value);
    showAscii4(buf);
  }

  // 4 chars left→right; supports 0-9 A-F space '-'
  void showAscii4(const char* s4) {
    uint8_t segs[4] = {0, 0, 0, 0};
    for (int i = 0; i < 4; i++) {
      char c = (s4 && s4[i]) ? s4[i] : ' ';
      segs[i] = charToSeg(c);
    }
    // SAA1064 write order starts at digit1 = rightmost
    writeDigits(segs[3], segs[2], segs[1], segs[0]);
  }

  // F{slot}{speed}{mode}  e.g. F351 = slot3 speed5 play; slot10 shows as '0'
  // mode: 0=idle 1=play 2=pause
  void showStatus(int slot1to10, int speed1to9, int mode) {
    char buf[5];
    buf[0] = 'F';
    if (slot1to10 >= 1 && slot1to10 <= 9) buf[1] = (char)('0' + slot1to10);
    else if (slot1to10 == 10) buf[1] = '0';
    else buf[1] = '-';
    buf[2] = (speed1to9 >= 1 && speed1to9 <= 9) ? (char)('0' + speed1to9) : '-';
    buf[3] = (mode == 1) ? '1' : (mode == 2) ? '2' : '0';
    buf[4] = 0;
    showAscii4(buf);
  }

 private:
  bool probe(uint8_t addr) {
    Wire.beginTransmission(addr);
    return Wire.endTransmission() == 0;
  }

  bool initChip() {
    // subaddr 0 → control: dynamic mode, digits 1+3 & 2+4 on, ~12mA
    Wire.beginTransmission(_addr);
    Wire.write((uint8_t)0x00);
    Wire.write((uint8_t)0x47);  // B01000111
    return Wire.endTransmission() == 0;
  }

  // d1=rightmost … d4=leftmost (SAA1064 convention)
  bool writeDigits(uint8_t d1, uint8_t d2, uint8_t d3, uint8_t d4) {
    if (!_ok && _addr == 0) return false;
    Wire.beginTransmission(_addr);
    Wire.write((uint8_t)0x01);  // start at digit 1 (RHS)
    Wire.write(d1);
    Wire.write(d2);
    Wire.write(d3);
    Wire.write(d4);
    return Wire.endTransmission() == 0;
  }

  static uint8_t charToSeg(char c) {
    if (c >= '0' && c <= '9') return CSG_SEG_DIGIT[c - '0'];
    if (c >= 'A' && c <= 'F') return CSG_SEG_DIGIT[10 + (c - 'A')];
    if (c >= 'a' && c <= 'f') return CSG_SEG_DIGIT[10 + (c - 'a')];
    if (c == '-') return 0x40;  // G
    return 0x00;               // blank
  }

  uint8_t _addr = 0;
  bool _ok = false;
};
