/**
 * BLE keyboard HID host (NimBLE central)
 * Upload:  pio run -e ble_keyboard -t upload
 * Monitor: pio device monitor -e ble_keyboard
 */
#include <Arduino.h>
#include <NimBLEDevice.h>
#include <cctype>
#include "soc/soc.h"
#include "soc/rtc_cntl_reg.h"
#include "hid_keyboard.h"
#include "servo_axis.h"
#include "pose_memory.h"
#include "csg4s.h"

static const NimBLEUUID UUID_HID_SERVICE("1812");
static const NimBLEUUID UUID_HID_REPORT("2A4D");
static const NimBLEUUID UUID_HID_BOOT_KB_IN("2A22");
static const NimBLEUUID UUID_HID_PROTOCOL_MODE("2A4E");
static const NimBLEUUID UUID_HID_CONTROL_POINT("2A4C");
static const NimBLEUUID UUID_REPORT_REF("2908");
static const NimBLEUUID UUID_BATTERY_LEVEL("2A19");
static const NimBLEUUID UUID_BATTERY_SERVICE("180F");
// Some devices expose Battery Level under HID as 0x2A33 in logs — keep generic notify

// Single-keyboard mode: prefer direct reconnect to PREFERRED_ADDR (fast path).
static const uint32_t RECONNECT_DELAY_MS = 200;   // after drop, before first retry
static const uint32_t RECONNECT_RETRY_MS = 400;   // between failed direct attempts
static const uint32_t CONNECT_TIMEOUT_S = 5;      // fail fast when KB is asleep
static const uint32_t SCAN_DURATION_S = 3;        // short fallback scan
static const int SCAN_EVERY_N_FAILS = 8;          // occasional scan if direct fails

static NimBLEClient* gClient = nullptr;
static NimBLEAdvertisedDevice* gTarget = nullptr;
static bool gDoConnect = false;
static bool gConnected = false;
static bool gEverReady = false;       // true after first successful HID subscribe
static bool gNeedReconnect = false;
static int gReconnectAttempt = 0;     // 0 = idle, N = attempts since last drop/scan
static uint32_t gReconnectAtMs = 0;    // when to start next attempt
static uint32_t gLastScanMs = 0;
static uint32_t gLastHeartbeatMs = 0;
static NimBLERemoteCharacteristic* gBootKb = nullptr;
static std::vector<NimBLERemoteCharacteristic*> gInputReports;
// Only these HID Input reports may drive servos (others cause phantom jog).
static uint16_t gKbReportHandle = 0;        // Report Ref id=1 (keyboard)
static uint16_t gConsumerReportHandle = 0;  // Report Ref id=2 (consumer)
static const bool HID_VERBOSE = true;       // show raw key/consumer reports on serial
static int gBestScanScore = -10000;
static NimBLEAddress gLastAddr;
static bool gHaveLastAddr = false;
static std::string gLastName;
// Only keyboard used with this firmware (update if yours differs)
static const char* PREFERRED_ADDR = "b8:90:13:e3:5c:48";

static void seedPreferredAddr() {
  gLastAddr = NimBLEAddress(PREFERRED_ADDR);
  gHaveLastAddr = true;
  if (gLastName.empty()) gLastName = "preferred";
}

// Hold state for mapped axes; servos ramp smoothly while held
static uint8_t gHeldAxes = 0;
static ServoBank gServos;
static PoseMemory gMem;
static Csg4s gCsg;
static uint8_t gPrevKbKeys[6] = {0};
static uint16_t gPrevConsumer = 0;
static uint32_t gLastConsumerEdgeMs = 0;
static const uint32_t CONSUMER_DEBOUNCE_MS = 60;  // ignore chatter on media keys

enum TourState : uint8_t { TOUR_IDLE = 0, TOUR_PLAY, TOUR_PAUSE };
static TourState gTour = TOUR_IDLE;
static int gTourSlot = 0;  // last/current slot 1..9
static uint32_t gTourArriveMs = 0;
static const uint32_t TOUR_DWELL_MS = 350;  // pause after all axes idle before next slot
// Tour PLAY: suppress BLE reconnect except a short window at F1 arrival.
static const uint32_t TOUR_F1_BLE_WINDOW_MS = 2000;
static const uint32_t TOUR_F1_CONNECT_TIMEOUT_S = 2;  // fail fast inside F1 window
static uint32_t gTourBleUntilMs = 0;  // millis deadline; 0 = no window

static bool tourPlayBleWindowOpen() {
  if (gTour != TOUR_PLAY) return false;
  if (gTourBleUntilMs == 0) return false;
  return (int32_t)(millis() - gTourBleUntilMs) < 0;
}

// During PLAY, only reconnect inside the F1 window. IDLE/PAUSE: always allowed.
static bool bleReconnectAllowedNow() {
  if (gTour != TOUR_PLAY) return true;
  return tourPlayBleWindowOpen();
}

// SPACE cycles display: status -> A(ax1) -> B -> C -> D -> status...
enum DispPage : uint8_t {
  DISP_STATUS = 0,
  DISP_AX1,
  DISP_AX2,
  DISP_AX3,
  DISP_AX4,
  DISP_PAGE_COUNT
};
static DispPage gDispPage = DISP_STATUS;
static uint32_t gLastDispMs = 0;
static const uint32_t DISP_ANGLE_REFRESH_MS = 100;

static void refreshDisplay() {
  if (!gCsg.ok()) return;
  if (gDispPage == DISP_STATUS) {
    // Disconnected: blink LOSt <-> normal status so tour info is not lost.
    if (!gConnected && ((millis() / 500) & 1)) {
      gCsg.showBleLost();
      return;
    }
    int mode = (gTour == TOUR_PLAY) ? 1 : (gTour == TOUR_PAUSE) ? 2 : 0;
    gCsg.showStatus(gTourSlot, getSpeedLevel(), mode);
    return;
  }
  int deg[4];
  gServos.getAngles(deg);
  const int idx = (int)gDispPage - 1;  // 0..3
  static const char letters[4] = {'A', 'B', 'C', 'D'};
  gCsg.showAxisDeg(letters[idx], deg[idx]);
}

static bool keyInList(uint8_t usage, const uint8_t keys[6]) {
  if (usage == 0) return false;
  for (int i = 0; i < 6; i++) if (keys[i] == usage) return true;
  return false;
}

static void tourStop(const char* why) {
  gTour = TOUR_IDLE;
  gTourArriveMs = 0;
  gTourBleUntilMs = 0;
  Serial.printf("[TOUR] stop (%s)\n", why);
  refreshDisplay();
}

static bool tourGotoSlot(int slot) {
  int deg[4];
  if (!gMem.recall(slot, deg)) return false;
  gHeldAxes = 0;
  gServos.gotoAngles(deg);
  gTourSlot = slot;
  gTourArriveMs = 0;
  Serial.printf("[TOUR] goto F%d\n", slot);
  refreshDisplay();
  return true;
}

static void tourAdvance() {
  int next = gMem.nextValidSlot(gTourSlot);
  if (!next) {
    tourStop("no saved poses");
    return;
  }
  tourGotoSlot(next);
}

static void onPlayPauseClick() {
  if (gTour == TOUR_IDLE) {
    int next = gMem.nextValidSlot(0);
    if (!next) {
      Serial.println("[TOUR] no F1-F10 saved - cannot play");
      return;
    }
    gTour = TOUR_PLAY;
    gTourSlot = 0;
    gHeldAxes = 0;
    gServos.stopJogOnly();
    Serial.println("[TOUR] play");
    tourGotoSlot(next);
    refreshDisplay();
  } else if (gTour == TOUR_PLAY) {
    gTour = TOUR_PAUSE;
    gTourArriveMs = 0;
    Serial.printf("[TOUR] pause at F%d\n", gTourSlot);
    refreshDisplay();
  } else {
    gTour = TOUR_PLAY;
    Serial.printf("[TOUR] resume from F%d\n", gTourSlot);
    refreshDisplay();
    if (!gServos.busy()) {
      gTourArriveMs = millis();  // dwell then advance
    }
  }
}

static void serviceTour() {
  if (gTour != TOUR_PLAY) return;
  if (gServos.busy()) {
    gTourArriveMs = 0;
    return;
  }
  if (gTourArriveMs == 0) {
    gTourArriveMs = millis();
    // Each lap at F1: brief BLE reconnect window (keyboard may be asleep).
    if (gTourSlot == 1 && !gConnected) {
      gTourBleUntilMs = millis() + TOUR_F1_BLE_WINDOW_MS;
      seedPreferredAddr();
      gNeedReconnect = true;
      gReconnectAttempt = 0;
      gReconnectAtMs = millis();  // try immediately inside window
      Serial.printf("[TOUR] F1 BLE window %lums\n",
                    (unsigned long)TOUR_F1_BLE_WINDOW_MS);
    }
  }
  // Stay at F1 for the BLE window when disconnected; else normal dwell.
  uint32_t dwell = TOUR_DWELL_MS;
  if (gTourSlot == 1 && !gConnected && tourPlayBleWindowOpen()) {
    dwell = TOUR_F1_BLE_WINDOW_MS;
  }
  if (millis() - gTourArriveMs < dwell) return;
  gTourArriveMs = 0;
  if (gTourSlot == 1) gTourBleUntilMs = 0;  // close window before leaving F1
  tourAdvance();
}

static void gotoHome() {
  tourStop("home");
  int deg[4];
  gMem.getHome(deg);
  gHeldAxes = 0;
  gServos.gotoAngles(deg);
  Serial.printf("[MEM] HOME RECALL -> %d,%d,%d,%d\n",
                deg[0], deg[1], deg[2], deg[3]);
}

// ESC: hard init — stop tour/jog, reassert PWM, slew smoothly to HOME.
static void hardInit() {
  tourStop("esc init");
  gHeldAxes = 0;
  int deg[4];
  gMem.getHome(deg);
  gServos.reinitToAngles(deg);  // reassert hold, then goto (no snap jerk)
  gTourSlot = 0;
  refreshDisplay();
  Serial.printf("[INIT] ESC soft HOME %d,%d,%d,%d (PWM reassert + slew)\n",
                deg[0], deg[1], deg[2], deg[3]);
}

static void handlePoseKeys(const HidBootKeyboardReport& report) {
  const bool tourActive = (gTour == TOUR_PLAY || gTour == TOUR_PAUSE);

  for (int i = 0; i < 6; i++) {
    uint8_t u = report.keys[i];
    if (u == 0) continue;
    if (keyInList(u, gPrevKbKeys)) continue;

    // ESC works anytime (recovery from limp / stuck limit / tour).
    if (u == 0x29) {
      hardInit();
      continue;
    }

    // SPACE: cycle display page (status / A / B / C / D) anytime.
    if (u == 0x2C) {
      gDispPage = (DispPage)((gDispPage + 1) % DISP_PAGE_COUNT);
      Serial.printf("[DISP] page=%u\n", (unsigned)gDispPage);
      refreshDisplay();
      continue;
    }

    // Speed (= faster / - slower): IDLE + tour. Applies to jog and goto/tour.
    if (u == 0x2E || u == 0x2D) {
      int delta = (u == 0x2E) ? +1 : -1;
      if (bumpSpeedLevel(delta)) {
        Serial.printf("[SPEED] level=%d (%.0f deg/s)\n",
                      getSpeedLevel(), (double)getServoSpeedDegS());
      } else {
        Serial.printf("[SPEED] level=%d (limit)\n", getSpeedLevel());
      }
      refreshDisplay();
      continue;
    }

    // Tour: ignore F-save / recall / other keys while PLAY/PAUSE
    if (tourActive) {
      continue;
    }

    int fn = hidUsageToFnSlot(u);
    if (fn) {
      int deg[4];
      gServos.getAngles(deg);
      gMem.save(fn, deg);
      gTourSlot = fn;
      refreshDisplay();
      continue;
    }
    int digit = hidUsageToDigitSlot(u);
    if (digit) {
      tourStop("manual recall");
      int deg[4];
      if (gMem.recall(digit, deg)) {
        gHeldAxes = 0;
        gServos.gotoAngles(deg);
        gTourSlot = digit;
        refreshDisplay();
      }
    }
  }
  for (int i = 0; i < 6; i++) gPrevKbKeys[i] = report.keys[i];
}

static void handleConsumerEdge(uint16_t usage) {
  const bool tourActive = (gTour == TOUR_PLAY || gTour == TOUR_PAUSE);
  const uint32_t now = millis();
  // Release always accepted (clear held). Presses debounced against chatter.
  if (usage != 0 && usage == gPrevConsumer) return;
  if (usage != 0 && (now - gLastConsumerEdgeMs) < CONSUMER_DEBOUNCE_MS &&
      usage != HID_CONSUMER_PLAY_PAUSE && usage != HID_CONSUMER_TOUR_STOP &&
      usage != HID_CONSUMER_HOME && usage != HID_CONSUMER_CLEAR) {
    return;
  }
  if (usage != gPrevConsumer) gLastConsumerEdgeMs = now;

  if (usage == HID_CONSUMER_PLAY_PAUSE && gPrevConsumer != HID_CONSUMER_PLAY_PAUSE) {
    onPlayPauseClick();
    gPrevConsumer = usage;
    return;
  }

  // WWW / AL Internet Browser (0x0196): end tour -> IDLE
  if (usage == HID_CONSUMER_TOUR_STOP && gPrevConsumer != HID_CONSUMER_TOUR_STOP) {
    if (tourActive) {
      tourStop("www stop");
      gHeldAxes = 0;
      gServos.stopJogOnly();
      refreshDisplay();
    }
    gPrevConsumer = usage;
    return;
  }

  // Tour: ignore HOME / CLEAR / other consumer (jog axes cleared elsewhere)
  if (tourActive) {
    gPrevConsumer = usage;
    return;
  }

  if (usage == HID_CONSUMER_HOME && gPrevConsumer != HID_CONSUMER_HOME) {
    gotoHome();
  }
  if (usage == HID_CONSUMER_CLEAR && gPrevConsumer != HID_CONSUMER_CLEAR) {
    tourStop("mem clear");
    gMem.clearAll();
  }
  gPrevConsumer = usage;
}

static void clearHeldAxes() {
  // Disconnect/jog-clear must not cancel pose recall / tour goto targets.
  gHeldAxes = 0;
  gServos.stopJogOnly();
}

static void emitAxesLog(uint8_t mask, bool repeat) {
  for (uint8_t i = 0; i < AXIS_COUNT; i++) {
    if (mask & (1u << i)) {
      if (repeat) Serial.printf("[AXIS] %s (repeat)\n", axisName(i));
      else Serial.printf("[AXIS] %s\n", axisName(i));
    }
  }
}

static void updateHeldAxes(uint8_t clearMask, uint8_t setMask) {
  uint8_t prev = gHeldAxes;
  gHeldAxes = (uint8_t)((gHeldAxes & ~clearMask) | setMask);
  uint8_t pressed = (uint8_t)(gHeldAxes & ~prev);
  if (pressed) emitAxesLog(pressed, false);
}

static int heldDir(uint8_t negBit, uint8_t posBit) {
  int dir = 0;
  if (gHeldAxes & (1u << negBit)) dir -= 1;
  if (gHeldAxes & (1u << posBit)) dir += 1;
  return dir;
}

static void serviceHeldInputs() {
  // Tour: ignore jogging entirely (speed + Play/Pause only)
  if (gTour == TOUR_PLAY || gTour == TOUR_PAUSE) {
    if (gHeldAxes) gHeldAxes = 0;
    gServos.stopJogOnly();
    gServos.service();
    serviceTour();
    return;
  }

#ifdef SERVO_SINGLE_PIN
  int dir = 0;
  dir += heldDir(AX1_1, AX1_2);
  dir += heldDir(AX2_1, AX2_2);
  dir += heldDir(AX3_1, AX3_2);
  dir += heldDir(AX4_1, AX4_2);
  if (dir < -1) dir = -1;
  if (dir > 1) dir = 1;
  gServos.setDirection(0, dir);
#else
  int d0 = heldDir(AX1_1, AX1_2);
  int d1 = heldDir(AX2_1, AX2_2);
  int d2 = heldDir(AX3_1, AX3_2);
  int d3 = heldDir(AX4_1, AX4_2);
  gServos.setDirection(0, d0);
  gServos.setDirection(1, d1);
  gServos.setDirection(2, d2);
  gServos.setDirection(3, d3);
#endif
  gServos.service();
  serviceTour();
}

static bool nameLooksLikeKeyboard(const std::string& name) {
  if (name.empty()) return false;
  std::string lower;
  lower.reserve(name.size());
  for (char c : name) lower.push_back((char)tolower((unsigned char)c));
  return lower.find("keyboard") != std::string::npos ||
         lower.find("keybord") != std::string::npos ||  // common typo
         lower.find("kbd") != std::string::npos;
}

class AdvCallbacks : public NimBLEAdvertisedDeviceCallbacks {
  void onResult(NimBLEAdvertisedDevice* adv) override {
    std::string name = adv->getName();
    std::string addr = adv->getAddress().toString();
    bool hasHid = adv->isAdvertisingService(UUID_HID_SERVICE);
    bool kbName = nameLooksLikeKeyboard(name);
    bool preferred = (addr == PREFERRED_ADDR);
    uint16_t appearance = adv->haveAppearance() ? adv->getAppearance() : 0;
    // BLE appearance: 0x03C1 = keyboard, 0x03C0 = HID generic
    bool kbAppear = (appearance == 0x03C1 || appearance == 0x03C0);
    int rssi = adv->getRSSI();

    // Debug: print anything with a name or service UUID once-ish (throttle by printing all named)
    if (!name.empty() || hasHid || preferred || kbAppear || adv->haveServiceUUID()) {
      Serial.printf("[ADV] %s rssi=%d addr=%s hid=%d app=0x%04X svc=%s\n",
                    name.empty() ? "-" : name.c_str(),
                    rssi,
                    addr.c_str(),
                    hasHid ? 1 : 0,
                    appearance,
                    adv->haveServiceUUID() ? adv->getServiceUUID().toString().c_str() : "-");
    }

    if (!hasHid && !kbName && !preferred && !kbAppear) return;

    Serial.printf("[SCAN] candidate %s RSSI=%d addr=%s HID=%s\n",
                  name.empty() ? "(no-name)" : name.c_str(),
                  rssi, addr.c_str(), hasHid ? "yes" : "no");

    int score = 0;
    if (preferred) score += 200;
    if (kbName) score += 100;
    if (hasHid) score += 50;
    if (kbAppear) score += 40;
    score += (rssi + 100);

    if (score > gBestScanScore) {
      gBestScanScore = score;
      if (gTarget) delete gTarget;
      gTarget = new NimBLEAdvertisedDevice(*adv);
    }

    // Single keyboard: stop scan as soon as preferred MAC is seen
    if (preferred) {
      Serial.println("[SCAN] preferred MAC found - stop scan");
      NimBLEDevice::getScan()->stop();
    }
  }
};

class ClientCallbacks : public NimBLEClientCallbacks {
  void onConnect(NimBLEClient* c) override {
    Serial.println("[BLE] connected");
  }
  void onDisconnect(NimBLEClient* c) override {
    gConnected = false;
    gDoConnect = false;
    gBootKb = nullptr;
    gInputReports.clear();
    gKbReportHandle = 0;
    gConsumerReportHandle = 0;
    clearHeldAxes();

    if (gEverReady && !gNeedReconnect) {
      Serial.println("[BLE] disconnected");
      seedPreferredAddr();
      gNeedReconnect = true;
      gReconnectAttempt = 0;
      gReconnectAtMs = millis() + RECONNECT_DELAY_MS;
      Serial.printf("[BLE] reconnect in %lums...\n",
                    (unsigned long)RECONNECT_DELAY_MS);
    } else if (!gNeedReconnect) {
      Serial.println("[BLE] disconnected - direct preferred retry");
      seedPreferredAddr();
      gNeedReconnect = true;
      gReconnectAttempt = 0;
      gReconnectAtMs = millis() + RECONNECT_DELAY_MS;
    }
    refreshDisplay();
  }
  void onAuthenticationComplete(ble_gap_conn_desc* desc) override {
    Serial.printf("[BLE] auth complete encrypted=%d bonded=%d\n",
                  desc->sec_state.encrypted, desc->sec_state.bonded);
  }
};

// Per-handle dedupe (kb + consumer can alternate without wiping each other).
struct HidDedupe {
  uint16_t handle = 0xFFFF;
  size_t len = 0;
  uint8_t data[16];
};
static HidDedupe gHidDedup[2];

static bool isDuplicateHidPayload(uint16_t handle, const uint8_t* data, size_t len) {
  if (len == 0 || len > sizeof(gHidDedup[0].data)) return false;
  HidDedupe* slot = &gHidDedup[0];
  if (gHidDedup[0].handle == handle) slot = &gHidDedup[0];
  else if (gHidDedup[1].handle == handle) slot = &gHidDedup[1];
  else if (gHidDedup[0].handle == 0xFFFF) slot = &gHidDedup[0];
  else slot = &gHidDedup[1];

  if (slot->handle == handle && slot->len == len &&
      memcmp(data, slot->data, len) == 0) {
    return true;
  }
  slot->handle = handle;
  slot->len = len;
  memcpy(slot->data, data, len);
  return false;
}

static bool isKeyboardReportHandle(uint16_t handle) {
  if (gBootKb && handle == gBootKb->getHandle()) return true;
  return gKbReportHandle != 0 && handle == gKbReportHandle;
}

static bool isConsumerReportHandle(uint16_t handle) {
  return gConsumerReportHandle != 0 && handle == gConsumerReportHandle;
}

static void handleHidPayload(const char* tag, NimBLERemoteCharacteristic* chr,
                             uint8_t* data, size_t len) {
  const uint16_t handle = chr ? chr->getHandle() : 0;
  // Identical consecutive reports (notify spam / POLL reread) — skip log + reapply.
  if (isDuplicateHidPayload(handle, data, len)) return;

  const bool asKb = isKeyboardReportHandle(handle);
  const bool asCons = isConsumerReportHandle(handle);
  // Extra Input reports (mouse/vendor/etc.) must not drive axes.
  if (!asKb && !asCons) {
    if (HID_VERBOSE) {
      Serial.printf("[%s] ignore handle=%u len=%u\n", tag, handle, (unsigned)len);
    }
    return;
  }

  if (HID_VERBOSE) {
    Serial.printf("[%s] uuid=%s handle=%u len=%u hex=",
                  tag,
                  chr ? chr->getUUID().toString().c_str() : "?",
                  handle,
                  (unsigned)len);
    for (size_t i = 0; i < len; i++) Serial.printf("%02X", data[i]);
    Serial.println();
  }

  // Boot keyboard is exactly 8 bytes. Report-protocol keyboard input is often
  // also 8 bytes (modifier, reserved, 6 keys) or 8+ with report id stripped by stack.
  HidBootKeyboardReport report;
  if (asKb && len == 8 && parseBootKeyboardReport(data, len, &report)) {
    if (HID_VERBOSE) printBootKeyboardReport(report);
    handlePoseKeys(report);
    if (gTour == TOUR_PLAY || gTour == TOUR_PAUSE) {
      updateHeldAxes(AXIS_KB_MASK, 0);
    } else {
      updateHeldAxes(AXIS_KB_MASK, keyboardAxesMask(report));
    }
  } else if (asKb && len == 9) {
    if (parseBootKeyboardReport(data + 1, 8, &report)) {
      if (HID_VERBOSE) {
        Serial.printf("  reportId=%u\n", data[0]);
        printBootKeyboardReport(report);
      }
      handlePoseKeys(report);
      if (gTour == TOUR_PLAY || gTour == TOUR_PAUSE) {
        updateHeldAxes(AXIS_KB_MASK, 0);
      } else {
        updateHeldAxes(AXIS_KB_MASK, keyboardAxesMask(report));
      }
    }
  } else if (asCons && len == 2) {
    if (HID_VERBOSE) printConsumerReport(data, len);
    uint16_t usage = (uint16_t)data[0] | ((uint16_t)data[1] << 8);
    handleConsumerEdge(usage);
    if (gTour == TOUR_PLAY || gTour == TOUR_PAUSE) {
      updateHeldAxes(AXIS_CONSUMER_MASK, 0);
    } else {
      updateHeldAxes(AXIS_CONSUMER_MASK, consumerAxisMask(usage));
    }
  } else if (HID_VERBOSE && len > 0) {
    Serial.println("  (raw report)");
  }
}

static void notifyCB(NimBLERemoteCharacteristic* chr, uint8_t* data, size_t len, bool isNotify) {
  handleHidPayload(isNotify ? "NOTIFY" : "INDICATE", chr, data, len);
}

static bool subscribeChar(NimBLERemoteCharacteristic* ch, const char* label) {
  if (!ch) return false;
  Serial.printf("[HID] %s uuid=%s handle=%u notify=%d indicate=%d\n",
                label,
                ch->getUUID().toString().c_str(),
                ch->getHandle(),
                ch->canNotify(),
                ch->canIndicate());

  if (ch->canNotify()) {
    // response=true: wait for CCCD write ack
    if (ch->subscribe(true, notifyCB, true)) {
      Serial.println("  -> notify CCCD OK");
      return true;
    }
    Serial.println("  -> notify subscribe FAILED");
  }
  if (ch->canIndicate()) {
    if (ch->subscribe(false, notifyCB, true)) {
      Serial.println("  -> indicate CCCD OK");
      return true;
    }
    Serial.println("  -> indicate subscribe FAILED");
  }
  return false;
}

static bool reportRefOf(NimBLERemoteCharacteristic* ch, uint8_t* outId, uint8_t* outType) {
  // Report Reference 0x2908: [reportId, reportType] type 1=Input 2=Output 3=Feature
  NimBLERemoteDescriptor* ref = ch->getDescriptor(UUID_REPORT_REF);
  if (!ref) return false;
  std::string v = ref->readValue();
  if (v.size() < 2) return false;
  *outId = (uint8_t)v[0];
  *outType = (uint8_t)v[1];
  Serial.printf("  reportRef id=%u type=%u (%s)\n",
                *outId, *outType,
                *outType == 1 ? "Input" : *outType == 2 ? "Output" : *outType == 3 ? "Feature" : "?");
  return true;
}

static bool subscribeHidReports(NimBLEClient* client);
static bool connectToKeyboard();
static void startScan();
static void scheduleNextReconnect();

static void scheduleNextReconnect() {
  // Tour PLAY: never scan/spam — wait for next F1 window.
  if (gTour == TOUR_PLAY) {
    gReconnectAtMs = millis() + 60000;
    return;
  }
  // Mostly stay on direct connect; short scan every N fails as a fallback.
  if (gReconnectAttempt > 0 && (gReconnectAttempt % SCAN_EVERY_N_FAILS) == 0) {
    Serial.printf("[BLE] %d direct fails - quick %lus scan\n",
                  gReconnectAttempt, (unsigned long)SCAN_DURATION_S);
    gNeedReconnect = false;
    startScan();
    return;
  }
  gReconnectAtMs = millis() + RECONNECT_RETRY_MS;
  Serial.printf("[BLE] retry in %lums...\n", (unsigned long)RECONNECT_RETRY_MS);
}

static bool subscribeHidReports(NimBLEClient* client) {
  gBootKb = nullptr;
  gInputReports.clear();
  gKbReportHandle = 0;
  gConsumerReportHandle = 0;

  // Fresh discovery after bonding
  client->getServices(true);

  NimBLERemoteService* hid = client->getService(UUID_HID_SERVICE);
  if (!hid) {
    Serial.println("[HID] service 0x1812 not found");
    auto* svcs = client->getServices(false);
    if (svcs) {
      for (auto* s : *svcs) {
        Serial.printf("  svc %s\n", s->getUUID().toString().c_str());
      }
    }
    return false;
  }

  // Exit suspend if supported
  NimBLERemoteCharacteristic* cp = hid->getCharacteristic(UUID_HID_CONTROL_POINT);
  if (cp && cp->canWrite()) {
    uint8_t wake = 0x00;  // Exit Suspend
    if (cp->writeValue(&wake, 1, true)) {
      Serial.println("[HID] control point -> Exit Suspend");
    }
  }

  bool any = false;
  auto* chars = hid->getCharacteristics(true);
  if (!chars) return false;

  for (auto* ch : *chars) {
    NimBLEUUID uuid = ch->getUUID();

    if (uuid == UUID_HID_BOOT_KB_IN) {
      if (subscribeChar(ch, "BOOT_KB_IN")) {
        gBootKb = ch;
        any = true;
      }
      continue;
    }

    if (uuid == UUID_HID_REPORT) {
      uint8_t rid = 0, rtype = 0;
      bool haveRef = reportRefOf(ch, &rid, &rtype);
      // Subscribe Input reports; if no Report Ref, still try (some cheap boards omit it)
      if (haveRef && (rtype == 2 || rtype == 3)) {
        Serial.println("  skip Output/Feature report");
        continue;
      }
      if (subscribeChar(ch, "REPORT")) {
        gInputReports.push_back(ch);
        any = true;
        // Bind control paths: id1=keyboard, id2=consumer (this keyboard's layout)
        if (haveRef && rtype == 1) {
          if (rid == 1 && gKbReportHandle == 0) {
            gKbReportHandle = ch->getHandle();
            Serial.printf("  -> keyboard report handle=%u\n", gKbReportHandle);
          } else if (rid == 2 && gConsumerReportHandle == 0) {
            gConsumerReportHandle = ch->getHandle();
            Serial.printf("  -> consumer report handle=%u\n", gConsumerReportHandle);
          }
        }
      }
      continue;
    }

    // Skip battery under HID unless useful
    if (uuid == NimBLEUUID("2A33") || uuid == UUID_BATTERY_LEVEL) {
      Serial.printf("[HID] skip battery char %s\n", uuid.toString().c_str());
      continue;
    }
  }

  // Also try Boot char via direct get (some stacks list quirks)
  if (!gBootKb) {
    NimBLERemoteCharacteristic* boot = hid->getCharacteristic(UUID_HID_BOOT_KB_IN);
    if (boot && subscribeChar(boot, "BOOT_KB_IN(direct)")) {
      gBootKb = boot;
      any = true;
    }
  }

  // Protocol AFTER subscribe: Boot only if boot CCCD worked; else Report mode.
  // (Previously forced Boot while boot=no → silent key path.)
  NimBLERemoteCharacteristic* proto = hid->getCharacteristic(UUID_HID_PROTOCOL_MODE);
  if (proto && proto->canWrite()) {
    uint8_t mode = gBootKb ? 0 : 1;  // 0=Boot, 1=Report
    if (proto->writeValue(&mode, 1, true)) {
      Serial.printf("[HID] protocol mode -> %s(%u)\n",
                    gBootKb ? "Boot" : "Report", (unsigned)mode);
    }
  }

  // Fallback if Report Ref ids missing: first subscribed Input is keyboard.
  if (gKbReportHandle == 0 && !gInputReports.empty()) {
    gKbReportHandle = gInputReports[0]->getHandle();
    Serial.printf("[HID] fallback keyboard handle=%u\n", gKbReportHandle);
  }
  if (gConsumerReportHandle == 0 && gInputReports.size() >= 2) {
    gConsumerReportHandle = gInputReports[1]->getHandle();
    Serial.printf("[HID] fallback consumer handle=%u\n", gConsumerReportHandle);
  }

  Serial.printf("[HID] subscribed boot=%s inputReports=%u kbH=%u consH=%u\n",
                gBootKb ? "yes" : "no",
                (unsigned)gInputReports.size(),
                (unsigned)gKbReportHandle,
                (unsigned)gConsumerReportHandle);
  return any;
}

static bool connectToKeyboard() {
  if (!gHaveLastAddr) seedPreferredAddr();
  if (!gTarget && !gHaveLastAddr) return false;

  if (gTarget) {
    Serial.printf("[BLE] connecting to %s (%s) RSSI=%d\n",
                  gTarget->getName().c_str(),
                  gTarget->getAddress().toString().c_str(),
                  gTarget->getRSSI());
  } else {
    Serial.printf("[BLE] connecting to %s (%s)\n",
                  gLastName.empty() ? "?" : gLastName.c_str(),
                  gLastAddr.toString().c_str());
  }

  if (gClient) {
    if (gClient->isConnected()) gClient->disconnect();
    NimBLEDevice::deleteClient(gClient);
    gClient = nullptr;
    delay(100);
  }

  gClient = NimBLEDevice::createClient();
  gClient->setClientCallbacks(new ClientCallbacks(), true);
  // Tour F1 window: short timeout so one miss does not stall the arm.
  const uint32_t connTo = tourPlayBleWindowOpen() ? TOUR_F1_CONNECT_TIMEOUT_S
                                                   : CONNECT_TIMEOUT_S;
  gClient->setConnectTimeout(connTo);

  bool ok = false;
  // Prefer known MAC first (single-keyboard); scanned target is fallback.
  if (gHaveLastAddr) {
    ok = gClient->connect(gLastAddr, true);
    if (!ok) ok = gClient->connect(gLastAddr);
  }
  if (!ok && gTarget) {
    ok = gClient->connect(gTarget, true);
    if (!ok) ok = gClient->connect(gTarget);
  }
  if (!ok) {
    Serial.println("[BLE] connect failed");
    return false;
  }

  delay(100);

  if (!gClient->secureConnection()) {
    Serial.println("[BLE] secureConnection failed (continuing)");
  } else {
    Serial.println("[BLE] secureConnection OK");
  }

  delay(150);

  if (!subscribeHidReports(gClient)) {
    Serial.println("[HID] no report subscriptions - disconnect");
    gClient->disconnect();
    return false;
  }

  if (gTarget) {
    gLastAddr = gTarget->getAddress();
    gLastName = gTarget->getName();
    gHaveLastAddr = true;
  } else {
    seedPreferredAddr();
  }

  gConnected = true;
  gEverReady = true;
  gNeedReconnect = false;
  gReconnectAttempt = 0;
  gLastHeartbeatMs = millis();
  Serial.println("[BLE] ready - press keys now");
  Serial.println("[TIP] If still silent: power-cycle keyboard, keep it awake, retry");
  refreshDisplay();
  return true;
}

static void startScan() {
  Serial.println();
  Serial.printf("[SCAN] starting %lus scan (preferred %s)\n",
                (unsigned long)SCAN_DURATION_S, PREFERRED_ADDR);
  if (gTarget) {
    delete gTarget;
    gTarget = nullptr;
  }
  gBestScanScore = -10000;

  NimBLEScan* scan = NimBLEDevice::getScan();
  scan->stop();
  scan->clearResults();
  scan->setAdvertisedDeviceCallbacks(new AdvCallbacks(), false);
  scan->setActiveScan(true);
  scan->setInterval(45);
  scan->setWindow(30);
  scan->start(SCAN_DURATION_S, false);
  gLastScanMs = millis();
}

void setup() {
  WRITE_PERI_REG(RTC_CNTL_BROWN_OUT_REG, 0);

  Serial.begin(115200);
  delay(800);

  Serial.println();
  Serial.println("========================================");
  Serial.println("  ESP32 BLE KEYBOARD HOST");
  Serial.println("========================================");
  Serial.println("1) Put BLE keyboard in pairing mode");
  Serial.println("2) Wait for [SCAN] / connect");
  Serial.println("3) ESC=reassert+HOME slew | F1-F10 save | 0-9 recall | HOME | clear | tour | WWW stop");
  Serial.println("========================================");
#ifdef SERVO_SINGLE_PIN
  Serial.printf("SINGLE TEST: signal->GPIO%d | ext VCC + common GND\n", SERVO_SINGLE_PIN);
#else
  Serial.println("Pins: ax1=13 ax2=14 ax3=23 ax4=27 | ext 5-6V + common GND");
#endif

  setCpuFrequencyMhz(160);
  delay(200);

  // I2C first (pins 25/26) so Wire never steals GPIO21 after servo PWM attach
  Serial.printf("CSG-4S: SDA=GPIO%d SCL=GPIO%d (5V+common GND)\n",
                CSG_SDA_PIN, CSG_SCL_PIN);
  gCsg.begin();

  gMem.begin();
  // First PWM is absolute: start at last pose (if any), then slew to HOME.
  // Old begin() wrote 90° immediately → looked like "boot still too fast" and slew was a no-op.
  {
    int start[4];
    int home[4];
    gMem.getHome(home);
    if (gMem.loadLast(start)) {
      Serial.printf("[MEM] boot restore last %d,%d,%d,%d\n",
                    start[0], start[1], start[2], start[3]);
    } else {
      for (int i = 0; i < 4; i++) start[i] = home[i];
      Serial.println("[MEM] boot no last pose — start at HOME");
    }
    gServos.begin(start);
    delay(500);  // hold last pose before moving

    const int bootLevel = 5;
    const int savedLevel = getSpeedLevel();
    setSpeedLevel(bootLevel);
    gServos.gotoAngles(home);
    Serial.printf("[MEM] boot HOME slew -> %d,%d,%d,%d (speed %d)\n",
                  home[0], home[1], home[2], home[3], getSpeedLevel());
    const uint32_t t0 = millis();
    while (gServos.busy() && (millis() - t0) < 20000) {
      gServos.service();
      delay(10);
    }
    gServos.setAnglesImmediate(home);
    gMem.saveLast(home);
    setSpeedLevel(savedLevel);
    Serial.println("[MEM] boot HOME done");
  }
  gTourSlot = 0;
  refreshDisplay();

  NimBLEDevice::init("ESP32-HID-Host");
  NimBLEDevice::setPower(ESP_PWR_LVL_P3);
  // bond + MITM + secure connections — many keyboards need bonding for HID notify
  NimBLEDevice::setSecurityAuth(true, true, true);
  NimBLEDevice::setSecurityIOCap(BLE_HS_IO_NO_INPUT_OUTPUT);

  seedPreferredAddr();
  Serial.printf("[BLE] single keyboard %s - direct connect first\n", PREFERRED_ADDR);
  if (!connectToKeyboard()) {
    Serial.println("[BLE] boot direct connect failed - short scan");
    startScan();
  }
}

void loop() {
  NimBLEScan* scan = NimBLEDevice::getScan();

  // Direct reconnect to preferred/last MAC (single-keyboard fast path).
  // Tour PLAY: only inside F1's 2s window (avoids blocking the motion loop).
  if (gNeedReconnect && !gConnected && (int32_t)(millis() - gReconnectAtMs) >= 0) {
    if (!bleReconnectAllowedNow()) {
      // Defer until F1 window or tour leaves PLAY; keep flag for later.
      gReconnectAtMs = millis() + 500;
    } else {
      gReconnectAttempt++;
      Serial.printf("[BLE] reconnect attempt %d%s\n", gReconnectAttempt,
                    tourPlayBleWindowOpen() ? " (tour F1)" : "");
      if (connectToKeyboard()) {
        Serial.printf("[BLE] reconnect OK (attempt %d)\n", gReconnectAttempt);
        gTourBleUntilMs = 0;
      } else {
        Serial.printf("[BLE] reconnect attempt %d failed\n", gReconnectAttempt);
        if (gTour == TOUR_PLAY) {
          // One short try per F1; do not schedule scan/spam inside tour.
          if (!tourPlayBleWindowOpen()) {
            gReconnectAtMs = millis() + 60000;  // wait next F1
          } else {
            gReconnectAtMs = millis() + 200;  // maybe one more inside window
          }
        } else {
          scheduleNextReconnect();
        }
      }
    }
  }

  if (!gConnected && !gDoConnect && !gNeedReconnect && gLastScanMs != 0 &&
      !scan->isScanning() && (millis() - gLastScanMs > 100)) {
    if (gTour == TOUR_PLAY && !tourPlayBleWindowOpen()) {
      // Ignore scan completion during tour outside F1 window.
      gLastScanMs = 0;
    } else if (gTarget) {
      Serial.printf("[SCAN] selected: %s (%s)\n",
                    gTarget->getName().c_str(),
                    gTarget->getAddress().toString().c_str());
      gDoConnect = true;
    } else {
      Serial.println("[SCAN] none - back to preferred direct retry");
      seedPreferredAddr();
      gNeedReconnect = true;
      gReconnectAttempt = 0;
      gReconnectAtMs = millis();
    }
  }

  if (gDoConnect && !gConnected && !gNeedReconnect) {
    if (gTour == TOUR_PLAY && !tourPlayBleWindowOpen()) {
      gDoConnect = false;
    } else {
      gDoConnect = false;
      if (!connectToKeyboard()) {
        seedPreferredAddr();
        gNeedReconnect = true;
        gReconnectAttempt = 0;
        gReconnectAtMs = millis() + RECONNECT_RETRY_MS;
      }
    }
  }

  // Heartbeat log only — GATT POLL here blocked the servo loop and could
  // clear held keys (read as key-up) while the user was still holding.
  static bool gWasBusy = false;
  if (gConnected && gClient && gClient->isConnected()) {
    serviceHeldInputs();

    if (millis() - gLastHeartbeatMs > 5000) {
      gLastHeartbeatMs = millis();
      Serial.printf("[HB] connected, waiting for key notifies (boot=%s reports=%u)\n",
                    gBootKb ? "yes" : "no",
                    (unsigned)gInputReports.size());
    }
  } else {
    // Keep ramping to recall/tour targets even while BLE is down.
    if (gHeldAxes) clearHeldAxes();
    gServos.service();
    serviceTour();
  }
  // Remember pose when motion settles (used for gentle next boot)
  {
    const bool busy = gServos.busy();
    if (gWasBusy && !busy) {
      int deg[4];
      gServos.getAngles(deg);
      gMem.saveLast(deg);
    }
    gWasBusy = busy;
  }

  // Live-update axis pages while moving; status blinks LOSt when BLE down
  if (gCsg.ok()) {
    const uint32_t period =
        (!gConnected && gDispPage == DISP_STATUS) ? 250 : DISP_ANGLE_REFRESH_MS;
    if ((gDispPage != DISP_STATUS || !gConnected) &&
        (millis() - gLastDispMs >= period)) {
      gLastDispMs = millis();
      refreshDisplay();
    }
  }

  delay(20);
}
