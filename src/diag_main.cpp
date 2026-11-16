/**
 * ESP32 board health check
 * Upload:  pio run -e diag -t upload
 * Monitor: pio device monitor -e diag
 */
#include <Arduino.h>

#ifndef LED_BUILTIN
#define LED_BUILTIN 2
#endif

void setup() {
  Serial.begin(115200);
  delay(400);

  pinMode(LED_BUILTIN, OUTPUT);

  Serial.println();
  Serial.println("========================================");
  Serial.println("  ESP32 DIAGNOSTIC");
  Serial.println("========================================");
  Serial.printf("Chip model : %s\n", ESP.getChipModel());
  Serial.printf("Revision   : %d\n", ESP.getChipRevision());
  Serial.printf("Cores      : %d\n", ESP.getChipCores());
  Serial.printf("CPU freq   : %u MHz\n", ESP.getCpuFreqMHz());
  Serial.printf("Flash size : %u bytes\n", ESP.getFlashChipSize());
  Serial.printf("Sketch     : %u bytes\n", ESP.getSketchSize());
  Serial.printf("Free heap  : %u bytes\n", ESP.getFreeHeap());
  Serial.printf("SDK        : %s\n", ESP.getSdkVersion());
  Serial.printf("MAC        : %012llX\n", ESP.getEfuseMac());
  Serial.println("----------------------------------------");
  Serial.println("If this keeps printing 'alive', board/USB/serial are OK.");
  Serial.println("========================================");
  Serial.println();
}

void loop() {
  static uint32_t n = 0;
  digitalWrite(LED_BUILTIN, (n & 1) ? HIGH : LOW);
  Serial.printf("[alive] #%lu  uptime=%lus  free_heap=%u\n",
                (unsigned long)n,
                (unsigned long)(millis() / 1000UL),
                ESP.getFreeHeap());
  n++;
  delay(1000);
}
