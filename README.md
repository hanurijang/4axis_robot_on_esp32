# ESP32 BLE 키보드 + MG996R 4축 + CSG-4S

개발 기록 (2026-08).  
ESP32가 **BLE HID 호스트**로 키보드를 받아 서보 4축·포즈 메모리·자동순회·7세그 상태표시를 수행합니다.

관련 폴더:

| 경로 | 역할 |
|------|------|
| `~/codespace/MCU/esp32_4axis_robot` | **본 프로젝트** (활성 펌웨어) |
| `~/codespace/robot_4axis` | 원본 로봇 펌웨어 **백업/복원만** |

---

## 1. 하드웨어

### 보드 / 시리얼

| 항목 | 값 |
|------|-----|
| MCU | ESP32-D0WD-V3 (보드: esp32dev) |
| USB-UART | CP210x → `/dev/ttyUSB0` 또는 `ttyUSB1` |
| ESP32 MAC (실기) | `c8:2e:18:68:a7:70` |
| 시리얼 보드레이트 | **115200** |
| 업로드 속도 | **115200** (`upload_speed`, USB 불안정 시) |
| CPU (런타임) | 160 MHz (`setCpuFrequencyMhz(160)`) |
| 브라운아웃 | 부팅 시 비활성 |

### BLE 키보드

| 항목 | 값 |
|------|-----|
| 장치명 예 | `BT KeyBoard` |
| 선호 MAC | `b8:90:13:e3:5c:48` (`PREFERRED_ADDR`) |
| 프로토콜 | **BLE HID만** (클래식 BT HID 미지원) |
| 스택 | NimBLE central + observer (advertising 없음) |
| 모드 | **단일 키보드** — 부팅 시 선호 MAC 직결 우선 |
| 재연결 | 끊김 후 **200ms**부터 직결 재시도(400ms 간격, connect timeout 5초). 8회 실패마다 3초 스캔. 포기 없음 |
| 스캔 우선 | 선호 MAC > 이름(keyboard) > HID 서비스 > Appearance |
| 본딩 | NVS 유지. 앱은 bond를 지우지 않음 |

### 전원 (권장)

| 대상 | 전원 | 비고 |
|------|------|------|
| ESP32 | USB 5V | 유지 |
| MG996R ×4 | **외장 5~6V** (고기전 전용 레일) | 보드 5V로 4축 구동 **금지**. 용량 여유 필요(5V 5A급 권장) |
| CSG-4S | **ESP 보드 5V** (또는 서보와 분리된 안정 5V) | 서보 VCC와 **직접 묶지 말 것** (서보 피크에 세그먼트 죽음) |
| GND | **전부 공통** | ESP · 서보 전원 · CSG |

서보 케이블 색: 주황=신호, 빨강=VCC, 황토/갈색=GND.

---

## 2. 핀맵 (전체)

### 서보 (PWM)

| 축 | GPIO | 방향키 (− / +) | HID Usage |
|----|------|----------------|-----------|
| **ax1** | **13** | Prev / Next | Consumer `0x00B6` / `0x00B5` |
| **ax2** | **14** | Vol+ (−비트 ax2-1) / Vol− (+비트 ax2-2) | `0x00E9` / `0x00EA` |
| **ax3** | **23** | RIGHT (−) / LEFT (+) | Keyboard `0x4F` / `0x50` |
| **ax4** | **27** | DOWN (−) / UP (+) | Keyboard `0x51` / `0x52` |

코드: `SERVO_PINS[] = {13, 14, 23, 27}` (`include/servo_axis.h`).  
이력: 구 ax3=GPIO21(기본 I2C SDA 충돌) → 18(PWM 무응답) → **23**.

### CSG-4S (I2C 7세그)

| 신호 | ESP32 GPIO |
|------|------------|
| SDA | **25** (`CSG_SDA_PIN`) |
| SCL | **26** (`CSG_SCL_PIN`) |
| GND | GND |
| +5V | ESP 보드 5V (권장) |

| 항목 | 값 |
|------|-----|
| 칩 | **SAA1064** (Comfile CSG-4S) |
| I2C 클럭 | 100 kHz |
| DIP 슬레이브 0~3 | 7-bit 주소 **`0x38` ~ `0x3B`** |

5V I2C ↔ ESP 3.3V: 레벨시프터 권장. 시리얼 `[CSG] OK addr=0x..` / `NOT FOUND`로 확인.  
**주의:** CSG I2C(25/26)는 **서보 attach보다 먼저** 초기화. (순서 바뀌면 축 PWM 이상 가능)

### 사용 핀 요약

| GPIO | 용도 |
|------|------|
| 13, 14, **23**, 27 | 서보 신호 |
| 25, 26 | CSG I2C |
| USB 시리얼 | 디버그/업로드 |
| ~~21, 18~~ | 미사용 (구 ax3 후보) |

---

## 3. 서보 파라미터

| 항목 | 값 |
|------|-----|
| 기종 | MG996R ×4 |
| 각도 범위 (축별) | ax1 **60~120**, ax2 **80~130**, ax3 **60~140**, ax4 **60~90** |
| 센터 / HOME | **90° × 4** (하드코딩, NVS 아님) |
| 펄스폭 | 500 ~ 2500 µs, 50 Hz |
| 조그/이송 틱 | 10 ms |
| 홀드 | 키 유지 시 연속 슬루 |

저장·리콜 각도도 축별 min/max로 클램프 (`clampServoDeg`).

### 소프트 리밋 · 홀드

- 조그가 축별 min/max에 닿으면 **그 방향 조그 중지**, `(limit)` 로그는 **1회**.
- 반대 방향 조그는 그대로 가능.
- 유휴 중에도 **약 200 ms마다 PWM 재출력**해 홀드 토크 유지 (토크 풀림 완화).
- 동일 HID 리포트가 연속으로 오면 로그·재적용을 건너뜀.

### 이송 속도 레벨 (1~9, 기본 5)

| 레벨 | °/s |
|------|-----|
| 1 | 12 |
| 2 | 20 |
| 3 | 30 |
| 4 | 40 |
| **5** | **50** (기본) |
| 6 | 70 |
| 7 | 90 |
| 8 | 110 |
| 9 | 140 |

`=` / `-` 변경은 **투어 PLAY 또는 PAUSE 중**에만 적용.

### 부팅 HOME

1. 서보 attach  
2. ~400 ms 대기 후 HOME PWM  
3. ~500 ms 후 HOME PWM 재적용  

### BLE 끊김과 서보

- 끊김 시 조그만 해제 (`stopJogOnly`). **리콜/투어 goto 타깃은 유지**하고 disconnected 중에도 `service()` 계속.  
- 예전 `stopAll()`은 goto까지 지워 “세그먼트만 바뀌고 팔 안 움직임”을 유발했음 → 수정됨.

---

## 4. 키 / 기능 맵

### 조그 (홀드 연속) — IDLE일 때만

| 키 | Usage | 동작 |
|----|-------|------|
| Prev Track | `0x00B6` | ax1 − |
| Next Track | `0x00B5` | ax1 + |
| Volume Up | `0x00E9` | ax2 − |
| Volume Down | `0x00EA` | ax2 + |
| LEFT | `0x50` | ax3 + |
| RIGHT | `0x4F` | ax3 − |
| UP | `0x52` | ax4 + |
| DOWN | `0x51` | ax4 − |

### 포즈 메모리 (NVS `poses`)

| 키 | Usage | 동작 |
|----|-------|------|
| **ESC** | `0x29` | **하드 초기화** (언제든): 투어/조그 중지 → PWM 재확인 → HOME(90×4) 즉시 스냅. **F1~F10 NVS는 유지** |
| **F1~F10** | `0x3A`~`0x43` | 현재 4축 → 슬롯 저장 (`s1`~`s10`) |
| **1~9 / 0** | `0x1E`~`0x27` | 슬롯 리콜 (0 = 슬롯10). 투어 중이면 무시, IDLE에서는 투어 중지 후 이동 |
| Consumer **HOME** | `0x0223` | 고정 HOME(90×4)으로 슬루 이동, 투어 중지 (투어 중 무시) |
| Consumer **CLEAR** | `0x0221` | F1~F10 NVS 전부 삭제 (투어 중 무시) |
| **Play/Pause** | `0x00CD` | 투어 play ↔ pause ↔ resume |
| **WWW / Browser** | `0x0196` | **투어 종료 → IDLE** |
| **`=`** | `0x2E` | 투어 중 속도 +1 |
| **`-`** | `0x2D` | 투어 중 속도 −1 |

### 자동 투어 (Tour)

| 상태 | 의미 | CSG 모드 |
|------|------|----------|
| IDLE | 수동 | `0` |
| PLAY | F1~F10 중 **저장된 슬롯만** 순회 | `1` |
| PAUSE | 정지, 재개 가능 | `2` |

**투어 PLAY/PAUSE 중 허용 입력만:**

1. `=` / `-` — 속도  
2. Play/Pause — 일시정지 ↔ 재개  
3. WWW (`0x0196`) — 투어 종료  
4. **ESC** — 하드 초기화(투어 종료 + HOME 스냅)  

조그·F저장·숫자리콜·HOME·CLEAR는 무시 (조그 때문에 투어가 PAUSE 되던 문제 방지).

기타:

- 슬롯 도착 후 **dwell 500 ms** 다음으로  
- 빈 슬롯 건너뜀 (`nextValidSlot`)  
- 저장 슬롯 없으면 play 불가  

---

## 5. CSG-4S 표시

형식 **`{슬롯}F{속도}{모드}`**

| 자리 | 내용 |
|------|------|
| 1 | 슬롯 `1`~`9`, 슬롯10은 **`0`**, 없으면 `-` |
| 2 | 고정 `F` |
| 3 | 속도 레벨 `1`~`9` |
| 4 | 모드 `0` IDLE / `1` PLAY / `2` PAUSE |

예: `3F51` = 슬롯3, 속도5, PLAY. `0F51` = 슬롯10, 속도5, PLAY.

---

## 6. PlatformIO 환경

| env | 소스 | 용도 |
|-----|------|------|
| `diag` | `diag_main.cpp` | USB/시리얼 점검 |
| **`ble_keyboard`** | `ble_keyboard_main.cpp` | **본 펌웨어** |
| `ble_servo_test` | 동일 + `-DSERVO_SINGLE_PIN=27` | GPIO27 단일 서보 점검 |
| `servo_test` | `servo_test_main.cpp` | 시리얼만, BLE 없음 |

의존성: `NimBLE-Arduino ^1.4.3`, `ESP32Servo ^3.0.5`, Wire, Preferences.

### 빌드 / 업로드 / 모니터

```bash
cd ~/codespace/MCU/esp32_4axis_robot
pio run -e ble_keyboard -t upload
pio device monitor -e ble_keyboard
```

포트 사용 중이면 모니터를 끄거나 `fuser -k /dev/ttyUSB0` 후 업로드.

개별 서보 점검:

```bash
pio run -e ble_servo_test -t upload
# 점검 서보 신호만 GPIO27 → 아무 축 키로 구동
# 복귀: pio run -e ble_keyboard -t upload
```

원본 로봇 FW 복원:

```bash
cd ~/codespace/robot_4axis
./scripts/restore_robot_firmware.sh
```

---

## 7. 소스 구조

```
esp32_4axis_robot/
├── platformio.ini
├── README.md
├── include/
│   ├── hid_keyboard.h        # 축/키 Usage 매핑, 리포트 파싱
│   ├── servo_axis.h          # 핀·축별 각도·속도 1~9·ServoBank
│   ├── pose_memory.h         # NVS F1~F10, HOME, Consumer Usage
│   └── csg4s.h               # SAA1064 / CSG-4S
└── src/
    ├── ble_keyboard_main.cpp # 메인: BLE + 서보 + 메모리 + 투어 + CSG
    ├── diag_main.cpp
    └── servo_test_main.cpp
```

---

## 8. 동작 요약 (사용자 절차)

1. 서보 **외장전원** + **GND 공통**, 신호 **13/14/23/27**  
2. CSG: **ESP 5V** + SDA25 / SCL26 (서보 VCC와 분리)  
3. `ble_keyboard` 업로드 → `[CSG]`, `[MEM] boot HOME`, 선호 MAC 직결/스캔  
4. `[BLE] ready` 후 조그·F1~F10 저장·0~9 리콜  
5. Play/Pause로 투어, `=`/`-` 속도, WWW로 투어 종료, HOME/CLEAR는 IDLE에서  

---

## 9. 알려진 이슈 / 메모

- 서보·CSG를 **같은 약한 5V**에 묶으면 투어 중 세그먼트가 죽을 수 있음 → VCC 분리 + GND 공통.  
- 투어 중 포즈마다 ~0.5초 멈춤은 **dwell** (정상). 이동 중 불규칙 멈춤·표시 꺼짐은 전원 쪽을 의심.  
- USB 업로드 실패: 케이블 재연결, 모니터 종료 후 재업로드, `upload_speed=115200`.  
- PC에 이미 페어링된 키보드는 해제 후 ESP에 연결.  
- ax3는 **GPIO23** (구 21/18 아님).

---

## 10. 실기 식별 값 (참고)

| 항목 | 값 |
|------|-----|
| ESP32 MAC | `c8:2e:18:68:a7:70` |
| 키보드 MAC | `b8:90:13:e3:5c:48` |
| 선호 키보드 이름 | BT KeyBoard 계열 |
