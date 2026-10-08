# Quadruped Robot Control — Firmware ESP32

**Kontrol firmware untuk robot quadruped (4 kaki) dengan 8 derajat kebebasan (8-DOF) berbasis ESP32 dan aktuator Dynamixel XL330.**

Firmware ini mengendalikan robot berjalan berkaki empat yang dapat berjalan maju/mundur, berbelok kiri/kanan, berdiri, dan duduk. Robot menerima perintah dari berbagai sumber input (USB Serial, SPI dari Orange Pi, dan PS3 Bluetooth) serta dilengkapi sistem monitoring baterai real-time menggunakan sensor PZEM-017.

---

## Daftar Isi

- [Deskripsi Proyek](#deskripsi-proyek)
- [External Link](#external-link)
- [Fitur](#fitur)
- [Tech Stack](#tech-stack)
- [Arsitektur Perangkat Keras](#arsitektur-perangkat-keras)
- [Arsitektur Perangkat Lunak](#arsitektur-perangkat-lunak)
- [Prasyarat](#prasyarat)
- [Instalasi & Build](#instalasi--build)
- [Konfigurasi](#konfigurasi)
- [Cara Menjalankan](#cara-menjalankan)
- [Dokumentasi API / Protokol](#dokumentasi-api-protokol)
- [Struktur Proyek](#struktur-proyek)
- [Testing](#testing)
- [Lisensi](#lisensi)

---

## Deskripsi Proyek

Proyek ini adalah **firmware ESP32** (Arduino Framework) untuk robot quadruped berkaki empat yang memiliki **8 servo Dynamixel XL330** (2 servo per kaki: hip + knee). Robot ini dirancang untuk menerima perintah dari tiga sumber input:

| Sumber Input    | Interface      | Mode Operasi | Kecepatan Data          |
|-----------------|----------------|--------------|-------------------------|
| USB Serial      | UART0          | MANUAL       | Command-driven          |
| SPI Slave       | VSPI_HOST      | AUTO         | ~20 Hz (50 ms/transaksi)|
| PS3 Bluetooth   | Ps3Controller  | MANUAL       | ~100 Hz (notifikasi)    |

Robot mendukung **dua mode operasi**:
- **AUTO** — perintah berasal dari Orange Pi (atau SBC Linux) melalui SPI, biasanya untuk navigasi berbasis visi.
- **MANUAL** — perintah berasal dari USB Serial atau PS3 Bluetooth gamepad.

Sistem monitoring baterai berjalan sebagai **tugas FreeRTOS dedikasi** pada core 1 dengan frekuensi 2 Hz, menyediakan data tegangan, arus, daya, dan State of Charge (SOC) secara real-time.

---

## External Link

- [Robot Server Repository](https://github.com/vortex-byte/quadruped-robot-server)
- [Robot GUI Repository](https://github.com/vortex-byte/quadruped-robot-gui)

---

## Fitur

### Kontrol Multi-Sumber
- **USB Serial (UART0)** — CLI berbasis karakter tunggal (A/B/C/D) dan perintah mode (AUTO/MANUAL)
- **SPI Slave** — Komunikasi dengan Orange Pi/Linux SBC untuk perintah berbasis visi, paket 32-byte full-duplex dengan CRC-16/MODBUS
- **PS3 Bluetooth** — Gamepad nirkabel dengan analog stick untuk kontrol continuous 2-axis (maju + belok simultan)

### Gait (Cara Berjalan)
- **Blended trot gait 8 fase** — perpaduan continuous forward + turn dalam satu gerakan
- **Keyframe interpolation** — transisi halus untuk gerakan berdiri (*stand up*) dan duduk (*sit down*) dengan easing *easeInOutCubic*
- **Adaptive phase timing** — opsional, membaca status *Moving* servo untuk mempercepat transisi fase

### Monitoring Baterai
- Sensor **PZEM-017** (Modbus RTU) untuk pengukuran tegangan, arus, daya, dan energi
- **Voltage-based SOC lookup** saat cold start (tabel 13 titik untuk 3S LiPo)
- **Coulomb counting** (integrasi Ah) setelah startup
- **Persistensi NVS** setiap 30 detik — SOC tersimpan meski robot dimatikan

### Telemetri
- Paket **14-byte binary** (header 0xAA55 + tegangan + arus + SOC + kode perilaku + CRC-16) dikirim ke Orange Pi setiap transaksi SPI
- **CRC-16/MODBUS** untuk integritas data

### Fitur Keamanan
- **PS3 disconnect timeout** (2 detik) → robot berhenti otomatis
- **SPI watchdog timeout** (5 detik) → robot berhenti jika Orange Pi tidak merespon
- **Saturation limits** pada posisi servo (mencegah kerusakan mekanik)
- **Monitoring loop time** — peringatan jika loop > 50 ms

### Logging
- **Serial logging** dengan 4 level (Debug/Info/Warn/Error)
- **WiFi UDP logging** opsional — kirim log ke IP:port tertentu

---

## Tech Stack

| Kategori            | Teknologi                                           |
|---------------------|-----------------------------------------------------|
| **Bahasa**          | C++ (C++17)                                         |
| **Framework**       | Arduino (ESP32 Arduino Core)                        |
| **Build System**    | PlatformIO                                          |
| **RTOS**            | FreeRTOS (bawaan ESP32 Arduino Core)                |
| **Target Board**    | ESP32-DevKit (espressif32)                          |
| **Protokol Komunikasi** | Dynamixel Protocol 2.0 (UART/RS485), Modbus RTU, SPI Slave |
| **Bluetooth**       | PS3 Controller Host (library `jvpernis/PS3 Controller Host`) |
| **Sensor Baterai**  | PZEM-017 DC Energy Meter via Modbus (library `4-20ma/ModbusMaster`) |
| **Storage**         | ESP32 NVS (Preferences library)                     |
| **Unit Testing**    | Unity Test Framework (PlatformIO native)            |

### Dependencies (PlatformIO Libraries)

| Library                              | Versi    | Kegunaan                         |
|--------------------------------------|----------|----------------------------------|
| `jvpernis/PS3 Controller Host`       | `^1.1.0` | Komunikasi PS3 Bluetooth gamepad |
| `4-20ma/ModbusMaster`                | `^2.0.1` | Modbus RTU untuk PZEM-017        |

---

## Arsitektur Perangkat Keras

### GPIO Pin Mapping

| Sinyal              | GPIO ESP32 | Terhubung Ke                           |
|---------------------|------------|----------------------------------------|
| **Dynamixel Bus**   |            |                                        |
| `DXL_DIR_PIN`       | GPIO 2     | RS485 direction control (half-duplex)  |
| `DXL_UART_RX`       | GPIO 16    | Dynamixel UART RX                      |
| `DXL_UART_TX`       | GPIO 17    | Dynamixel UART TX                      |
| **PZEM-017 (Modbus)** |          |                                        |
| `PZEM_RX`           | GPIO 26    | PZEM-017 Modbus UART RX                |
| `PZEM_TX`           | GPIO 25    | PZEM-017 Modbus UART TX                |
| **SPI Slave (Orange Pi)** |        |                                        |
| `MOSI`              | GPIO 23    | Master Out Slave In                    |
| `MISO`              | GPIO 19    | Master In Slave Out                    |
| `SCLK`              | GPIO 18    | Serial Clock                           |
| `CS`                | GPIO 5     | Chip Select (active low)               |

*Didefinisikan di: `src/config/pin_config.h`*

### Alokasi Serial Port

| UART    | Baud Rate  | Kegunaan                         |
|---------|------------|----------------------------------|
| Serial (UART0) | 115200 | USB serial logging + CLI commands |
| Serial1 | 9600       | PZEM-017 Modbus RTU              |
| Serial2 | 1,000,000  | Dynamixel Protocol 2.0 bus       |

### Konfigurasi Servo Dynamixel

8 servo disusun sebagai 4 kaki × 2 sendi:

| Servo ID | Sendi | Kaki                  |
|----------|-------|-----------------------|
| 1        | Hip   | Front-Right (FR)      |
| 2        | Knee  | Front-Right (FR)      |
| 3        | Hip   | Front-Left (FL)       |
| 4        | Knee  | Front-Left (FL)       |
| 5        | Hip   | Back-Left (BL)        |
| 6        | Knee  | Back-Left (BL)        |
| 7        | Hip   | Back-Right (BR)       |
| 8        | Knee  | Back-Right (BR)       |

Masing-masing servo knee memiliki **center offset** untuk kompensasi variasi perakitan mekanik:

| Servo        | Center Offset |
|--------------|---------------|
| Knee FR (ID 2) | 2000        |
| Knee FL (ID 4) | 2200        |
| Knee BL (ID 6) | 2100        |
| Knee BR (ID 8) | 2300        |

### Spesifikasi Baterai

| Parameter            | Nilai    |
|----------------------|----------|
| Tipe                 | 3S LiPo  |
| Tegangan Nominal     | 11.1 V   |
| Tegangan Penuh       | 12.6 V   |
| Tegangan Kosong      | 9.0 V    |
| Kapasitas            | 2.2 Ah   |

---

## Arsitektur Perangkat Lunak

Proyek ini menggunakan **Clean Architecture** yang ketat dengan pemisahan lapisan yang jelas:

```
┌─────────────────────────────────────────────────────┐
│                   main.cpp                          │
│  (Dependency Injection — wiring semua komponen)     │
├─────────────────────────────────────────────────────┤
│                    Service                          │
│  ┌─────────────────┐  ┌──────────────────────────┐  │
│  │   GaitService   │  │     BatteryService       │  │
│  │ (orchestrasi    │  │ (SOC estimation,         │  │
│  │  gait, state    │  │  coulomb counting,       │  │
│  │  machine,       │  │  NVS persistence)        │  │
│  │  keyframes)     │  │                          │  │
│  └────────┬────────┘  └───────────┬──────────────┘  │
├───────────┴───────────────────────┴────────────────┤
│                    Domain                           │
│  ┌──────────┐ ┌──────────┐ ┌──────────────────┐   │
│  │ Command  │ │ Mode     │ │ MotionVector     │   │
│  │ ICommand │ │ IDynamixelBus │ IBatterySensor│   │
│  │ ILogger  │ │ IStorageRepository              │   │
│  └──────────┘ └──────────┘ └──────────────────┘   │
├────────────────────────────────────────────────────┤
│                    Driver                           │
│  ┌──────────┐ ┌──────────┐ ┌──────────────────┐   │
│  │Dynamixel │ │Pzem017   │ │CommandInputSerial │   │
│  │BusDriver │ │Driver    │ │CommandInputSpi    │   │
│  │NvsStorage│ │Pzem017   │ │CommandInputPs3    │   │
│  │Driver    │ │Driver    │ │                   │   │
│  └──────────┘ └──────────┘ └──────────────────┘   │
├────────────────────────────────────────────────────┤
│                  Platform                           │
│  ┌──────────┐ ┌──────────┐ ┌──────────────────┐   │
│  │Serial    │ │Battery   │ │ModeSharedState   │   │
│  │Logger    │ │SharedState│ │(FreeRTOS mutex)  │   │
│  └──────────┘ └──────────┘ └──────────────────┘   │
└────────────────────────────────────────────────────┘
```

### Aturan Dependency
- **`domain/`** — interface murni dan tipe data, **zero hardware dependencies**
- **`service/`** — logika bisnis, hanya bergantung pada interface di `domain/`
- **`driver/`** — implementasi hardware dari interface di `domain/`
- **`platform/`** — wrapper khusus platform (FreeRTOS mutex, logging)
- **`main.cpp`** — satu-satunya titik wiring menggunakan dependency injection

### FreeRTOS Task Model

| Task              | Core | Stack  | Prioritas | Fungsi                              |
|-------------------|------|--------|-----------|-------------------------------------|
| `loop()` (Arduino)| 0    | default| 1         | Input polling, gait, telemetri      |
| `battery`         | 1    | 4096   | 1         | Baca PZEM-017 tiap 500 ms (2 Hz)    |

Komunikasi antar task dilakukan via `BatterySharedState` yang dilindungi FreeRTOS mutex.

---

## Prasyarat

### Software
- **PlatformIO IDE** (VS Code extension) atau **PlatformIO CLI**
- **Git** (opsional, untuk version control)
- **Python 3** + `pip` (untuk script Orange Pi)
  - Library: `spidev` (`pip3 install spidev`)

### Hardware
| Komponen               | Jumlah | Keterangan                             |
|------------------------|--------|----------------------------------------|
| ESP32-DevKit board     | 1      | Mikrokontroler utama                   |
| Dynamixel XL330        | 8      | Smart servo (hip × 4 + knee × 4)      |
| RS485 half-duplex converter | 1  | Untuk bus Dynamixel                    |
| PZEM-017 DC Energy Meter | 1    | Sensor monitoring baterai              |
| Orange Pi / Linux SBC  | 1      | Opsional, untuk input SPI / visi       |
| PS3 Controller         | 1      | Opsional, untuk kontrol nirkabel       |
| Baterai 3S LiPo        | 1      | 11.1 V, 2.2 Ah                         |
| Adaptor USB            | 1      | Untuk flashing dan serial monitor      |

### Wiring
Hubungkan ESP32 dengan komponen sesuai tabel GPIO pin mapping di atas. Pastikan:
- Tegangan logic ESP32 dan Orange Pi sama-sama 3.3 V
- Koneksi RS485 untuk Dynamixel menggunakan half-duplex
- PZEM-017 terhubung via Modbus RTU (Serial1)

---

## Instalasi & Build

### 1. Clone Repository
```bash
git clone <repository-url> quadruped-robot-control
cd quadruped-robot-control
```

### 2. Install PlatformIO
Jika belum memiliki PlatformIO:

**VS Code:**
- Install ekstensi **PlatformIO IDE** dari marketplace

**CLI:**
```bash
pip install platformio
```

### 3. Build Firmware
```bash
# Build untuk ESP32 (production)
pio run

# Build native tests (PC)
pio run -e native
```

PlatformIO akan secara otomatis mengunduh semua dependencies yang diperlukan.

### 4. Upload ke ESP32
```bash
pio run --target upload
```

### 5. Serial Monitor
```bash
pio device monitor --baud 115200
```

---

## Konfigurasi

Semua konfigurasi berada di folder `src/config/` sebagai **compile-time constants** (header file C++).

### `system_config.h`
Konfigurasi utama sistem:

| Makro/Konstanta             | Default    | Keterangan                                      |
|-----------------------------|------------|-------------------------------------------------|
| `APP_DEBUG`                 | `0` (off)  | Aktifkan debug logging (1 = on)                 |
| `ENABLE_PS3_INPUT`          | `1` (on)   | Aktifkan input PS3 Bluetooth                    |
| `ENABLE_SERVO_READBACK`     | `0` (off)  | Adaptive phase timing via servo readback        |
| `ENABLE_WIFI_LOGGING`       | `0` (off)  | Forward log via WiFi UDP                        |
| `DEFAULT_MODE_AUTO`         | `false`    | Mode default saat startup (false = MANUAL)      |
| `DXL_BAUDRATE`              | 1000000    | Baud rate bus Dynamixel                         |
| `CMD_BAUDRATE`              | 115200     | Baud rate USB Serial                            |
| `BT_DEVICE_NAME`            | `ESP32_Robodog` | Nama device Bluetooth                     |
| `PS3_MAC_ADDRESS`           | `02:00:00:00:00:00` | MAC address controller PS3           |
| `PS3_DISCONNECT_TIMEOUT_MS` | 2000       | Timeout deteksi disconnect PS3 (ms)             |
| `PS3_DEADZONE`              | 20         | Deadzone analog stick PS3                       |
| `FORWARD_SATURATION_MAX`    | 127        | Saturasi maksimum input forward                 |
| `TURN_SATURATION_MAX`       | 127        | Saturasi maksimum input turn                    |

### `pin_config.h`
Mapping pin GPIO ESP32 (lihat tabel di atas).

### `servo_config.h`
Konfigurasi servo Dynamixel:
- ID servo (1–8)
- Center offset per-knee
- Profile acceleration/velocity
- Address control table Dynamixel

### `gait_config.h`
Parameter gait (cara berjalan):
- `INIT_POSE_*` — posisi netral masing-masing kaki
- `HIP_SATURATION_LIMIT` — 850 (clamp posisi hip)
- `KNEE_SATURATION_LIMIT` — 1200 (clamp posisi knee)
- `MAX_HIP_SWING` — 250 (amplitudo ayunan hip)
- `MAX_TURN_OFFSET` — 500 (diferensial stride untuk belok)
- `KNEE_LIFT_AMOUNT` — 200 (tinggi angkat knee)
- `KNEE_DIRECTION[4]` — arah tekuk knee per kaki
- `PHASE_DELAY_TABLE[8]` — delay statis per fase gait
- `STAND_UP_KEYFRAMES[5]` — keyframe gerakan berdiri
- `SIT_DOWN_KEYFRAMES[5]` — keyframe gerakan duduk
- `STAND_SIT_STEP_MS` — 20 ms (interval mikro-step interpolasi)

### `battery_config.h`
Konfigurasi baterai 3S LiPo:
- `BATTERY_FULL_VOLTAGE` — 12.6 V
- `BATTERY_EMPTY_VOLTAGE` — 9.0 V
- `BATTERY_CAPACITY_AH` — 2.2 Ah
- `SOC_PERSIST_INTERVAL_S` — 30 detik (interval simpan NVS)
- `VOLTAGE_SOC_TABLE` — tabel lookup 13 titik tegangan→SOC
- `BATTERY_READ_INTERVAL_MS` — 500 ms (2 Hz)

---

## Cara Menjalankan

### 1. Flash Firmware ke ESP32
```bash
pio run --target upload
```

### 2. Monitoring Serial
```bash
pio device monitor --baud 115200
```

Setelah terhubung, Anda akan melihat log inisialisasi dan dapat mengirim perintah.

### 3. Kontrol via USB Serial
Kirim karakter tunggal melalui serial monitor:

| Input    | Aksi                | Mode Diperlukan |
|----------|---------------------|-----------------|
| `A`      | Maju (FORWARD)      | MANUAL          |
| `B`      | Mundur (BACKWARD)   | MANUAL          |
| `C`      | Belok Kiri (TURN_LEFT) | MANUAL       |
| `D`      | Belok Kanan (TURN_RIGHT) | MANUAL      |
| `AUTO`   | Ganti mode ke AUTO   | Any             |
| `MANUAL` | Ganti mode ke MANUAL | Any             |

### 4. Kontrol via PS3 Bluetooth
1. Pastikan `ENABLE_PS3_INPUT = 1` di `system_config.h`
2. Set `PS3_MAC_ADDRESS` sesuai MAC controller Anda
3. Flash firmware dan pairing PS3
4. Robot akan merespon analog stick kiri (kontrol continuous forward + turn)
5. Robot berhenti otomatis jika PS3 disconnect > 2 detik

### 5. Kontrol via SPI (Orange Pi)
1. Aktifkan SPI overlay di Orange Pi (`armbian-config` → System → Hardware → enable `spi0`)
2. Reboot, pastikan `/dev/spidev0.0` tersedia
3. Install Python library: `pip3 install spidev`
4. Jalankan script kontrol (contoh di `docs/spi-communication.md`):
```bash
sudo python3 remote_control_server.py
```
5. Kirim perintah:
   - `"c:forward"` — maju
   - `"c:backward"` — mundur
   - `"c:right"` — belok kanan
   - `"c:left"` — belok kiri
   - `"c:stop"` — berhenti
   - `"m:auto"` — ganti mode AUTO
   - `"m:manual"` — ganti mode MANUAL

---

## Dokumentasi API / Protokol

### Protokol SPI (ESP32 ↔ Orange Pi)

**Mode SPI:** Mode 0 (CPOL=0, CPHA=0)  
**Frekuensi maksimum:** ≤ 10 MHz  
**Ukuran frame:** 32 byte (full-duplex)  

#### RX Direction (Orange Pi → ESP32)
String ASCII null-terminated (32 byte, dipadding dengan 0):

| String           | Command Enum              | Aksi           |
|------------------|---------------------------|----------------|
| `"c:forward"`    | `Command::FORWARD` (1)    | Maju           |
| `"c:backward"`   | `Command::BACKWARD` (2)   | Mundur         |
| `"c:right"`      | `Command::TURN_RIGHT` (4) | Belok kanan    |
| `"c:left"`       | `Command::TURN_LEFT` (3)  | Belok kiri     |
| `"c:stop"`       | `Command::NONE` (0)       | Berhenti       |
| `"m:auto"`       | —                         | Ganti ke AUTO  |
| `"m:manual"`     | —                         | Ganti ke MANUAL|

#### TX Direction (ESP32 → Orange Pi)
Paket binary **14-byte** (`TelemetryPacket`) + 18 byte padding zero:

| Offset | Size | Field           | Tipe     | Keterangan                            |
|--------|------|-----------------|----------|---------------------------------------|
| 0      | 2    | `header`        | uint8[]  | Start-of-frame marker: `0xAA 0x55`   |
| 2      | 4    | `voltage`       | float32  | Tegangan baterai (V), little-endian   |
| 6      | 4    | `current`       | float32  | Arus baterai (A), little-endian       |
| 10     | 1    | `socPercent`    | uint8    | State of Charge 0–100%               |
| 11     | 1    | `robotBehavior` | uint8    | Kode perilaku (lihat tabel di bawah)  |
| 12     | 2    | `crc`           | uint16   | CRC-16/MODBUS atas bytes[0..11]      |

**Total telemetri:** 14 byte + 18 byte padding = **32 byte total**.

#### Kode Perilaku (robotBehavior)

| Nilai | Makna              |
|-------|--------------------|
| 0     | IDLE / SITTING / STANDING_UP / SITTING_DOWN |
| 1     | FORWARD (maju)     |
| 2     | BACKWARD (mundur)  |
| 3     | TURN_LEFT (belok kiri) |
| 4     | TURN_RIGHT (belok kanan) |

#### Algoritma CRC-16/MODBUS
- **Polynomial:** 0xA001 (reverse dari 0x8005)
- **Initial value:** 0xFFFF
- **Final XOR:** 0x0000

**Python:**
```python
def crc16_modbus(data: bytes) -> int:
    crc = 0xFFFF
    for byte in data:
        crc ^= byte
        for _ in range(8):
            if crc & 0x0001:
                crc = (crc >> 1) ^ 0xA001
            else:
                crc >>= 1
    return ((crc >> 8) | ((crc & 0xFF) << 8)) & 0xFFFF
```

### Command Enum

```cpp
enum class Command : uint8_t {
    NONE       = 0,  // Berhenti / idle
    FORWARD    = 1,  // Maju
    BACKWARD   = 2,  // Mundur
    TURN_LEFT  = 3,  // Belok kiri
    TURN_RIGHT = 4,  // Belok kanan
    STAND_UP   = 5,  // Berdiri
    STAND_DOWN = 6,  // Duduk
};
```

### Mode Enum

```cpp
enum class Mode : uint8_t {
    AUTO   = 0,  // Input dari SPI/visi
    MANUAL = 1,  // Input dari Serial/PS3
};
```

### MotionVector (Continuous 2-Axis)
```cpp
struct MotionVector {
    int16_t forward;       // ±MAX_HIP_SWING (±250), negatif = mundur
    int16_t turn;          // ±MAX_TURN_OFFSET (±500), negatif = kiri
    Command buttonCommand; // NONE, STAND_UP, atau STAND_DOWN
};
```

### TelemetryPacket (Packed, 14 byte)
```cpp
#pragma pack(push, 1)
struct TelemetryPacket {
    uint8_t  header[2];      // {0xAA, 0x55}
    float    voltage;        // Tegangan (V)
    float    current;        // Arus (A)
    uint8_t  socPercent;     // SOC 0–100%
    uint8_t  robotBehavior;  // Kode perilaku
    uint16_t crc;            // CRC-16/MODBUS
};
#pragma pack(pop)
```

---

## Struktur Proyek

```
Quadruped Robot Control/
├── platformio.ini              # Konfigurasi PlatformIO
├── .gitignore
├── src/
│   ├── main.cpp                # Entry point: setup(), loop(), FreeRTOS battery task
│   ├── config/
│   │   ├── pin_config.h        # Mapping GPIO pins
│   │   ├── system_config.h     # Konstanta sistem, feature flags
│   │   ├── servo_config.h      # ID servo, address control table Dynamixel
│   │   ├── gait_config.h       # Parameter gait, keyframes
│   │   └── battery_config.h    # Tabel SOC, threshold baterai
│   ├── domain/                 # Interface murni & tipe data (zero HW deps)
│   │   ├── Command.h           # Enum perintah robot
│   │   ├── Mode.h              # Enum mode operasi
│   │   ├── MotionVector.h      # Struct kontrol continuous 2-axis
│   │   ├── BatteryData.h       # Struct PzemReading & BatteryState
│   │   ├── TelemetryPacket.h   # Struct paket telemetri 14-byte
│   │   ├── ICommandInput.h     # Interface input sources
│   │   ├── IDynamixelBus.h     # Interface bus Dynamixel
│   │   ├── IBatterySensor.h    # Interface sensor baterai
│   │   ├── ILogger.h           # Interface logging
│   │   └── IStorageRepository.h# Interface NVS persistence
│   ├── service/                # Logika bisnis
│   │   ├── GaitService.h/.cpp  # Orchestrasi gait, state machine
│   │   └── BatteryService.h/.cpp # Estimasi SOC, coulomb counting
│   ├── driver/                 # Implementasi hardware
│   │   ├── DynamixelBusDriver.h/.cpp    # Dynamixel Protocol 2.0
│   │   ├── CommandInputSerial.h         # USB serial CLI (template)
│   │   ├── CommandInputSpi.h/.cpp       # SPI slave dari Orange Pi
│   │   ├── CommandInputPs3.h/.cpp       # PS3 Bluetooth gamepad
│   │   ├── NvsStorageDriver.h/.cpp      # ESP32 NVS (Preferences)
│   │   └── Pzem017Driver.h/.cpp         # PZEM-017 Modbus RTU
│   ├── platform/               # Wrapper khusus platform
│   │   ├── SerialLogger.h      # ILogger via HardwareSerial + WiFi UDP
│   │   ├── BatterySharedState.h/.cpp  # Thread-safe BatteryState (FreeRTOS mutex)
│   │   └── ModeSharedState.h/.cpp     # Thread-safe Mode sharing
│   └── utils/
│       └── Easing.h            # easeInOutCubic untuk stand/sit
├── docs/
│   ├── SYSTEM_ARCHITECTURE.md  # Dokumentasi arsitektur lengkap
│   ├── PROJECT_WORKFLOW.md     # Workflow & konvensi proyek
│   ├── spi-communication.md    # Protokol SPI + code Python Orange Pi
│   └── ESP32_UPDATE_GUIDE.md   # Panduan update firmware ESP32
├── test/                       # Unit tests (native PC)
│   ├── mocks/
│   │   ├── MockDynamixelBus.h
│   │   ├── MockCommandInput.h
│   │   ├── MockLogger.h
│   │   └── MockSerial.h
│   ├── test_support/           # Stub Arduino untuk native testing
│   ├── test_domain/
│   │   └── test_command_enum.cpp
│   ├── test_driver/
│   │   └── test_command_input_serial.cpp
│   ├── test_mode_enum/
│   │   └── test_mode_enum.cpp
│   └── test_service/
│       └── test_gait_service.cpp
├── lib/                        # PlatformIO private libraries (kosong)
│   └── README
└── include/                    # PlatformIO global headers (kosong)
    └── README
```

---

## Testing

Proyek ini menggunakan **Unity Test Framework** dengan environment native PC PlatformIO.

### Menjalankan Unit Tests
```bash
pio test -e native
```

### Struktur Test
- **`test/test_domain/`** — test untuk domain logic (Command enum, dll.)
- **`test/test_driver/`** — test untuk driver (serial input)
- **`test/test_service/`** — test untuk service (gait service)
- **`test/test_mode_enum/`** — test untuk mode enum
- **`test/mocks/`** — mock object untuk testing (MockDynamixelBus, MockSerial, dll.)
- **`test/test_support/`** — stub Arduino compatibility untuk native testing

**Catatan:** File stub di `test/test_support/config/gait_config.h` mungkin perlu diperbarui agar sesuai dengan implementasi 8-phase gait terbaru.

---

## Lisensi

Proyek ini adalah bagian dari Proyek Akhir dan dilindungi oleh hak cipta yang berlaku. Semua hak dilindungi undang-undang.

---

**Dibuat oleh** - Tim Proyek Akhir Robot Quadruped Angkatan 2022 **Politeknik Elektronika Negeri Surabaya (PENS)**