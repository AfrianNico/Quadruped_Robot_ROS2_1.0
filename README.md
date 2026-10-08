# Quadruped Robot ROS 2

Workspace untuk robot quadruped (4 kaki, 8-DOF) yang menggabungkan
**penghindaran rintangan berbasis visi (ROS 2 + YOLO)** dengan
**firmware kontrol gerak (ESP32 + Dynamixel)**.

```
┌─────────────────────────────┐         ┌──────────────────────────┐
│  Orange Pi 5 Pro (SBC)      │         │  ESP32 (firmware)        │
│                             │  SPI /  │                          │
│  vision_avoidance_ros2 ─────┼─micro-──┼─▶ quadruped-robot-control│
│  YOLO → /quadruped/cmd_vision│  ROS   │  gait engine, 8x XL330   │
└─────────────────────────────┘         └──────────────────────────┘
```

---

## Struktur Repository

Repository ini adalah isi folder `src/` dari ROS 2 workspace (`ros2_ws`):

| Folder | Deskripsi |
|---|---|
| [`vision_avoidance_ros2/`](vision_avoidance_ros2/) | Paket ROS 2 (Python) — deteksi rintangan YOLO, estimasi jarak, dan logika perintah gerak. Publish ke `/quadruped/cmd_vision`. |
| [`quadruped-robot-control/`](quadruped-robot-control/) | Firmware ESP32 (PlatformIO/Arduino) — gait 8-DOF, input SPI/Serial/PS3, monitoring baterai PZEM-017. |

> Folder `build/`, `install/`, dan `log/` hasil colcon **tidak** di-track
> oleh repository ini (sesuai konvensi workspace ROS 2).

---

## Komponen

### 1. `vision_avoidance_ros2` — Perception & Decision (ROS 2)

Node ROS 2 untuk penghindaran rintangan vertikal & horizontal menggunakan
model YOLO (backend ONNX/RKNN). Output berupa perintah gerak: `maju`,
`mundur`, `kiri`, `kanan`, `merunduk`.

- Target: **Orange Pi 5 Pro**, Python 3.10, ROS 2 Humble/Jazzy
- Build & run:

```bash
cd ~/ros2_ws
colcon build --packages-select vision_avoidance_ros2
source install/setup.bash
ros2 launch vision_avoidance_ros2 vision_avoidance.launch.py
```

Dokumentasi lengkap: [`vision_avoidance_ros2/README.md`](vision_avoidance_ros2/README.md)

### 2. `quadruped-robot-control` — Motion Control (Firmware)

Firmware ESP32 untuk 8 servo Dynamixel XL330 (2 per kaki). Menerima
perintah dari USB Serial, SPI (dari SBC), atau gamepad PS3 Bluetooth,
dengan mode operasi AUTO/MANUAL.

- Target: **ESP32** (`esp32dev`), PlatformIO, framework Arduino
- Terintegrasi **micro-ROS** (`micro_ros_arduino v2.0.8-humble`)
- Build & upload:

```bash
cd quadruped-robot-control
pio run                 # build
pio run -t upload       # flash ke ESP32
pio device monitor      # serial monitor 115200
```

Dokumentasi lengkap: [`quadruped-robot-control/README.md`](quadruped-robot-control/README.md)

---

## Alur Sistem

1. Kamera pada Orange Pi menangkap citra lingkungan.
2. `vision_avoidance_ros2` menjalankan inferensi YOLO, mengklasifikasi
   rintangan (vertikal/horizontal), lalu memutuskan perintah gerak.
3. Perintah dikirim ke ESP32 (via SPI / micro-ROS).
4. Firmware ESP32 mengeksekusi gait (jalan, belok, duduk, berdiri) pada
   servo Dynamixel sambil memonitor baterai secara real-time.

---

## Catatan

- Model YOLO (`.onnx`) ada di `vision_avoidance_ros2/models/`.
  Untuk deployment di Orange Pi, konversi ke `.rknn` (lihat README paket).
- Konfigurasi kelas & threshold deteksi: `vision_avoidance_ros2/config/class_mapping.yaml`.
- Konfigurasi pin, servo, dan gait: `quadruped-robot-control/src/config/`.
