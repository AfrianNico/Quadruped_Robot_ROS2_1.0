# Quadruped Robot ROS 2

Repository ini berisi proyek robot quadruped berbasis ROS 2 yang menggabungkan:

- penghindaran rintangan berbasis visi menggunakan model YOLO
- kontrol gerak robot empat kaki berbasis ESP32
- integrasi perangkat lunak dan firmware dalam satu workspace

Tujuan utama dari repositori ini adalah membangun sistem navigasi dan kontrol robot quadruped yang dapat mendeteksi objek di sekitar, mengambil keputusan gerak, dan mengeksekusi gait pada servo Dynamixel secara real-time.

## Gambaran Sistem

```text
Kamera / sensor visual
        ↓
vision_avoidance_ros2
        ↓
/quadruped/cmd_vision
        ↓
ESP32 / quadruped-robot-control
        ↓
Servo Dynamixel XL330
        ↓
Gerak robot 4 kaki
```

Secara umum alur kerja sistem adalah:

1. Kamera pada Orange Pi / SBC menangkap frame lingkungan.
2. Paket `vision_avoidance_ros2` memproses citra menggunakan model YOLO dan menentukan keputusan gerak.
3. Perintah gerak dipublish ke topik ROS 2 dan dikirim ke ESP32.
4. Firmware `quadruped-robot-control` menjalankan gait dan mengendalikan servo robot.

## Struktur Repository

```text
Quadruped_Robot_ROS2_1.0/
├── README.md
├── .gitmodules
├── .gitignore
├── vision_avoidance_ros2/        # Paket ROS 2 untuk deteksi dan penghindaran rintangan
├── quadruped-robot-control/      # Firmware ESP32 untuk kontrol gerak robot
├── sllidar_ros2/                # Submodule LiDAR (opsional / dependency)
├── build/                        # Hasil build ROS 2 (dibuat secara lokal)
├── install/                     # Hasil install ROS 2 (dibuat secara lokal)
├── log/                         # Log build dan runtime (dibuat secara lokal)
└── src/                         # Biasanya berisi workspace ROS 2 packages
```

## Komponen Utama

### 1. `vision_avoidance_ros2`
Paket ROS 2 ini menangani:

- input citra dari kamera
- deteksi objek dengan YOLO
- klasifikasi rintangan vertikal / horizontal
- penentuan arah gerak berdasarkan hasil deteksi
- publish perintah gerak ke topik `/quadruped/cmd_vision`

Fungsi utamanya adalah memberikan keputusan navigasi robot agar dapat menghindari rintangan secara otomatis.

Dokumentasi detail: [`vision_avoidance_ros2/README.md`](vision_avoidance_ros2/README.md)

### 2. `quadruped-robot-control`
Folder ini berisi firmware ESP32 untuk robot quadruped dengan:

- 8 servo Dynamixel XL330 (2 per kaki)
- kontrol gait dan mode bergerak
- komunikasi serial / SPI / input gamepad
- monitoring baterai dan status robot
- penggunaan PlatformIO + Arduino framework

Dokumentasi detail: [`quadruped-robot-control/README.md`](quadruped-robot-control/README.md)

### 3. `sllidar_ros2`
Merupakan submodule yang dipakai untuk mendukung sensing jarak / lingkungan berbasis LiDAR, sesuai kebutuhan sistem robot secara keseluruhan.

## Teknologi yang Digunakan

- ROS 2
- Python 3
- OpenCV
- YOLO / model deteksi visual
- ESP32
- PlatformIO
- Dynamixel XL330
- Optional: LiDAR Slamtec / `sllidar_ros2`

## Prasyarat

### Perangkat Lunak
- ROS 2 (Humble / Jazzy tergantung setup lingkungan)
- Python 3.10+
- colcon
- PlatformIO
- Git

### Perangkat Keras
- Orange Pi / SBC Linux untuk sisi perception dan kontrol ROS 2
- ESP32 untuk firmware robot
- Kamera untuk deteksi visual
- Servo Dynamixel XL330
- Baterai 3S LiPo dan modul monitoring baterai
- (Opsional) LiDAR untuk pendukung sensor lingkungan

## Quick Start

### 1. Clone repository

```bash
git clone https://github.com/AfrianNico/Quadruped_Robot_ROS2_1.0.git
cd Quadruped_Robot_ROS2_1.0
```

### 2. Build paket ROS 2

```bash
cd ~/ros2_ws
cp -r /path/to/Quadruped_Robot_ROS2_1.0/vision_avoidance_ros2 src/
colcon build --packages-select vision_avoidance_ros2
source install/setup.bash
```

### 3. Jalankan node deteksi robot

```bash
ros2 launch vision_avoidance_ros2 vision_avoidance.launch.py
```

### 4. Build firmware ESP32

```bash
cd quadruped-robot-control
pio run
pio run -t upload
```

## Catatan Pengembangan

- Folder `build/`, `install/`, dan `log/` merupakan output build ROS 2 lokal dan biasanya tidak diikutsertakan dalam kontrol versi.
- Model YOLO dan konfigurasi deteksi berada di dalam package `vision_avoidance_ros2`.
- Konfigurasi kontrol dan gait untuk robot berada di `quadruped-robot-control/src/config/`.
- Repo ini merupakan kombinasi dari dua fokus utama: perception dan motion control.

## Status Proyek

Proyek ini masih dalam tahap pengembangan dan eksperimen robot quadruped dengan arsitektur ROS 2 + ESP32. Repository ini dibuat untuk mendokumentasikan, mengorganisir, dan memudahkan pengembangan sistem robot dalam satu workspace.

## Lisensi

Project ini bersifat internal untuk kebutuhan riset dan pengembangan robot quadruped. Silakan cek detail lisensi pada masing-masing paket jika akan digunakan untuk publikasi atau distribusi lanjutan.

---

Dibuat untuk pengembangan robot quadruped dengan pendekatan modular: vision-driven decision making di sisi ROS 2 dan motion control di sisi firmware ESP32.
