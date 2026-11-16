# Quadruped Robot ROS 2

Proyek robot quadruped 4 kaki yang menggabungkan **perception berbasis kamera dan YOLO**, paket **ROS 2**, serta **firmware kontrol gerak ESP32** untuk 8 servo Dynamixel XL330.

> **Penting:** repository ini berisi kode dan konfigurasi, bukan robot yang siap dijalankan tanpa setup. Anda memerlukan lingkungan ROS 2, firmware yang dikompilasi untuk board yang sesuai, perangkat keras, dan model YOLO yang kompatibel. Model YOLO tidak dijamin tersedia di repository; siapkan file model sendiri.

## Isi Repository

| Path | Fungsi |
|---|---|
| [`vision_avoidance_ros2/`](vision_avoidance_ros2/) | Paket ROS 2 Python untuk kamera, inferensi YOLO, klasifikasi rintangan, dan publish perintah ke `/quadruped/cmd_vision`. Mendukung backend ONNX dan RKNN sesuai platform. |
| [`quadruped-robot-control/`](quadruped-robot-control/) | Firmware PlatformIO/Arduino untuk ESP32, gait robot 8-DOF, input kontrol, dan monitoring baterai. |
| [`sllidar_ros2/`](sllidar_ros2/) | Git submodule driver LiDAR Slamtec; opsional jika setup Anda menggunakan LiDAR. |
| [`.gitmodules`](.gitmodules) | Konfigurasi submodule `sllidar_ros2`. |

## Cara Kerja Singkat

```text
Kamera → vision_avoidance_ros2 (YOLO) → /quadruped/cmd_vision
                                             ↓
                              integrasi/bridge ke kontrol ESP32
                                             ↓
                              Firmware → Dynamixel XL330
```

Node visi memublikasikan `std_msgs/String` pada `/quadruped/cmd_vision`. Firmware juga mendukung input seperti USB Serial dan SPI. **Topik ROS 2 tersebut tidak otomatis menjadi koneksi langsung ke firmware**; Anda perlu menyiapkan mekanisme integrasi/bridge yang sesuai dengan hardware dan implementasi Anda (misalnya aplikasi di SBC yang meneruskan perintah melalui SPI). Verifikasi pemetaan format perintah sebelum menghubungkan robot.

## Prasyarat

### Untuk membangun dan menjalankan paket visi

- Ubuntu dengan ROS 2 terpasang dan sudah disource di terminal (paket menargetkan ROS 2 Humble/Jazzy).
- Python 3; paket menargetkan Python 3.10.
- `colcon`, `rosdep`, kamera yang dapat diakses dari Linux, dan model YOLO hasil ekspor yang cocok.
- Untuk laptop/PC non-ARM: backend ONNX Runtime. Untuk Orange Pi 5 Pro: backend RKNN Lite yang sesuai dengan versi sistem dan runtime Rockchip Anda.

### Untuk firmware

- PlatformIO CLI atau ekstensi PlatformIO IDE di VS Code.
- ESP32 DevKit yang cocok dengan target `esp32dev` di konfigurasi proyek.
- Untuk menjalankan robot: Dynamixel XL330, rangkaian bus/RS485, catu daya dan perangkat pendukung sesuai wiring proyek.

> **Keselamatan:** build firmware tidak sama dengan validasi hardware. Sebelum menyalakan aktuator, periksa wiring, polaritas dan tegangan catu daya, ID servo, batas gerak, serta konfigurasi pin. Untuk pengujian awal, amankan robot agar gerakan yang tidak diharapkan tidak menyebabkan cedera atau kerusakan.

## 1. Clone Repository

Clone ke direktori `src` di ROS 2 workspace. Opsi `--recurse-submodules` juga mengambil source `sllidar_ros2`.

```bash
mkdir -p ~/ros2_ws/src
cd ~/ros2_ws/src
git clone --recurse-submodules https://github.com/AfrianNico/Quadruped_Robot_ROS2_1.0.git
```

Jika repository sudah terlanjur di-clone tanpa submodule, ambil submodule dengan:

```bash
cd ~/ros2_ws/src/Quadruped_Robot_ROS2_1.0
git submodule update --init --recursive
```

Submodule LiDAR bersifat opsional untuk menjalankan paket visi dan firmware. Jika tidak diperlukan, Anda tetap dapat melanjutkan tanpa menggunakannya.

## 2. Siapkan Paket Visi

Source ROS 2 sebelum menjalankan perintah `ros2` atau `colcon`. Ganti `humble` dengan distro yang memang terpasang, misalnya `jazzy`.

```bash
source /opt/ros/humble/setup.bash
sudo apt update
sudo apt install -y python3-colcon-common-extensions python3-rosdep \
  python3-opencv python3-numpy python3-yaml
```

Jika `rosdep` belum diinisialisasi pada sistem:

```bash
sudo rosdep init
rosdep update
```

Pasang dependency ROS dari workspace, lalu build paket visi:

```bash
cd ~/ros2_ws
rosdep install --from-paths src --ignore-src -r -y
colcon build --symlink-install --packages-up-to vision_avoidance_ros2
source install/setup.bash
```

### Siapkan model YOLO

Paket memerlukan model hasil ekspor yang kompatibel dengan pipeline-nya. Ikuti petunjuk ekspor, urutan kelas, dan konfigurasi pada [`vision_avoidance_ros2/README.md`](vision_avoidance_ros2/README.md). Letakkan model di lokasi yang Anda pilih, lalu sesuaikan konfigurasi kelas di `vision_avoidance_ros2/config/class_mapping.yaml` dengan kelas hasil training Anda.

- Laptop/PC non-ARM: siapkan file model ONNX dan dependency `onnxruntime` sesuai `vision_avoidance_ros2/requirements.txt`.
- Orange Pi 5 Pro: siapkan runtime dan file RKNN yang sesuai. Paket RKNN Lite biasanya perlu dipasang dari wheel yang cocok dengan platform; lihat dokumentasi Rockchip dan README paket.
- Argumen `model_path` pada perintah launch adalah path absolut ke model **tanpa ekstensi**. Contoh `/home/user/models/best` untuk file `best.onnx` atau `best.rknn`.

> Jangan mengasumsikan model tersedia hanya karena folder `models/` disebut dalam dokumentasi. Pastikan file model benar-benar ada dan dapat dibaca sebelum menjalankan node. Periksa juga lisensi dan izin penggunaan model.

## 3. Jalankan dan Verifikasi Paket Visi

Pastikan kamera terhubung, model tersedia, dan terminal telah menjalankan `source /opt/ros/humble/setup.bash` serta `source ~/ros2_ws/install/setup.bash`.

```bash
ros2 launch vision_avoidance_ros2 vision_avoidance.launch.py \
  model_path:=/home/$USER/models/best
```

Argumen berguna lainnya:

```bash
ros2 launch vision_avoidance_ros2 vision_avoidance.launch.py \
  model_path:=/home/$USER/models/best \
  camera_index:=0 \
  debug_view:=true \
  telemetry_enabled:=true
```

Lihat semua argumen dan default aktual pada [`vision_avoidance_ros2/launch/vision_avoidance.launch.py`](vision_avoidance_ros2/launch/vision_avoidance.launch.py). Untuk memeriksa bahwa node memublikasikan perintah:

```bash
ros2 topic list
ros2 topic echo /quadruped/cmd_vision
```

Jika kamera tidak terbuka, periksa `camera_index`, izin akses perangkat kamera, dan apakah kamera sedang digunakan aplikasi lain. Jika topic tidak muncul, pastikan node berhasil dimulai dan model/config dapat dimuat.

## 4. Build dan Upload Firmware ESP32

Firmware dibangun terpisah dari ROS 2. Perintah berikut menjalankan PlatformIO dari direktori firmware:

```bash
cd ~/ros2_ws/src/Quadruped_Robot_ROS2_1.0/quadruped-robot-control
pio run
```

Hubungkan ESP32 melalui USB, periksa port yang terdeteksi, lalu upload:

```bash
pio device list
pio run -t upload
pio device monitor --baud 115200
```

Jika upload memerlukan port tertentu, gunakan opsi PlatformIO yang sesuai atau atur `upload_port` di konfigurasi lokal. Hindari mengubah konfigurasi pin dan servo tanpa mencocokkannya dengan wiring robot.

Unit test firmware native (jika didukung lingkungan PlatformIO Anda):

```bash
pio test -e native
```

Detail kontrol serial/SPI, konfigurasi servo, wiring, dan protokol tersedia di [`quadruped-robot-control/README.md`](quadruped-robot-control/README.md).

## Troubleshooting Awal

- **`ros2` atau `colcon` tidak ditemukan:** source distro ROS 2 yang terpasang dan pastikan paket build tools sudah terinstal.
- **Paket tidak ditemukan setelah build:** jalankan `source ~/ros2_ws/install/setup.bash` di terminal yang sama.
- **Model tidak ditemukan / backend gagal:** periksa path absolut, jangan tambahkan ekstensi pada `model_path`, pastikan format model dan runtime backend cocok dengan komputer.
- **Tidak ada deteksi atau nama kelas salah:** cocokkan urutan `class_names` dan `class_mapping` dengan dataset/model Anda di `config/class_mapping.yaml`.
- **Kamera gagal dibuka:** uji indeks kamera dan izin device; pastikan tidak sedang dipakai proses lain.
- **ESP32 gagal ter-upload:** periksa kabel USB data, port, permission perangkat serial, dan mode boot board.
- **Gerakan robot tidak sesuai:** jangan menguji dengan robot menggantung bebas; cek ID servo, arah sendi, offset, catu daya, dan konfigurasi gait sebelum menyalakan aktuator.

## Dokumentasi Lanjutan

- [Panduan paket visi, model, parameter, dan perilaku deteksi](vision_avoidance_ros2/README.md)
- [Panduan firmware, wiring, kontrol, protokol, dan testing](quadruped-robot-control/README.md)
- [Driver LiDAR Slamtec](https://github.com/Slamtec/sllidar_ros2)

## Status dan Batasan

Repository ini menggabungkan komponen visi ROS 2 dan firmware ESP32, tetapi menjalankan satu komponen tidak otomatis mengonfigurasi keseluruhan robot. Integrasi kamera, model, topic ROS, komunikasi ke ESP32, wiring, dan kalibrasi harus disesuaikan serta diuji pada setup masing-masing. Petunjuk di README ini belum merupakan klaim bahwa seluruh kombinasi hardware dan distro ROS 2 telah diuji.

## Lisensi

Lisensi paket visi mencantumkan MIT di metadata paketnya. Untuk firmware dan aset/model lain, periksa lisensi masing-masing sebelum menggunakan, memodifikasi, atau mendistribusikannya.

---

Kontribusi, laporan masalah, dan perbaikan dokumentasi sangat dipersilakan melalui GitHub Issues dan Pull Requests.
