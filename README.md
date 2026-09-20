# vision_avoidance_ros2 (v0.3.0)

Node ROS 2 untuk penghindaran rintangan **vertikal** dan **horizontal**
pada quadruped robot, menggunakan model YOLO hasil training Roboflow.
Output berupa string perintah gerak: `maju`, `mundur`, `kiri`, `kanan`,
`merunduk`, di-publish ke topic `/quadruped/cmd_vision` (`std_msgs/String`).

Target: **Orange Pi 5 Pro**, **Python 3.10**, **ROS 2** (Humble/Jazzy).

---

## Yang baru di v0.3.0

Tiga fitur diadaptasi dari codebase tim lomba kapal Anda (juga ROS2 +
YOLO + RKNN, konteks berbeda tapi arsitekturnya relevan):

1. **Ground-plane distance estimation** (`distance_estimator.py`) --
   estimasi jarak METER sungguhan dari geometri kamera (tinggi + sudut
   kemiringan), bukan cuma proxy `area_ratio`. HANYA valid untuk
   rintangan VERTICAL yang menyentuh lantai -- opsional, mati secara
   default, aktifkan di `config/class_mapping.yaml` section `distance`.
2. **Frame buffer + worker thread terpisah** -- capture kamera
   (`capture_callback`, timer cepat) tidak lagi tertahan menunggu
   inferensi selesai. Frame ditaruh ke `deque`, worker thread terpisah
   memproses secepat model sanggup. Meningkatkan responsivitas kalau
   inferensi lambat (mis. backend ONNX CPU).
3. **Model warm-up + `CAP_PROP_BUFFERSIZE=1`** -- warm-up menghilangkan
   lonjakan latency "cold start" di inferensi pertama; buffer size 1
   memaksa kamera selalu memberi frame TERBARU, bukan frame basi dari
   buffer internal driver.

Lihat komentar di masing-masing file untuk detail teknis & alasan desain.

---

## Asal-usul desain package ini

| Modul | Asal | Keterangan |
|---|---|---|
| `cv_model.py` | Diadaptasi dari `core/tracking/models.py` kating | Dual backend RKNN/ONNX |
| `yolo_pipeline.py` | Diadaptasi dari `core/tracking/pipeline.py` kating | Letterbox, parsing multi-kelas, NMS berlapis |
| `metrics.py` | Diadaptasi hampir verbatim dari `core/cv/metrics.py` kating | Telemetry FPS/latency |
| `distance_estimator.py` | **Baru**, diadaptasi dari `vision.distance.*` tim kapal | Ground-plane projection |
| `obstacle_classifier.py` | **Baru**, hasil diskusi kita | Klasifikasi vertikal/horizontal |
| `command_logic.py` | **Baru**, hasil diskusi kita | Prioritas keputusan gerak |
| `vision_node.py` | Pola diadaptasi dari `core/cv/processor.py` kating + `vision_node.py` tim kapal | Graceful degradation, frame buffer/worker thread, warm-up |

### Yang SENGAJA dihilangkan (dari kedua sumber referensi)

- **DeepSort + ReID** (kating) -- bukan tracking objek bergerak.
- **PyAV/WebRTC/SPI** (kating) -- tidak ada streaming jarak jauh / motor SPI.
- **3 instance model paralel di 3 NPU core, publish image streaming, RC value mapping, LiDAR/GPS/compass integration** (tim kapal) -- di luar scope TA penghindaran rintangan Anda; kompleksitasnya tidak proporsional untuk 1 model deteksi rintangan sederhana.

---

## 1. Menyiapkan model dari Roboflow

1. Export ke `.onnx` (testing laptop) dan/atau `.rknn` (Orange Pi).
2. Taruh di `models/` (mis. `models/best.onnx` dan/atau `models/best.rknn`).
3. Cek urutan kelas: `python3 -c "from ultralytics import YOLO; print(YOLO('models/best.pt').names)"`
4. Isi `config/class_mapping.yaml`: `class_names` (urutan index PERSIS training) dan `class_mapping`.
5. **Opsional**: isi section `distance` kalau mau estimasi jarak metrik -- lihat panduan kalibrasi lengkap di dalam file YAML itu sendiri.

---

## 2. Instalasi

```bash
pip3 install -r requirements.txt   # onnxruntime, opencv-python, pyyaml, numpy

# Khusus Orange Pi 5 Pro, tambahan (tidak ada di PyPI publik):
# wget https://github.com/airockchip/rknn-toolkit2/raw/main/rknn-toolkit-lite2/packages/<wheel>.whl
# pip3 install <wheel>.whl

sudo apt update && sudo apt install python3-yaml python3-numpy python3-opencv
cp -r vision_avoidance_ros2 ~/ros2_ws/src/
cd ~/ros2_ws
rosdep install --from-paths src --ignore-src -r -y
colcon build --packages-select vision_avoidance_ros2 --symlink-install
source install/setup.bash
```

---

## 3. Menjalankan

```bash
ros2 launch vision_avoidance_ros2 vision_avoidance.launch.py \
  model_path:=/home/orangepi/ros2_ws/src/vision_avoidance_ros2/models/best \
  debug_view:=true \
  telemetry_enabled:=true \
  capture_rate_hz:=15.0
```

`model_path` **tanpa ekstensi** -- otomatis `.rknn` (ARM) atau `.onnx` (non-ARM).

Cek output: `ros2 topic echo /quadruped/cmd_vision`

---

## 4. Parameter baru v0.3.0

| Parameter | Default | Fungsi |
|---|---|---|
| `capture_rate_hz` | 15.0 | Rate capture kamera (bukan lagi rate publish -- publish sekarang secepat worker thread menyelesaikan inferensi) |
| `frame_buffer_size` | 2 | Ukuran buffer antara capture & worker (frame lebih lama otomatis dibuang kalau penuh) |

Di `config/class_mapping.yaml`, section `distance` (semua opsional):

| Key | Default | Fungsi |
|---|---|---|
| `use_distance_based` | `false` | Aktifkan ground-plane distance |
| `camera_height` | 0.30 | Tinggi kamera dari lantai (meter) |
| `camera_tilt_deg` | 15.0 | Sudut kemiringan kamera ke bawah (derajat) |
| `focal_length_y` | 500.0 | Focal length vertikal (pixel) |
| `principal_point_y` | 240.0 | Titik utama vertikal (pixel, umumnya frame_height/2) |
| `near_distance_stop_m` | 0.4 | Jarak (meter) memicu mundur, untuk rintangan VERTICAL |
| `near_distance_turn_m` | 0.8 | Jarak (meter) memicu belok, untuk rintangan VERTICAL |

Kalau `use_distance_based: false` (default), perilaku identik v0.2.0 -- 100% backward compatible, tidak wajib dikonfigurasi.

---

## 5. Cara kerja frame buffer + worker thread (penting dipahami)

```
[Main thread]                          [Worker thread, terpisah]
capture_timer (15 Hz)                  loop terus-menerus:
    |                                      |
    |- cap.read()                          |- ambil frame dari buffer
    |- push ke frame_buffer (deque)        |- inferensi model (durasi bervariasi)
    |  (buang frame terlama kalau penuh)   |- decide_command()
    |                                      |- publish ke topic
preview_timer (15 Hz)                      |- simpan gambar utk preview (lock)
    |- ambil gambar terbaru (lock)         `- (ulang)
    `- cv2.imshow() -- SELALU main thread
```

Kenapa preview dipisah lagi jadi timer sendiri (bukan langsung dari
worker thread): `cv2.imshow` di beberapa platform (terutama macOS) HARUS
dipanggil dari main thread. Worker thread cuma menyiapkan gambar
(disimpan dengan `threading.Lock`), window sungguhan digambar oleh
`_preview_timer_callback` yang berjalan di main thread ROS2 (lewat
`create_timer` biasa, bukan thread manual).

`self._stats = {"processed": ..., "dropped": ...}` dicatat untuk
melihat seberapa sering frame dibuang karena worker tidak sanggup
mengejar rate capture -- muncul di log diagnostics (`diagnostics_log_every_n`)
dan di overlay debug (`debug_view:=true`).

---

## 6. Batasan yang tetap perlu dicatat di laporan TA

- Ground-plane distance **hanya** akurat kalau kalibrasi geometri kamera
  (`camera_height`, `camera_tilt_deg`, `focal_length_y`) benar -- kalau
  Anda tidak sempat kalibrasi presisi, biarkan `use_distance_based: false`
  dan gunakan `area_ratio` seperti sebelumnya (tetap berfungsi, cuma
  kurang presisi jaraknya).
- Distance estimation **tidak berlaku** untuk rintangan HORIZONTAL
  (palang, dahan) karena asumsi ground-plane tidak valid untuk objek
  yang tidak menyentuh lantai -- ini penting dijelaskan di laporan kalau
  ditanya kenapa cuma sebagian rintangan yang dapat estimasi jarak metrik.
- Frame buffer/worker thread mengurangi risiko frame "basi", tapi tetap
  ada overhead kecil dari threading (context switching, lock) -- untuk
  robot dengan CPU sangat terbatas, ukur dulu (`telemetry_enabled:=true`)
  apakah perubahan ini benar-benar meningkatkan FPS efektif di device
  Anda, jangan asumsikan otomatis lebih cepat tanpa diukur.
