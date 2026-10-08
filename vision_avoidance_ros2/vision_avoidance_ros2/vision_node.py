"""
vision_node.py (v0.3.0)

Node ROS2 utama. PERUBAHAN dari v0.2.0 (diadaptasi dari pola
vision_node.py Anda):

  1. Frame buffer + worker thread terpisah dari capture loop.
     Sebelumnya: timer_callback melakukan capture -> inferensi -> publish
     SECARA BERURUTAN dalam 1 thread. Kalau inferensi lambat, capture
     ikut tertahan -> frame yang diproses jadi "basi".
     Sekarang: capture_callback (timer, cepat) HANYA capture frame lalu
     taruh ke buffer (deque). Worker thread terpisah mengambil frame dari
     buffer, jalankan inferensi+keputusan+publish, secepat model sanggup.
     Capture TIDAK PERNAH tertahan menunggu inferensi selesai.

  2. CAP_PROP_BUFFERSIZE=1 -- paksa cap.read() selalu ambil frame
     TERBARU dari driver kamera, bukan frame lama dari buffer internal
     OpenCV/driver.

  3. Model warm-up -- 1x inferensi dummy setelah model dimuat, supaya
     lonjakan latency "cold start" terjadi saat startup node, bukan saat
     robot pertama kali menghadapi rintangan sungguhan.

  4. Ground-plane distance estimation (opsional) -- kalau dikonfigurasi
     di class_mapping.yaml (section `distance`), setiap rintangan yang
     memiliki referensi di lantai dapat estimasi jarak metrik (meter).
     Contohnya rintangan VERTICAL dan portal HORIZONTAL yang bbox-nya
     mencakup kaki sampai lantai.

CATATAN PLATFORM PENTING: cv2.imshow (dev popup preview) sebaiknya
dipanggil dari MAIN THREAD, bukan dari worker thread -- beberapa platform
(terutama macOS) mengharuskan ini. Karena itu, worker thread HANYA
menyiapkan gambar visualisasi (disimpan di memory dengan lock), dan
window ditampilkan lewat timer terpisah yang berjalan di main thread
(_preview_timer_callback).
"""

from __future__ import annotations

import os
import threading
import time
from collections import deque
from typing import Any

import cv2
import numpy as np
import rclpy
import yaml
from ament_index_python.packages import get_package_share_directory
from rclpy.node import Node
from std_msgs.msg import String

# Circuit breaker for error handling
CIRCUIT_BREAKER_ERROR_THRESHOLD = 10
CIRCUIT_BREAKER_RESET_INTERVAL_S = 10.0

from vision_avoidance_ros2.command_logic import (
    CommandStabilizer,
    CommandThresholds,
    decide_command,
    emergency_stop_required,
)
from vision_avoidance_ros2.cv_model import CvModel
from vision_avoidance_ros2.distance_estimator import CameraGeometry, estimate_ground_distance
from vision_avoidance_ros2.metrics import CvMetricsWindow, CvTelemetryWriter
from vision_avoidance_ros2.obstacle_classifier import ObstacleClassifier, ObstacleType
from vision_avoidance_ros2.obstacle_tracker import ObstacleTracker
from vision_avoidance_ros2.yolo_pipeline import (
    YoloParseStats,
    filter_by_size,
    nms_by_center,
    parse_yolo_output_with_stats,
    prepare_yolo_input,
)


class VisionAvoidanceNode(Node):
    def __init__(self) -> None:
        super().__init__("vision_avoidance_node")

        self._declare_parameters()
        p = self.get_parameter

        self.camera_index = p("camera_index").value
        self.frame_width = p("frame_width").value
        self.frame_height = p("frame_height").value
        self.publish_topic = p("publish_topic").value
        self.capture_rate_hz = p("capture_rate_hz").value
        self.debug_view = p("debug_view").value
        self.telemetry_enabled = p("telemetry_enabled").value
        self.diagnostics_log_every_n = p("diagnostics_log_every_n").value
        self.frame_buffer_size = max(1, int(p("frame_buffer_size").value))
        self.command_stability_frames = max(1, int(p("command_stability_frames").value))
        self.obstacle_confirmation_frames = max(
            1, int(p("obstacle_confirmation_frames").value)
        )
        self.obstacle_hold_seconds = max(0.0, float(p("obstacle_hold_seconds").value))

        # ---------------- Parameter validation ----------------
        if self.frame_width <= 0 or self.frame_height <= 0:
            raise ValueError("frame_width dan frame_height harus positif")
        if self.capture_rate_hz <= 0:
            raise ValueError("capture_rate_hz harus positif")

        model_path = p("model_path").value
        if not model_path:
            raise RuntimeError(
                "Parameter 'model_path' wajib diisi (tanpa ekstensi file), contoh:\n"
                "  ros2 run vision_avoidance_ros2 vision_node --ros-args "
                "-p model_path:=/path/ke/best"
            )

        config_path = p("class_mapping_path").value
        if not config_path:
            share_dir = get_package_share_directory("vision_avoidance_ros2")
            config_path = os.path.join(share_dir, "config", "class_mapping.yaml")
        cfg = self._load_yaml_config(config_path)

        self.class_names: list[str] = cfg.get("class_names", [])
        if not self.class_names:
            self.get_logger().warn(
                "class_names kosong di config -- parsing multi-kelas tidak akan "
                "bekerja dengan benar. Isi config/class_mapping.yaml sesuai model Anda."
            )

        self.conf_threshold = float(cfg.get("confidence_threshold", 0.5))
        self.nms_iou_threshold = float(cfg.get("nms_iou_threshold", 0.45))
        self.nms_center_distance = float(cfg.get("nms_center_distance", 20))
        self.min_box_area = float(cfg.get("min_box_area", 900))
        self.max_box_area_ratio = float(cfg.get("max_box_area_ratio", 0.6))
        self.model_input_size = int(cfg.get("model_input_size", 640))

        self.classifier = ObstacleClassifier(
            class_mapping=cfg.get("class_mapping", {}),
            ground_contact_classes=cfg.get("ground_contact_classes", []),
            vertical_aspect_ratio=float(cfg.get("vertical_aspect_ratio", 1.4)),
            horizontal_aspect_ratio=float(cfg.get("horizontal_aspect_ratio", 1.4)),
        )

        # ---------------- Ground-plane distance (opsional) ----------------
        dist_cfg = cfg.get("distance", {}) or {}
        self.distance_enabled = bool(dist_cfg.get("use_distance_based", False))
        self.geometry: CameraGeometry | None = None
        if self.distance_enabled:
            try:
                self.geometry = CameraGeometry(
                    camera_height=float(dist_cfg["camera_height"]),
                    camera_tilt_deg=float(dist_cfg["camera_tilt_deg"]),
                    focal_length_y=float(dist_cfg["focal_length_y"]),
                    principal_point_y=float(dist_cfg["principal_point_y"]),
                )
                self.get_logger().info(
                    f"Ground-plane distance AKTIF. horizon_y perkiraan = "
                    f"{self.geometry.horizon_y:.1f} px (cek README bagian kalibrasi "
                    f"kalau nilai ini terasa tidak sesuai gambar nyata)."
                )
            except (KeyError, ValueError) as exc:
                self.distance_enabled = False
                self.get_logger().error(
                    f"Config 'distance' tidak lengkap/valid ({exc}), "
                    "ground-plane distance dinonaktifkan, fallback ke area_ratio."
                )

        self.thresholds = CommandThresholds(
            near_threshold_duck=p("near_threshold_duck").value,
            near_threshold_turn=p("near_threshold_turn").value,
            near_threshold_stop=p("near_threshold_stop").value,
            center_zone_min=p("center_zone_min").value,
            center_zone_max=p("center_zone_max").value,
            top_zone_max=p("top_zone_max").value,
            near_distance_stop_m=(
                float(dist_cfg["near_distance_stop_m"])
                if self.distance_enabled and "near_distance_stop_m" in dist_cfg
                else None
            ),
            near_distance_turn_m=(
                float(dist_cfg["near_distance_turn_m"])
                if self.distance_enabled and "near_distance_turn_m" in dist_cfg
                else None
            ),
            near_distance_duck_m=(
                float(dist_cfg["near_distance_duck_m"])
                if self.distance_enabled and "near_distance_duck_m" in dist_cfg
                else None
            ),
        )
        self._command_stabilizer = CommandStabilizer(self.command_stability_frames)
        self._obstacle_tracker = ObstacleTracker(
            confirmation_frames=self.obstacle_confirmation_frames,
            hold_seconds=self.obstacle_hold_seconds,
        )

        # ---------------- Model ----------------
        self._model: CvModel | None = None
        self._models_ready = False
        try:
            self._model = CvModel(model_path, self.get_logger())
            self._model.initialize()
            self._models_ready = True
            self._warm_up_model()
        except Exception:
            self.get_logger().error(
                "Gagal memuat model vision. Node tetap berjalan (graceful "
                "degradation) tapi TIDAK akan mempublish command sampai "
                "model berhasil dimuat ulang.",
            )

        # Background thread untuk retry load model tiap 30 detik jika belum ready
        self._model_retry_running = True
        self._model_retry_thread = threading.Thread(target=self._model_retry_loop, daemon=True)
        self._model_retry_thread.start()

        # ---------------- Telemetry (opsional) ----------------
        self._telemetry_writer: CvTelemetryWriter | None = None
        self._telemetry_window: CvMetricsWindow | None = None
        if self.telemetry_enabled:
            os.makedirs("logs", exist_ok=True)
            self._telemetry_writer = CvTelemetryWriter("logs")
            self._telemetry_window = CvMetricsWindow(1.0)
            self.get_logger().info("Telemetry aktif, log ditulis ke folder 'logs/'.")

        # ---------------- Popup preview (opsional, dev) ----------------
        self._popup_preview_enabled = False
        self._preview_window_name = "vision_avoidance debug"
        self._preview_window_opened = False
        if self.debug_view:
            if self._has_local_display():
                self._popup_preview_enabled = True
                self.get_logger().info("Dev popup preview aktif.")
            else:
                self.get_logger().warn(
                    "debug_view diminta tapi tidak ada local display "
                    "terdeteksi (headless) -- preview dinonaktifkan."
                )

        self._diagnostics_frame_counter = 0
        self._last_command: str | None = None

        # ---------------- Frame buffer + worker thread ----------------
        # deque bawaan Python thread-safe untuk append/popleft tunggal
        # (dijamin atomik oleh GIL), jadi TIDAK perlu lock tambahan untuk
        # struktur buffer itu sendiri -- pola yang sama seperti kode tim
        # kapal Anda.
        self._frame_buffer: deque = deque(maxlen=self.frame_buffer_size)
        self._stats = {"processed": 0, "dropped": 0}

        # Lock terpisah HANYA untuk data yang dibaca/ditulis lintas thread
        # secara bersamaan (gambar preview untuk ditampilkan di main thread).
        self._preview_lock = threading.Lock()
        self._latest_preview_frame: np.ndarray | None = None

        self._running = True
        self._worker_thread = threading.Thread(target=self._worker_loop, daemon=True)
        self._worker_thread.start()

        # ---------------- Error tracking (circuit breaker) ----------------
        self._consecutive_errors = 0
        self._last_error_reset_time = time.time()

        # ---------------- Camera ----------------
        self._camera_retry_count = 0
        self._max_camera_retries = 3
        self._camera_last_fail_time = 0.0
        
        self.cap = cv2.VideoCapture(self.camera_index)
        # CAP_PROP_BUFFERSIZE=1: paksa selalu ambil frame TERBARU dari
        # driver kamera, bukan frame lama yang tertahan di buffer internal
        # -- penting untuk sistem reaktif real-time seperti ini.
        self.cap.set(cv2.CAP_PROP_BUFFERSIZE, 1)
        self.cap.set(cv2.CAP_PROP_FRAME_WIDTH, self.frame_width)
        self.cap.set(cv2.CAP_PROP_FRAME_HEIGHT, self.frame_height)
        if not self.cap.isOpened():
            self.get_logger().error(f"Tidak bisa membuka kamera index {self.camera_index}. Retry mechanism aktif.")

        # ---------------- Publisher ----------------
        self.publisher_ = self.create_publisher(String, self.publish_topic, 10)

        capture_period = 1.0 / max(self.capture_rate_hz, 1e-3)
        self.capture_timer = self.create_timer(capture_period, self.capture_callback)

        # Timer preview TERPISAH, berjalan di main thread -- lihat catatan
        # platform di docstring modul soal kenapa cv2.imshow tidak boleh
        # dipanggil langsung dari worker thread.
        self.preview_timer = self.create_timer(1.0 / 15.0, self._preview_timer_callback)

        self.get_logger().info(
            f"vision_avoidance_node siap. Publish ke '{self.publish_topic}'. "
            f"Capture @ {self.capture_rate_hz} Hz (worker thread terpisah "
            f"untuk inferensi). Model ready: {self._models_ready}."
        )

    # ------------------------------------------------------------------
    def _declare_parameters(self) -> None:
        self.declare_parameter("camera_index", 0)
        self.declare_parameter("frame_width", 640)
        self.declare_parameter("frame_height", 480)
        self.declare_parameter("publish_topic", "/quadruped/cmd_vision")
        self.declare_parameter("capture_rate_hz", 15.0)
        self.declare_parameter("frame_buffer_size", 2)

        self.declare_parameter("model_path", "")
        self.declare_parameter("class_mapping_path", "")

        self.declare_parameter("near_threshold_duck", 0.06)
        self.declare_parameter("near_threshold_turn", 0.05)
        self.declare_parameter("near_threshold_stop", 0.15)
        self.declare_parameter("center_zone_min", 0.30)
        self.declare_parameter("center_zone_max", 0.70)
        self.declare_parameter("top_zone_max", 0.55)

        self.declare_parameter("debug_view", False)
        self.declare_parameter("telemetry_enabled", False)
        self.declare_parameter("diagnostics_log_every_n", 30)
        self.declare_parameter("command_stability_frames", 3)
        self.declare_parameter("obstacle_confirmation_frames", 3)
        self.declare_parameter("obstacle_hold_seconds", 0.30)

    @staticmethod
    def _load_yaml_config(path: str) -> dict:
        if not os.path.isfile(path):
            return {}
        with open(path, "r") as f:
            return yaml.safe_load(f) or {}

    @staticmethod
    def _has_local_display() -> bool:
        import platform
        if platform.system() == "Windows":
            return True
        return bool(os.getenv("DISPLAY") or os.getenv("WAYLAND_DISPLAY"))

    def _warm_up_model(self) -> None:
        """1x inferensi dummy supaya cold-start latency terjadi saat startup, bukan saat operasi."""
        if self._model is None:
            return
        try:
            dummy_frame = np.zeros((self.frame_height, self.frame_width, 3), dtype=np.uint8)
            input_tensor, _, _, _ = prepare_yolo_input(dummy_frame, self.model_input_size)
            data_format = self._model.preferred_data_format
            if data_format == "nchw":
                input_tensor = input_tensor.astype(np.float32) / 255.0
                input_tensor = np.transpose(input_tensor, (0, 3, 1, 2))
            t0 = time.perf_counter()
            self._model.infer(inputs=[input_tensor], data_format=data_format)
            elapsed_ms = (time.perf_counter() - t0) * 1000.0
            self.get_logger().info(f"Model warm-up selesai ({elapsed_ms:.1f} ms).")
        except Exception:
            self.get_logger().warn(
                "Model warm-up gagal (dilewati, tidak fatal -- cold-start "
                "latency mungkin muncul di inferensi pertama).",
                exc_info=True,
            )

    def _reload_model(self, new_model_path: str) -> bool:
        """Coba reload model dari path baru. Digunakan untuk recovery otomatis."""
        self.get_logger().info(f"Mencoba reload model dari: {new_model_path}")
        try:
            # Release model lama jika ada
            if self._model is not None:
                self._model.release()

            # Load model baru
            new_model = CvModel(new_model_path, self.get_logger())
            new_model.initialize()

            # Replace model
            self._model = new_model
            self._models_ready = True

            # Warm-up model baru
            self._warm_up_model()

            self.get_logger().info(f"Model berhasil reload dari: {new_model_path}")
            self._consecutive_errors = 0  # Reset error counter setelah reload berhasil
            return True

        except Exception as e:
            self.get_logger().error(f"Gagal reload model dari {new_model_path}: {e}", exc_info=True)
            return False

    def _model_retry_loop(self) -> None:
        """Background thread: coba reload model setiap 30 detik jika belum ready."""
        while self._model_retry_running:
            if not self._models_ready:
                try:
                    model_path = self.get_parameter("model_path").value
                    if model_path:
                        self.get_logger().info("Retry otomatis: mencoba memuat model lagi...")
                        self._reload_model(model_path)
                except Exception as e:
                    self.get_logger().error(f"Error saat retry load model: {e}")
            time.sleep(30.0)

    # ------------------------------------------------------------------
    # CAPTURE (main thread, timer, CEPAT -- tidak pernah menunggu inferensi)
    # ------------------------------------------------------------------
    def capture_callback(self) -> None:
        # Camera retry mechanism: coba reopen jika kamera tidak terbuka
        if not self.cap.isOpened():
            current_time = time.time()
            if (current_time - self._camera_last_fail_time) > CIRCUIT_BREAKER_RESET_INTERVAL_S:
                if self._camera_retry_count < self._max_camera_retries:
                    self._camera_retry_count += 1
                    self._camera_last_fail_time = current_time
                    self.get_logger().warn(
                        f"Camera tidak terbuka, attempt retry {self._camera_retry_count}/{self._max_camera_retries}"
                    )
                    self.cap.open(self.camera_index)
                else:
                    self.get_logger().error("Camera permanently unavailable. Tidak dapat membuka kamera setelah beberapa retry.")
                    self._camera_last_fail_time = current_time
            return

        ok, frame = self.cap.read()
        if not ok or frame is None:
            self.get_logger().warn("Gagal membaca frame dari webcam.")
            return

        if len(self._frame_buffer) >= self._frame_buffer.maxlen:
            self._frame_buffer.popleft()
            self._stats["dropped"] += 1
        self._frame_buffer.append(frame)

    # ------------------------------------------------------------------
    # WORKER (thread terpisah, LOOP TERUS -- secepat inferensi sanggup)
    # ------------------------------------------------------------------
    def _worker_loop(self) -> None:
        while self._running:
            if not self._frame_buffer:
                time.sleep(0.001)
                continue

            try:
                frame = self._frame_buffer.popleft()
            except IndexError:
                continue

            frame_start = time.perf_counter()
            input_bytes = int(frame.nbytes)

            # Reset error counter setiap CIRCUIT_BREAKER_RESET_INTERVAL_S detik
            current_time = time.time()
            if current_time - self._last_error_reset_time > CIRCUIT_BREAKER_RESET_INTERVAL_S:
                self._consecutive_errors = 0
                self._last_error_reset_time = current_time

            if not self._models_ready or self._model is None:
                if self._popup_preview_enabled:
                    with self._preview_lock:
                        self._latest_preview_frame = frame
                self._record_telemetry(frame_start, input_bytes, input_bytes, None)
                continue

            try:
                raw_obstacles, parse_stats, detections_count = self._process_frame(frame)
            except Exception as e:
                self._consecutive_errors += 1
                self.get_logger().error(
                    f"Error saat memproses frame di worker thread (error #{self._consecutive_errors}): {e}",
                    exc_info=True
                )
                # Circuit breaker: log critical jika error terlalu sering lalu reset system
                if self._consecutive_errors >= CIRCUIT_BREAKER_ERROR_THRESHOLD:
                    self.get_logger().fatal(
                        f"Circuit breaker triggered: {self._consecutive_errors} errors dalam "
                        f"{CIRCUIT_BREAKER_RESET_INTERVAL_S} detik. Periksa log detail."
                    )
                    # Reset counter dan coba reload model sebagai upaya recovery
                    self._consecutive_errors = 0
                    try:
                        model_path = self.get_parameter("model_path").value
                        if model_path:
                            self._reload_model(model_path)
                    except Exception as reload_exc:
                        self.get_logger().error(f"Gagal reload model setelah circuit breaker: {reload_exc}")
                continue

            # Deteksi hanya menjadi dasar command normal setelah konsisten
            # beberapa frame. Namun tiang yang sangat dekat tetap langsung
            # memicu mundur dari deteksi mentah demi keselamatan.
            tracked_obstacles = self._obstacle_tracker.update(raw_obstacles)
            candidate_command = (
                "mundur"
                if emergency_stop_required(raw_obstacles, self.thresholds)
                else decide_command(tracked_obstacles, self.thresholds)
            )
            command = self._command_stabilizer.update(candidate_command)

            msg = String()
            msg.data = command
            self.publisher_.publish(msg)
            self._stats["processed"] += 1

            if command != self._last_command:
                self.get_logger().info(f"Perintah: {command}")
                self._last_command = command

            self._log_diagnostics(parse_stats, len(raw_obstacles))

            if self._popup_preview_enabled:
                vis = self._draw_debug(
                    frame,
                    raw_obstacles,
                    command,
                    candidate_command,
                    tracked_count=len(tracked_obstacles),
                )
                with self._preview_lock:
                    self._latest_preview_frame = vis

            self._record_telemetry(frame_start, input_bytes, input_bytes, detections_count)

    # ------------------------------------------------------------------
    def _process_frame(self, frame: np.ndarray):
        frame_height, frame_width = frame.shape[:2]

        input_tensor, scale, pad_w, pad_h = prepare_yolo_input(frame, self.model_input_size)
        data_format = self._model.preferred_data_format
        if data_format == "nchw":
            input_tensor = input_tensor.astype(np.float32) / 255.0
            input_tensor = np.transpose(input_tensor, (0, 3, 1, 2))

        outputs = self._model.infer(inputs=[input_tensor], data_format=data_format)

        detections, parse_stats = parse_yolo_output_with_stats(
            outputs[0], scale, pad_w, pad_h, frame_width, frame_height,
            conf_threshold=self.conf_threshold, class_names=self.class_names,
        )

        boxes_list: list[list[int]] = []
        confidences_list: list[float] = []
        class_names_list: list[str] = []
        for bbox, conf, class_name in detections:
            boxes_list.append(bbox)
            confidences_list.append(float(conf))
            class_names_list.append(class_name)

        indices = cv2.dnn.NMSBoxes(
            boxes_list, confidences_list, self.conf_threshold, self.nms_iou_threshold
        )

        valid_detections: list[dict[str, Any]] = []
        if len(indices) > 0:
            for i in np.array(indices).flatten():
                valid_detections.append({
                    "box": boxes_list[i],
                    "conf": confidences_list[i],
                    "class_name": class_names_list[i],
                })

        valid_detections = filter_by_size(
            valid_detections, (frame_width, frame_height),
            min_area=self.min_box_area, max_area_ratio=self.max_box_area_ratio,
        )
        valid_detections = nms_by_center(valid_detections, max_center_dist=self.nms_center_distance)

        obstacles = []
        for det in valid_detections:
            x, y, w, h = det["box"]
            
            # --- Bounding box validation (clamping ke frame bounds) ---
            x = max(0, min(x, frame_width - 1))
            y = max(0, min(y, frame_height - 1))
            w = max(1, min(w, frame_width - x))
            h = max(1, min(h, frame_height - y))

            # Estimasi jarak ground-plane hanya untuk bbox yang sisi bawahnya
            # benar-benar menyentuh lantai: VERTICAL, atau kelas eksplisit
            # seperti portal yang memiliki kaki. Palang melayang tidak boleh
            # diproyeksikan ke lantai.
            distance_m = None
            if (
                self.distance_enabled
                and self.geometry is not None
                and self.classifier.has_ground_reference(det["class_name"], w, h)
            ):
                distance_m = estimate_ground_distance(float(y + h), self.geometry)

            obstacles.append(
                self.classifier.build_obstacle(
                    bbox=(x, y, w, h),
                    class_name=det["class_name"],
                    confidence=det["conf"],
                    frame_width=frame_width,
                    frame_height=frame_height,
                    distance_m=distance_m,
                )
            )

        return obstacles, parse_stats, len(detections)

    # ------------------------------------------------------------------
    def _log_diagnostics(self, parse_stats: YoloParseStats, obstacle_count: int) -> None:
        if not self.diagnostics_log_every_n:
            return
        self._diagnostics_frame_counter += 1
        every_n = max(1, int(self.diagnostics_log_every_n))
        if self._diagnostics_frame_counter % every_n != 0:
            return
        self.get_logger().debug(
            f"cv_diag candidates={parse_stats.total_candidates} "
            f"dropped(shape={parse_stats.dropped_bad_shape},"
            f"conf={parse_stats.dropped_low_confidence},"
            f"box={parse_stats.dropped_invalid_box}) "
            f"before_nms={parse_stats.before_nms} obstacles={obstacle_count} "
            f"buffer_stats(processed={self._stats['processed']},dropped={self._stats['dropped']})"
        )

    def _record_telemetry(
        self, frame_start: float, incoming_bytes: int, output_bytes: int, detected_count: int | None
    ) -> None:
        if self._telemetry_writer is None or self._telemetry_window is None:
            return
        latency_ms = (time.perf_counter() - frame_start) * 1000.0
        sample = self._telemetry_window.add_frame(
            mode="obstacle_avoidance",
            latency_ms=latency_ms,
            incoming_bytes=incoming_bytes,
            output_bytes=output_bytes,
            detected_count=detected_count,
        )
        if sample is not None:
            self._telemetry_writer.write_sample(sample)

    # ------------------------------------------------------------------
    def _draw_debug(
        self,
        frame: np.ndarray,
        obstacles,
        command: str,
        candidate_command: str,
        tracked_count: int,
    ) -> np.ndarray:
        vis = frame.copy()
        color_map = {
            ObstacleType.VERTICAL: (0, 0, 255),
            ObstacleType.HORIZONTAL: (255, 0, 0),
            ObstacleType.UNKNOWN: (0, 255, 255),
        }
        for obs in obstacles:
            x, y, w, h = obs.bbox
            color = color_map[obs.obstacle_type]
            cv2.rectangle(vis, (x, y), (x + w, y + h), color, 2)
            label = f"{obs.class_name} [{obs.obstacle_type.value}] {obs.confidence:.2f}"
            if obs.distance_m is not None:
                label += f" ground={obs.distance_m:.2f}m y={y + h}"
            elif obs.has_ground_reference:
                label += f" ground=calibrate y={y + h}"
            cv2.putText(vis, label, (x, max(y - 5, 10)),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.5, color, 1, cv2.LINE_AA)

        cv2.putText(vis, f"CMD: {command}", (10, 25),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.8, (0, 255, 0), 2, cv2.LINE_AA)
        if candidate_command != command:
            cv2.putText(
                vis, f"candidate: {candidate_command}", (10, 75),
                cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 255, 255), 1, cv2.LINE_AA
            )
        cv2.putText(
            vis, f"proc={self._stats['processed']} drop={self._stats['dropped']}",
            (10, 50), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (200, 200, 200), 1, cv2.LINE_AA
        )
        cv2.putText(
            vis,
            f"detected={len(obstacles)} tracked={tracked_count}",
            (10, 95),
            cv2.FONT_HERSHEY_SIMPLEX,
            0.5,
            (200, 200, 200),
            1,
            cv2.LINE_AA,
        )
        return vis

    def _preview_timer_callback(self) -> None:
        """Berjalan di MAIN THREAD -- satu-satunya tempat cv2.imshow dipanggil."""
        if not self._popup_preview_enabled:
            return
        with self._preview_lock:
            vis = self._latest_preview_frame
        if vis is None:
            return
        self._show_popup_preview(vis)

    def _show_popup_preview(self, image: np.ndarray) -> None:
        try:
            cv2.imshow(self._preview_window_name, image)
            self._preview_window_opened = True
            key = cv2.waitKey(1) & 0xFF
            if key in (27, ord("q"), ord("Q")):
                self.get_logger().info("Preview window ditutup oleh user (q/Esc).")
                self._popup_preview_enabled = False
                self._close_preview_window()
        except Exception:
            self.get_logger().warn("Gagal render preview window, dinonaktifkan.")
            self._popup_preview_enabled = False
            self._close_preview_window()

    def _close_preview_window(self) -> None:
        if not self._preview_window_opened:
            return
        try:
            cv2.destroyWindow(self._preview_window_name)
        except Exception:
            pass
        finally:
            self._preview_window_opened = False

    # ------------------------------------------------------------------
    def destroy_node(self) -> None:
        self._running = False
        if hasattr(self, "_model_retry_running"):
            self._model_retry_running = False
        if hasattr(self, "_worker_thread"):
            self._worker_thread.join(timeout=5.0)
            if self._worker_thread.is_alive():
                self.get_logger().warn("Worker thread tidak berhenti dengan bersih (timeout).")

        if self.cap is not None and self.cap.isOpened():
            self.cap.release()
        if self._model is not None:
            self._model.release()
        if self._telemetry_writer is not None:
            self._telemetry_writer.close()
        self._close_preview_window()

        self.get_logger().info(
            f"Statistik akhir - processed={self._stats['processed']}, "
            f"dropped={self._stats['dropped']}"
        )
        super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    node = VisionAvoidanceNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
