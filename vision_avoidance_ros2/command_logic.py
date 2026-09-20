"""
command_logic.py

Logika pengambilan keputusan gerak.

Prioritas (dari atas ke bawah, berhenti begitu 1 kondisi terpenuhi):
  1. Rintangan HORIZONTAL di jalur tengah, area atas frame, cukup dekat
     -> "merunduk"  (SELALU pakai area_ratio -- ground-plane distance
        tidak valid untuk objek melayang, lihat distance_estimator.py)
  2. Rintangan apapun di jalur tengah, SANGAT dekat (jalur tertutup)
     -> "mundur"    (pakai distance_m kalau tersedia [hanya utk VERTICAL],
        fallback area_ratio kalau tidak)
  3. Rintangan VERTICAL di jalur tengah, cukup dekat
     -> "kiri" / "kanan" (sama, distance_m kalau tersedia)
  4. Tidak ada rintangan berarti di jalur tengah
     -> "maju"

BARU (v0.3.0): CommandThresholds punya field opsional
near_distance_stop_m / near_distance_turn_m. Kalau diisi (bukan None)
DAN obstacle.distance_m tersedia (cuma untuk VERTICAL), threshold jarak
metrik dipakai -- lebih akurat daripada area_ratio. Kalau salah satu
kosong, otomatis fallback ke area_ratio seperti versi sebelumnya --
jadi tetap backward compatible, tidak wajib dikonfigurasi.
"""

from __future__ import annotations

from dataclasses import dataclass

from vision_avoidance_ros2.obstacle_classifier import Obstacle, ObstacleType


@dataclass(frozen=True)
class CommandThresholds:
    near_threshold_duck: float = 0.06
    near_threshold_turn: float = 0.05
    near_threshold_stop: float = 0.15
    center_zone_min: float = 0.30
    center_zone_max: float = 0.70
    top_zone_max: float = 0.55

    # opsional -- jarak metrik (meter), lihat docstring modul
    near_distance_stop_m: float | None = None
    near_distance_turn_m: float | None = None


def _is_near_stop(o: Obstacle, thresholds: CommandThresholds) -> bool:
    if (
        o.obstacle_type == ObstacleType.VERTICAL
        and o.distance_m is not None
        and thresholds.near_distance_stop_m is not None
    ):
        return o.distance_m <= thresholds.near_distance_stop_m
    return o.area_ratio >= thresholds.near_threshold_stop


def _is_near_turn(o: Obstacle, thresholds: CommandThresholds) -> bool:
    if (
        o.obstacle_type == ObstacleType.VERTICAL
        and o.distance_m is not None
        and thresholds.near_distance_turn_m is not None
    ):
        return o.distance_m <= thresholds.near_distance_turn_m
    return o.area_ratio >= thresholds.near_threshold_turn


def decide_command(obstacles: list[Obstacle], thresholds: CommandThresholds) -> str:
    center_obstacles = [
        o for o in obstacles
        if thresholds.center_zone_min <= o.center_x_ratio <= thresholds.center_zone_max
    ]

    # 1) HORIZONTAL di area atas & dekat -> merunduk (selalu area_ratio)
    for o in center_obstacles:
        if (
            o.obstacle_type == ObstacleType.HORIZONTAL
            and o.center_y_ratio <= thresholds.top_zone_max
            and o.area_ratio >= thresholds.near_threshold_duck
        ):
            return "merunduk"

    # 2) Rintangan sangat dekat & menutup jalur -> mundur
    for o in center_obstacles:
        if _is_near_stop(o, thresholds):
            return "mundur"

    # 3) VERTICAL cukup dekat di jalur tengah -> belok ke sisi lebih lapang
    vertical_center = [
        o for o in center_obstacles
        if o.obstacle_type == ObstacleType.VERTICAL and _is_near_turn(o, thresholds)
    ]
    if vertical_center:
        # Kepadatan kiri/kanan tetap pakai area_ratio (bukan distance_m) --
        # ini soal distribusi spasial rintangan di seluruh frame, bukan
        # soal seberapa dekat 1 rintangan spesifik, jadi area_ratio tetap
        # proxy yang cukup untuk keputusan arah.
        left_load = sum(o.area_ratio for o in obstacles if o.center_x_ratio < 0.5)
        right_load = sum(o.area_ratio for o in obstacles if o.center_x_ratio >= 0.5)
        return "kanan" if left_load >= right_load else "kiri"

    # 4) Default -> maju
    return "maju"
