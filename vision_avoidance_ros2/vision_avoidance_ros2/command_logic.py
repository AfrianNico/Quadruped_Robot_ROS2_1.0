"""
command_logic.py

Logika pengambilan keputusan gerak.

Prioritas (dari atas ke bawah, berhenti begitu 1 kondisi terpenuhi):
  1. Rintangan VERTICAL sangat dekat di jalur tengah -> "mundur".
  2. Rintangan VERTICAL dekat di jalur tengah -> "kiri" / "kanan".
  3. Rintangan HORIZONTAL dekat di jalur tengah, jika lintasan bebas dari
     rintangan vertical -> "merunduk".
  4. Tidak ada rintangan berarti di jalur tengah
     -> "maju"

CommandTemporalFilter menjaga command tetap stabil: perubahan normal baru
diterapkan setelah beberapa frame kandidat yang sama. "mundur" tetap
diterapkan segera sebagai tindakan keselamatan.
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
    near_distance_duck_m: float | None = None


class CommandStabilizer:
    """Mengurangi command yang berkedip karena hasil deteksi per-frame.

    Perubahan command normal harus muncul konsisten selama
    ``confirmation_frames`` frame. Command ``mundur`` tidak ditunda agar
    tindakan keselamatan tetap responsif.
    """

    def __init__(self, confirmation_frames: int = 3) -> None:
        self._confirmation_frames = max(1, int(confirmation_frames))
        self._stable_command: str | None = None
        self._pending_command: str | None = None
        self._pending_count = 0

    def update(self, candidate: str) -> str:
        if self._stable_command is None or candidate == "mundur":
            self._stable_command = candidate
            self._pending_command = None
            self._pending_count = 0
            return candidate

        if candidate == self._stable_command:
            self._pending_command = None
            self._pending_count = 0
            return self._stable_command

        if candidate != self._pending_command:
            self._pending_command = candidate
            self._pending_count = 1
        else:
            self._pending_count += 1

        if self._pending_count >= self._confirmation_frames:
            self._stable_command = candidate
            self._pending_command = None
            self._pending_count = 0

        return self._stable_command


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


def _is_near_duck(o: Obstacle, thresholds: CommandThresholds) -> bool:
    if o.distance_m is not None and thresholds.near_distance_duck_m is not None:
        return o.distance_m <= thresholds.near_distance_duck_m
    return o.area_ratio >= thresholds.near_threshold_duck


def _center_obstacles(obstacles: list[Obstacle], thresholds: CommandThresholds) -> list[Obstacle]:
    return [
        o for o in obstacles
        if thresholds.center_zone_min <= o.center_x_ratio <= thresholds.center_zone_max
    ]


def emergency_stop_required(obstacles: list[Obstacle], thresholds: CommandThresholds) -> bool:
    """True bila rintangan vertical sangat dekat perlu mundur segera.

    Fungsi ini sengaja dapat dipanggil pada deteksi mentah, sebelum tracker,
    supaya keselamatan tidak menunggu konfirmasi beberapa frame.
    """
    return any(
        o.obstacle_type == ObstacleType.VERTICAL and _is_near_stop(o, thresholds)
        for o in _center_obstacles(obstacles, thresholds)
    )


def decide_command(obstacles: list[Obstacle], thresholds: CommandThresholds) -> str:
    center_obstacles = _center_obstacles(obstacles, thresholds)

    # 1) Rintangan vertical sangat dekat selalu menang atas portal.
    if emergency_stop_required(obstacles, thresholds):
        return "mundur"

    # 2) Rintangan VERTICAL cukup dekat di jalur tengah -> belok ke sisi
    # yang paling lapang. Hanya vertical yang dihitung: portal horizontal
    # tidak boleh membelokkan pilihan kiri/kanan.
    vertical_center = [
        o for o in center_obstacles
        if o.obstacle_type == ObstacleType.VERTICAL and _is_near_turn(o, thresholds)
    ]
    if vertical_center:
        vertical_obstacles = [o for o in obstacles if o.obstacle_type == ObstacleType.VERTICAL]
        left_load = sum(o.area_ratio for o in vertical_obstacles if o.center_x_ratio < 0.5)
        right_load = sum(o.area_ratio for o in vertical_obstacles if o.center_x_ratio >= 0.5)
        return "kanan" if left_load >= right_load else "kiri"

    # 3) HORIZONTAL -> merunduk hanya setelah lintasan aman dari vertical.
    for o in center_obstacles:
        if (
            o.obstacle_type == ObstacleType.HORIZONTAL
            and o.center_y_ratio <= thresholds.top_zone_max
            and _is_near_duck(o, thresholds)
        ):
            return "merunduk"

    # 4) Default -> maju
    return "maju"
