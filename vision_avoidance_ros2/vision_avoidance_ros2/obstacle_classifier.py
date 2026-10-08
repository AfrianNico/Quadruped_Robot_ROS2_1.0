"""
obstacle_classifier.py

Klasifikasi tipe rintangan (VERTICAL/HORIZONTAL) -- strategi 2 lapis:
  1. Cek nama kelas hasil deteksi YOLO di class_mapping (dari config YAML).
  2. Fallback ke klasifikasi berbasis rasio tinggi/lebar bounding box.

Field `distance_m` opsional adalah jarak metrik hasil ground-plane
projection. Nilai ini hanya disimpan untuk rintangan yang punya referensi
di lantai: VERTICAL secara default, atau kelas eksplisit seperti `portal`
yang bounding box-nya meliputi kaki sampai lantai.
"""

from __future__ import annotations

from dataclasses import dataclass
from enum import Enum


class ObstacleType(Enum):
    VERTICAL = "vertical"
    HORIZONTAL = "horizontal"
    UNKNOWN = "unknown"


@dataclass(frozen=True)
class Obstacle:
    """Satu rintangan hasil deteksi, sudah diklasifikasi & dinormalisasi posisinya."""

    obstacle_type: ObstacleType
    bbox: tuple[int, int, int, int]  # (x, y, w, h) pixel
    area_ratio: float                 # luas bbox / luas frame (proxy kedekatan, selalu ada)
    center_x_ratio: float             # posisi tengah horizontal, 0..1
    center_y_ratio: float             # posisi tengah vertikal, 0..1
    class_name: str
    confidence: float
    distance_m: float | None = None   # jarak lantai dari kamera ke referensi objek
    has_ground_reference: bool = False


class ObstacleClassifier:
    def __init__(
        self,
        class_mapping: dict[str, str],
        ground_contact_classes: list[str] | None = None,
        vertical_aspect_ratio: float = 1.4,
        horizontal_aspect_ratio: float = 1.4,
    ) -> None:
        self._class_mapping = {
            str(k): str(v).lower() for k, v in (class_mapping or {}).items()
        }
        self._ground_contact_classes = {
            str(class_name) for class_name in (ground_contact_classes or [])
        }
        self._vertical_aspect_ratio = vertical_aspect_ratio
        self._horizontal_aspect_ratio = horizontal_aspect_ratio

    def classify(self, class_name: str, w: float, h: float) -> ObstacleType:
        mapped = self._class_mapping.get(class_name)
        if mapped == ObstacleType.VERTICAL.value:
            return ObstacleType.VERTICAL
        if mapped == ObstacleType.HORIZONTAL.value:
            return ObstacleType.HORIZONTAL

        if w <= 0 or h <= 0:
            return ObstacleType.UNKNOWN
        if (h / w) >= self._vertical_aspect_ratio:
            return ObstacleType.VERTICAL
        if (w / h) >= self._horizontal_aspect_ratio:
            return ObstacleType.HORIZONTAL
        return ObstacleType.UNKNOWN

    def has_ground_reference(self, class_name: str, w: float, h: float) -> bool:
        """True bila sisi bawah bbox adalah titik objek yang menyentuh lantai."""
        return (
            class_name in self._ground_contact_classes
            or self.classify(class_name, w, h) == ObstacleType.VERTICAL
        )

    def build_obstacle(
        self,
        bbox: tuple[int, int, int, int],
        class_name: str,
        confidence: float,
        frame_width: int,
        frame_height: int,
        distance_m: float | None = None,
    ) -> Obstacle:
        x, y, w, h = bbox
        obstacle_type = self.classify(class_name, w, h)

        frame_area = float(frame_width * frame_height)
        area_ratio = (w * h) / frame_area if frame_area > 0 else 0.0
        center_x_ratio = (x + w / 2.0) / frame_width if frame_width > 0 else 0.0
        center_y_ratio = (y + h / 2.0) / frame_height if frame_height > 0 else 0.0

        has_ground_reference = self.has_ground_reference(class_name, w, h)
        # distance_m hanya bermakna bila bbox menyertakan titik objek yang
        # menyentuh lantai. Ini memungkinkan portal tetap HORIZONTAL untuk
        # aksi "merunduk", sambil memakai kaki portal sebagai referensi jarak.
        final_distance = distance_m if has_ground_reference else None

        return Obstacle(
            obstacle_type=obstacle_type,
            bbox=bbox,
            area_ratio=area_ratio,
            center_x_ratio=center_x_ratio,
            center_y_ratio=center_y_ratio,
            class_name=class_name,
            confidence=confidence,
            distance_m=final_distance,
            has_ground_reference=has_ground_reference,
        )
