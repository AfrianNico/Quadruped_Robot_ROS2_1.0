"""Tracker ringan antar-frame untuk hasil deteksi obstacle.

Ini bukan object tracker visual penuh; fungsinya menahan false positive
sesaat dan menghindari command kembali ke ``maju`` ketika YOLO melewatkan
sebuah obstacle hanya satu atau dua frame.
"""

from __future__ import annotations

from dataclasses import dataclass
from time import monotonic

from vision_avoidance_ros2.obstacle_classifier import Obstacle


def _iou(a: Obstacle, b: Obstacle) -> float:
    ax, ay, aw, ah = a.bbox
    bx, by, bw, bh = b.bbox
    left, top = max(ax, bx), max(ay, by)
    right, bottom = min(ax + aw, bx + bw), min(ay + ah, by + bh)
    intersection = max(0, right - left) * max(0, bottom - top)
    union = aw * ah + bw * bh - intersection
    return intersection / union if union > 0 else 0.0


@dataclass
class _Track:
    obstacle: Obstacle
    consecutive_hits: int
    last_seen: float
    confirmed: bool = False


class ObstacleTracker:
    """Konfirmasi deteksi konsisten dan tahan track saat miss singkat."""

    def __init__(
        self,
        confirmation_frames: int = 3,
        hold_seconds: float = 0.30,
        min_iou: float = 0.10,
    ) -> None:
        self._confirmation_frames = max(1, int(confirmation_frames))
        self._hold_seconds = max(0.0, float(hold_seconds))
        self._min_iou = min(1.0, max(0.0, float(min_iou)))
        self._tracks: list[_Track] = []

    def update(self, detections: list[Obstacle], now: float | None = None) -> list[Obstacle]:
        now = monotonic() if now is None else now
        used_track_indices: set[int] = set()

        for detection in detections:
            candidate_indices = [
                index
                for index, track in enumerate(self._tracks)
                if index not in used_track_indices
                and track.obstacle.class_name == detection.class_name
                and track.obstacle.obstacle_type == detection.obstacle_type
            ]
            best_index = max(
                candidate_indices,
                key=lambda index: _iou(self._tracks[index].obstacle, detection),
                default=None,
            )
            if best_index is not None and _iou(self._tracks[best_index].obstacle, detection) >= self._min_iou:
                track = self._tracks[best_index]
                track.obstacle = detection
                track.consecutive_hits += 1
                track.last_seen = now
                track.confirmed = track.consecutive_hits >= self._confirmation_frames
                used_track_indices.add(best_index)
            else:
                self._tracks.append(_Track(detection, consecutive_hits=1, last_seen=now))
                used_track_indices.add(len(self._tracks) - 1)

        # Miss satu frame tidak langsung menghapus obstacle yang sudah
        # dikonfirmasi, tetapi hit berikutnya harus kembali konsisten dari
        # awal bila objek muncul lagi setelah terputus.
        for index, track in enumerate(self._tracks):
            if index not in used_track_indices:
                track.consecutive_hits = 0

        self._tracks = [
            track for track in self._tracks if now - track.last_seen <= self._hold_seconds
        ]
        return [
            track.obstacle
            for track in self._tracks
            if track.confirmed
        ]

    @property
    def active_track_count(self) -> int:
        return len(self._tracks)
