"""
metrics.py
Opsional -- diaktifkan lewat parameter ROS2 `telemetry_enabled`.
"""

from __future__ import annotations

from dataclasses import dataclass
from datetime import datetime
from pathlib import Path
from threading import RLock
from time import monotonic


def _format_rate_kbps(bytes_count: int, elapsed_seconds: float) -> float:
    if elapsed_seconds <= 0:
        return 0.0
    return (bytes_count * 8.0 / 1000.0) / elapsed_seconds


@dataclass(frozen=True)
class CvMetricsSample:
    timestamp: str
    mode: str
    latency_ms: float
    fps: float
    incoming_kbps: float
    incoming_mbps: float
    output_kbps: float
    output_mbps: float
    detected_total: int | None = None


class CvTelemetryWriter:
    """Penulis file TXT append-only untuk sesi telemetry vision."""

    def __init__(self, output_dir: str | Path) -> None:
        self._output_dir = Path(output_dir)
        self._lock = RLock()
        self._path: Path | None = None
        self._handle = None

    @property
    def path(self) -> Path | None:
        with self._lock:
            return self._path

    def _ensure_open(self) -> None:
        if self._handle is not None:
            return
        self._output_dir.mkdir(parents=True, exist_ok=True)
        timestamp = datetime.now().strftime("%d%m%Y_%H%M%S")
        self._path = self._output_dir / f"metrics_{timestamp}.txt"
        self._handle = self._path.open("a", encoding="utf-8")
        self._handle.write(f"session_start {timestamp}\n")
        self._handle.flush()

    def write_line(self, text: str) -> None:
        with self._lock:
            self._ensure_open()
            assert self._handle is not None
            self._handle.write(text.rstrip() + "\n")
            self._handle.flush()

    def write_sample(self, sample: CvMetricsSample) -> None:
        line = (
            f"ts={sample.timestamp} "
            f"mode={sample.mode} "
            f"lat={sample.latency_ms:.2f} "
            f"fps={sample.fps:.2f} "
            f"in_kbps={sample.incoming_kbps:.2f} "
            f"in_mbps={sample.incoming_mbps:.3f} "
            f"out_kbps={sample.output_kbps:.2f} "
            f"out_mbps={sample.output_mbps:.3f}"
        )
        if sample.detected_total is not None:
            line += f" detected={sample.detected_total}"
        self.write_line(line)

    def close(self) -> None:
        with self._lock:
            if self._handle is None:
                return
            self._handle.flush()
            self._handle.close()
            self._handle = None


class CvMetricsWindow:
    """Agregasi telemetry per-frame jadi sampel periodik (mis. tiap 1 detik)."""

    def __init__(self, window_seconds: float) -> None:
        self._window_seconds = max(0.1, float(window_seconds))
        self._window_start = monotonic()
        self._frame_count = 0
        self._latency_sum_ms = 0.0
        self._incoming_bytes = 0
        self._output_bytes = 0

    def add_frame(
        self,
        mode: str,
        latency_ms: float,
        incoming_bytes: int,
        output_bytes: int,
        detected_count: int | None,
    ) -> CvMetricsSample | None:
        self._frame_count += 1
        self._latency_sum_ms += latency_ms
        self._incoming_bytes += max(0, incoming_bytes)
        self._output_bytes += max(0, output_bytes)

        now = monotonic()
        elapsed = now - self._window_start
        if elapsed < self._window_seconds:
            return None

        incoming_kbps = _format_rate_kbps(self._incoming_bytes, elapsed)
        output_kbps = _format_rate_kbps(self._output_bytes, elapsed)
        sample = CvMetricsSample(
            timestamp=datetime.now().isoformat(timespec="seconds"),
            mode=mode,
            latency_ms=(self._latency_sum_ms / self._frame_count) if self._frame_count else 0.0,
            fps=(self._frame_count / elapsed) if elapsed > 0 else 0.0,
            incoming_kbps=incoming_kbps,
            incoming_mbps=incoming_kbps / 1000.0,
            output_kbps=output_kbps,
            output_mbps=output_kbps / 1000.0,
            detected_total=detected_count,
        )
        self._window_start = now
        self._frame_count = 0
        self._latency_sum_ms = 0.0
        self._incoming_bytes = 0
        self._output_bytes = 0
        return sample
