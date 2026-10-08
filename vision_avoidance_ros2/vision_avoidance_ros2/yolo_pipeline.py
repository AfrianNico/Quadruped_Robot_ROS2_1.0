"""
yolo_pipeline.py

Pipeline pra/pasca-proses untuk model YOLO, diadaptasi dari
core/tracking/pipeline.py -- digeneralisasi multi-kelas,.
"""

from __future__ import annotations

import math
from dataclasses import dataclass
from typing import Any

import cv2
import numpy as np


def letterbox_image(
    frame: np.ndarray, target_size: int
) -> tuple[np.ndarray, float, int, int]:
    """Resize frame ke target_size x target_size, pertahankan rasio aspek + padding."""
    h, w = frame.shape[:2]
    scale = min(target_size / w, target_size / h)
    new_w, new_h = int(round(w * scale)), int(round(h * scale))

    resized = cv2.resize(frame, (new_w, new_h), interpolation=cv2.INTER_LINEAR)

    pad_w = (target_size - new_w) // 2
    pad_h = (target_size - new_h) // 2

    canvas = np.full((target_size, target_size, 3), 114, dtype=np.uint8)
    canvas[pad_h:pad_h + new_h, pad_w:pad_w + new_w] = resized

    return canvas, scale, pad_w, pad_h


def prepare_yolo_input(
    frame: np.ndarray, model_input_size: int
) -> tuple[np.ndarray, float, int, int]:
    """Bangun tensor input model + metadata scaling untuk konversi balik koordinat."""
    input_image, scale, pad_w, pad_h = letterbox_image(frame, model_input_size)
    input_rgb = cv2.cvtColor(input_image, cv2.COLOR_BGR2RGB)
    input_tensor = np.expand_dims(input_rgb, axis=0)
    return input_tensor, scale, pad_w, pad_h


@dataclass(frozen=True)
class YoloParseStats:
    total_candidates: int
    dropped_bad_shape: int
    dropped_low_confidence: int
    dropped_invalid_box: int
    before_nms: int
    after_nms: int


def _parse_yolo_candidates(
    output_raw: np.ndarray,
    scale: float,
    pad_w: int,
    pad_h: int,
    frame_width: int,
    frame_height: int,
    conf_threshold: float,
    class_names: list[str],
) -> tuple[list[tuple[list[int], float, str]], int, int, int, int]:
    detections: list[tuple[list[int], float, str]] = []
    total_candidates = 0
    dropped_bad_shape = 0
    dropped_low_confidence = 0
    dropped_invalid_box = 0

    output = np.squeeze(np.asarray(output_raw))

    if output.ndim == 1:
        output = output.reshape(-1, 1)
    if output.ndim != 2:
        return detections, total_candidates, dropped_bad_shape, dropped_low_confidence, dropped_invalid_box

    num_classes = len(class_names)
    expected_features = 4 + num_classes

    if output.shape[0] != expected_features and output.shape[1] == expected_features:
        output = output.T

    for det in output.T:
        total_candidates += 1
        if det.shape[0] < 5:
            dropped_bad_shape += 1
            continue

        box = det[:4]
        class_scores = det[4:4 + num_classes] if num_classes > 0 else det[4:5]
        if class_scores.size == 0:
            dropped_bad_shape += 1
            continue

        class_id = int(np.argmax(class_scores))
        conf = float(class_scores[class_id])

        if conf <= conf_threshold:
            dropped_low_confidence += 1
            continue

        x_center, y_center, width_padded, height_padded = box
        x_center_orig = (x_center - pad_w) / scale
        y_center_orig = (y_center - pad_h) / scale
        width_orig = width_padded / scale
        height_orig = height_padded / scale

        left = max(0, int(x_center_orig - width_orig / 2))
        top = max(0, int(y_center_orig - height_orig / 2))
        width = min(int(width_orig), frame_width - left)
        height = min(int(height_orig), frame_height - top)

        if width <= 0 or height <= 0:
            dropped_invalid_box += 1
            continue

        class_name = class_names[class_id] if 0 <= class_id < num_classes else str(class_id)
        detections.append(([left, top, width, height], conf, class_name))

    return detections, total_candidates, dropped_bad_shape, dropped_low_confidence, dropped_invalid_box


def parse_yolo_output_with_stats(
    output_raw: np.ndarray,
    scale: float,
    pad_w: int,
    pad_h: int,
    frame_width: int,
    frame_height: int,
    conf_threshold: float,
    class_names: list[str],
) -> tuple[list[tuple[list[int], float, str]], YoloParseStats]:
    detections, total, bad_shape, low_conf, invalid_box = _parse_yolo_candidates(
        output_raw=output_raw,
        scale=scale,
        pad_w=pad_w,
        pad_h=pad_h,
        frame_width=frame_width,
        frame_height=frame_height,
        conf_threshold=conf_threshold,
        class_names=class_names,
    )
    before_nms = len(detections)
    return detections, YoloParseStats(
        total_candidates=total,
        dropped_bad_shape=bad_shape,
        dropped_low_confidence=low_conf,
        dropped_invalid_box=invalid_box,
        before_nms=before_nms,
        after_nms=before_nms,
    )


def filter_by_size(
    detections: list[dict[str, Any]],
    frame_shape: tuple[int, int],
    min_area: float = 900,
    max_area_ratio: float = 0.6,
) -> list[dict[str, Any]]:
    frame_area = frame_shape[0] * frame_shape[1]
    filtered: list[dict[str, Any]] = []
    for det in detections:
        x, y, w, h = det["box"]
        area = w * h
        if area < min_area:
            continue
        if area / frame_area > max_area_ratio:
            continue
        filtered.append(det)
    return filtered


def nms_by_center(
    detections: list[dict[str, Any]], max_center_dist: float = 20
) -> list[dict[str, Any]]:
    dets = sorted(detections, key=lambda d: d["conf"], reverse=True)
    kept: list[dict[str, Any]] = []
    for det in dets:
        x, y, w, h = det["box"]
        cx, cy = x + w / 2, y + h / 2
        too_close = False
        for k in kept:
            kx, ky, kw, kh = k["box"]
            kcx, kcy = kx + kw / 2, ky + kh / 2
            dist = math.sqrt((cx - kcx) ** 2 + (cy - kcy) ** 2)
            if dist < max_center_dist:
                too_close = True
                break
        if not too_close:
            kept.append(det)
    return kept
