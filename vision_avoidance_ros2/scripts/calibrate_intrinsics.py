#!/usr/bin/env python3
"""Kalibrasi intrinsics kamera untuk ground-plane distance.

Contoh:
  python3 scripts/calibrate_intrinsics.py \
    --images 'calibration_images/*.jpg' --cols 9 --rows 6 --square-size-mm 25

``cols`` dan ``rows`` adalah jumlah SUDUT BAGIAN DALAM checkerboard, bukan
jumlah kotak. Foto harus diambil dengan kamera dan resolusi yang sama dengan
yang dipakai node ROS2.
"""

from __future__ import annotations

import argparse
import glob
from pathlib import Path

import cv2
import numpy as np


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--images", required=True, help="Pola glob gambar checkerboard")
    parser.add_argument("--cols", type=int, required=True, help="Jumlah sudut internal horizontal")
    parser.add_argument("--rows", type=int, required=True, help="Jumlah sudut internal vertikal")
    parser.add_argument(
        "--square-size-mm", type=float, required=True, help="Sisi satu kotak checkerboard dalam mm"
    )
    return parser.parse_args()


def main() -> None:
    args = parse_args()
    if args.cols < 2 or args.rows < 2 or args.square_size_mm <= 0:
        raise SystemExit("Ukuran checkerboard dan square-size harus positif.")

    image_paths = sorted(glob.glob(args.images))
    if not image_paths:
        raise SystemExit("Tidak ada gambar yang cocok dengan --images.")

    pattern_size = (args.cols, args.rows)
    square_size_m = args.square_size_mm / 1000.0
    object_template = np.zeros((args.cols * args.rows, 3), np.float32)
    object_template[:, :2] = np.mgrid[0 : args.cols, 0 : args.rows].T.reshape(-1, 2)
    object_template *= square_size_m

    object_points: list[np.ndarray] = []
    image_points: list[np.ndarray] = []
    image_size: tuple[int, int] | None = None

    for raw_path in image_paths:
        image = cv2.imread(raw_path)
        if image is None:
            print(f"Lewati gambar rusak: {raw_path}")
            continue
        gray = cv2.cvtColor(image, cv2.COLOR_BGR2GRAY)
        size = (gray.shape[1], gray.shape[0])
        if image_size is None:
            image_size = size
        elif image_size != size:
            print(f"Lewati {raw_path}: resolusi {size}, bukan {image_size}")
            continue

        found, corners = cv2.findChessboardCorners(
            gray,
            pattern_size,
            cv2.CALIB_CB_ADAPTIVE_THRESH + cv2.CALIB_CB_NORMALIZE_IMAGE,
        )
        if not found:
            print(f"Checkerboard tidak ditemukan: {raw_path}")
            continue

        refined = cv2.cornerSubPix(
            gray,
            corners,
            (11, 11),
            (-1, -1),
            (cv2.TERM_CRITERIA_EPS + cv2.TERM_CRITERIA_MAX_ITER, 30, 0.001),
        )
        object_points.append(object_template)
        image_points.append(refined)
        print(f"Dipakai: {raw_path}")

    if image_size is None or len(object_points) < 10:
        raise SystemExit("Butuh minimal 10 foto checkerboard yang valid; ambil 15--25 foto lebih baik.")

    rms, camera_matrix, _distortion, _rvecs, _tvecs = cv2.calibrateCamera(
        object_points, image_points, image_size, None, None
    )
    print(f"\nKalibrasi selesai: {len(object_points)} foto, RMS reprojection error={rms:.4f} px")
    print("\nSalin nilai berikut ke section distance pada config/class_mapping.yaml:")
    print(f"  focal_length_y: {camera_matrix[1, 1]:.6f}")
    print(f"  principal_point_y: {camera_matrix[1, 2]:.6f}")
    print(f"\nResolusi kalibrasi: {image_size[0]}x{image_size[1]}")
    print("Jaga resolusi node sama dengan resolusi ini saat memakai hasil kalibrasi.")


if __name__ == "__main__":
    main()
