"""
distance_estimator.py

Ground-plane projection: mengestimasi JARAK METRIK SUNGGUHAN (meter) --
bukan lagi cuma proxy area_ratio -- dari posisi vertikal (y pixel) sisi
BAWAH bounding box rintangan.

Diadaptasi dari parameter `vision.distance.*` pada vision_node.py milik
tim lomba kapal Anda (kamera kapal menghadap ke arah air/permukaan
dengan ketinggian & sudut tetap -- prinsip geometrinya identik dengan
kamera robot quadruped yang menghadap lantai).

=====================================================================
ASUMSI & BATASAN PENTING (WAJIB dipahami sebelum dipakai)
=====================================================================
Formula ini HANYA VALID untuk objek yang MENYENTUH TANAH/LANTAI --
karena asumsi dasarnya adalah "titik yang diproyeksikan berada persis
di atas permukaan datar (ground plane)".

  - COCOK untuk rintangan VERTICAL (tiang, kaki kursi, kaki meja, kaki
    manusia) -- semuanya berpijak di lantai, jadi sisi BAWAH bounding
    box-nya bisa dianggap valid berada di ground plane.
  - TIDAK VALID untuk rintangan HORIZONTAL yang melayang (palang, dahan,
    tepi meja setinggi dada) -- dasarnya TIDAK menyentuh lantai, kalau
    dipaksa dihitung pakai formula ini hasilnya akan SALAH/menyesatkan.

Karena itu, di vision_node.py, distance_m HANYA dipakai untuk override
keputusan pada rintangan VERTICAL. Rintangan HORIZONTAL tetap memakai
area_ratio seperti sebelumnya -- lihat command_logic.py.

=====================================================================
Cara kerja geometri
=====================================================================
Kamera terpasang pada ketinggian `camera_height` (meter) dari lantai,
dimiringkan ke bawah sejumlah `camera_tilt_deg` derajat dari horizontal.
Untuk baris pixel y (posisi vertikal di frame):

  alpha         = sudut baris tsb relatif terhadap sumbu optik kamera
                = atan((y - principal_point_y) / focal_length_y)
  total_angle   = camera_tilt_deg (radian) + alpha
  distance      = camera_height / tan(total_angle)

Kalau total_angle <= 0, artinya baris itu ada DI ATAS garis horizon --
tidak mungkin menyentuh lantai pada jarak berapa pun -- return None.
"""

from __future__ import annotations

import math
from dataclasses import dataclass


@dataclass(frozen=True)
class CameraGeometry:
    """Parameter kalibrasi geometri kamera untuk ground-plane projection."""

    camera_height: float       # meter, tinggi kamera dari lantai
    camera_tilt_deg: float     # derajat, sudut kemiringan ke bawah (0 = lurus horizontal)
    focal_length_y: float      # pixel, focal length vertikal
    principal_point_y: float   # pixel, titik utama vertikal (umumnya ~ frame_height / 2)

    @property
    def tilt_rad(self) -> float:
        return math.radians(self.camera_tilt_deg)

    @property
    def horizon_y(self) -> float:
        """
        Baris pixel tempat garis horizon (jarak tak-hingga) diproyeksikan.
        Berguna untuk KALIBRASI: arahkan kamera, ukur baris pixel horizon
        sungguhan di gambar, lalu sesuaikan camera_tilt_deg sampai nilai
        ini cocok dengan yang terlihat -- lihat panduan kalibrasi di README.
        """
        return self.principal_point_y - self.focal_length_y * math.tan(self.tilt_rad)


def estimate_ground_distance(y_pixel: float, geometry: CameraGeometry) -> float | None:
    """
    Estimasi jarak (meter) dari kamera ke titik lantai yang diproyeksikan
    pada baris pixel y_pixel -- gunakan sisi BAWAH bounding box rintangan
    (bukan titik tengah), karena itu titik yang paling dekat dengan lantai.

    Return None kalau y_pixel berada di atas garis horizon (tidak valid
    secara geometris -- lihat docstring modul).
    """
    alpha = math.atan2(y_pixel - geometry.principal_point_y, geometry.focal_length_y)
    total_angle = geometry.tilt_rad + alpha

    if total_angle <= 1e-6:
        return None

    return geometry.camera_height / math.tan(total_angle)
