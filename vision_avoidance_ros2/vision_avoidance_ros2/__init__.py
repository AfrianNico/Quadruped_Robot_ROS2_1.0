"""
Package vision_avoidance_ros2.

Mengekspos class/fungsi paling sering dipakai ke level package, supaya
kalau package ini berkembang (mis. ada node kedua yang memakai ulang
ObstacleClassifier/CvModel untuk keperluan lain), importnya lebih ringkas.

CATATAN: vision_node.py (entry point node ROS2) TIDAK diekspos di sini
dengan sengaja -- node dipanggil ROS2 lewat entry_points di setup.py,
bukan lewat import package level.
"""

from vision_avoidance_ros2.cv_model import CvModel
from vision_avoidance_ros2.obstacle_classifier import Obstacle, ObstacleClassifier, ObstacleType
from vision_avoidance_ros2.command_logic import CommandStabilizer, CommandThresholds, decide_command
from vision_avoidance_ros2.distance_estimator import CameraGeometry, estimate_ground_distance
from vision_avoidance_ros2.obstacle_tracker import ObstacleTracker

__all__ = [
    "CvModel",
    "Obstacle",
    "ObstacleClassifier",
    "ObstacleType",
    "CommandThresholds",
    "CommandStabilizer",
    "decide_command",
    "ObstacleTracker",
    "CameraGeometry",
    "estimate_ground_distance",
]
