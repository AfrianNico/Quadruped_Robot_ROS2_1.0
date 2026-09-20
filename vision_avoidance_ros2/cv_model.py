"""
cv_model.py

Wrapper inferensi model dengan DUA backend otomatis, tergantung platform:
  - Orange Pi 5 Pro / ARM (aarch64, arm64) -> RKNNLite (pakai NPU RK3588)
  - Platform lain (x86, buat development/testing di laptop)  -> ONNX Runtime (CPU).
"""

from __future__ import annotations

import logging
import platform
from typing import Any, Optional

import numpy as np

try:
    from rknnlite.api import RKNNLite  # type: ignore
except ImportError:  # pragma: no cover - platform specific
    RKNNLite = None

try:
    import onnxruntime as ort  # type: ignore
except ImportError:  # pragma: no cover - platform specific
    ort = None


class CvModel:
    """Wrapper pemuatan model & runtime inferensi (RKNN di ARM, ONNX di lainnya)."""

    def __init__(
        self,
        path: str,
        logger: logging.Logger,
        *,
        core_mask: int = 0,
        rknn_lite_cls: Optional[type] = None,
        ort_module: Any | None = None,
    ) -> None:
        self.path = path
        self.core_mask = core_mask
        self.logger = logger
        self.machine = platform.machine().lower()
        self._is_arm_runtime = self.machine in ("aarch64", "arm64")

        self._rknn_lite_cls = rknn_lite_cls if rknn_lite_cls is not None else RKNNLite
        self._ort_module = ort_module if ort_module is not None else ort

        self._rknn_model: Any | None = None
        self._ort_session: Any | None = None
        self._ort_input_names: list[str] = []
        self._ort_output_names: list[str] = []
        self._ort_input_types: list[str] = []
        self._ort_input_shapes: list[list[Any]] = []

        if self._is_arm_runtime:
            if self._rknn_lite_cls is None:
                raise RuntimeError(
                    "RKNNLite tidak tersedia di platform ARM ini. "
                    "Install rknn-toolkit-lite2 sesuai README."
                )
            self._rknn_model = self._rknn_lite_cls()
        else:
            if self._ort_module is None:
                raise RuntimeError(
                    "onnxruntime tidak tersedia di platform non-ARM ini. "
                    "Install dengan: pip install onnxruntime"
                )

    def initialize(self) -> None:
        if self._is_arm_runtime:
            self.path += ".rknn"
            assert self._rknn_model is not None

            ret = self._rknn_model.load_rknn(self.path)
            if ret != 0:
                raise RuntimeError(f"Gagal load file RKNN: {self.path}, code={ret}")

            ret = self._rknn_model.init_runtime(core_mask=self.core_mask)
            if ret != 0:
                raise RuntimeError(f"Gagal init runtime RKNN: {self.path}, code={ret}")

            self.logger.info(f"Model RKNN siap (NPU): {self.path}")
        else:
            self.path += ".onnx"
            ort_module: Any = self._ort_module
            session = ort_module.InferenceSession(
                self.path, providers=["CPUExecutionProvider"]
            )
            self._ort_session = session
            self._ort_input_names = [n.name for n in session.get_inputs()]
            self._ort_input_types = [n.type for n in session.get_inputs()]
            self._ort_input_shapes = [list(n.shape) for n in session.get_inputs()]
            self._ort_output_names = [n.name for n in session.get_outputs()]

            self.logger.info(f"Model ONNX siap (CPU): {self.path}")

    @staticmethod
    def _guess_layout(shape: list[Any]) -> str | None:
        if len(shape) != 4:
            return None
        c_at_1 = shape[1] if isinstance(shape[1], int) else None
        c_at_3 = shape[3] if isinstance(shape[3], int) else None
        if c_at_1 in (1, 3):
            return "nchw"
        if c_at_3 in (1, 3):
            return "nhwc"
        return None

    @staticmethod
    def _cast_for_onnx(input_array: np.ndarray, expected_type: str) -> np.ndarray:
        if expected_type == "tensor(float)":
            if input_array.dtype != np.float32:
                converted = input_array.astype(np.float32)
                if np.issubdtype(input_array.dtype, np.integer):
                    converted /= 255.0
                return converted
            return input_array
        if expected_type == "tensor(float16)":
            return input_array.astype(np.float16)
        if expected_type == "tensor(int64)":
            return input_array.astype(np.int64)
        if expected_type == "tensor(int32)":
            return input_array.astype(np.int32)
        if expected_type == "tensor(uint8)":
            return input_array.astype(np.uint8)
        return input_array

    @property
    def is_arm_runtime(self) -> bool:
        return self._is_arm_runtime

    @property
    def preferred_data_format(self) -> str:
        return "nhwc" if self._is_arm_runtime else "nchw"

    def infer(self, inputs: list[np.ndarray], data_format: str = "nhwc") -> list[np.ndarray]:
        if self._is_arm_runtime:
            if self._rknn_model is None:
                raise RuntimeError(f"RKNN runtime belum diinisialisasi: {self.path}")
            return self._rknn_model.inference(inputs=inputs, data_format=data_format)

        if self._ort_session is None:
            raise RuntimeError(f"ONNX runtime belum diinisialisasi: {self.path}")

        feed: dict[str, np.ndarray] = {}
        for idx, name in enumerate(self._ort_input_names):
            if idx >= len(inputs):
                continue
            array = np.asarray(inputs[idx])
            expected_type = self._ort_input_types[idx] if idx < len(self._ort_input_types) else ""
            expected_shape = self._ort_input_shapes[idx] if idx < len(self._ort_input_shapes) else []

            input_layout = data_format.lower()
            expected_layout = self._guess_layout(expected_shape)
            if array.ndim == 4 and input_layout in ("nhwc", "nchw") and expected_layout in ("nhwc", "nchw"):
                if input_layout == "nhwc" and expected_layout == "nchw":
                    array = np.transpose(array, (0, 3, 1, 2))
                elif input_layout == "nchw" and expected_layout == "nhwc":
                    array = np.transpose(array, (0, 2, 3, 1))

            array = self._cast_for_onnx(array, expected_type)
            feed[name] = np.ascontiguousarray(array)

        if not feed:
            raise RuntimeError(f"Tidak ada input ONNX yang valid untuk model: {self.path}")

        outputs = self._ort_session.run(self._ort_output_names, feed)
        return [np.asarray(o) for o in outputs]

    def release(self) -> None:
        if self._is_arm_runtime:
            try:
                if self._rknn_model is not None:
                    self._rknn_model.release()
            except Exception:
                self.logger.error(f"Gagal release model RKNN '{self.path}'", exc_info=True)
            return
        self._ort_session = None
        self._ort_input_names = []
        self._ort_output_names = []
