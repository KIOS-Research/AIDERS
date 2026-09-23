"""ONNX YOLOv7 detector with a ``supervision.Detections`` output bridge.

The preprocessing, inference, and output parsing logic is ported verbatim
from ``cvn/app/waldo/waldo_run.py`` (the legacy *ObjectDetector* class).
The only structural change is that ``predict()`` returns a
:class:`supervision.Detections` object instead of drawing on the frame
directly -- annotation is now the caller's responsibility via
``sv.BoundingBoxAnnotator`` / ``sv.TraceAnnotator``.
"""

from __future__ import annotations

import logging
import os
import re
from dataclasses import dataclass

import cv2
import numpy as np
import onnxruntime as rt
import supervision as sv

logger = logging.getLogger(__name__)

# Exact class list from the legacy model (order = class_id)
CLASS_NAMES: list[str] = [
    "car",
    "van",
    "truck",
    "building",
    "human",
    "gastank",
    "digger",
    "container",
    "bus",
    "u_pole",
    "boat",
    "bike",
    "smoke",
    "solarpanels",
    "arm",
    "plane",
]

_DEFAULT_MODEL_PATH = os.path.join(
    "/app/waldo",
    "yolov7-W25_rect_1280_736_newDefaults-bs96-best-topk-200.onnx",
)


class WaldoDetector:
    """Thin wrapper around the legacy ONNX YOLOv7 model.

    Parameters
    ----------
    model_path:
        Filesystem path to the ``.onnx`` file.
    """

    def __init__(self, model_path: str = _DEFAULT_MODEL_PATH) -> None:
        self._model_path = model_path
        self._expected_w, self._expected_h = self._resolution_from_path(model_path)
        self._sess, self._input_name, requested_providers, available_providers = self._init_session(model_path)
        active_providers = self._sess.get_providers()
        uses_cuda = "CUDAExecutionProvider" in active_providers
        logger.info(
            "WaldoDetector ready  model=%s  input=%dx%d  providers=%s  requested=%s  available=%s  gpu=%s",
            os.path.basename(model_path),
            self._expected_w,
            self._expected_h,
            active_providers,
            requested_providers,
            available_providers,
            uses_cuda,
        )

        if not uses_cuda:
            logger.warning(
                "ONNX Runtime is not using CUDAExecutionProvider; inference will run on CPU"
            )

    # ------------------------------------------------------------------
    # Session / model helpers
    # ------------------------------------------------------------------

    @staticmethod
    def _init_session(model_path: str) -> tuple[rt.InferenceSession, str, list[str], list[str]]:
        available_providers = rt.get_available_providers()

        env_providers = os.getenv("STREAMCV_ORT_PROVIDERS", "").strip()
        if env_providers:
            requested = [p.strip() for p in env_providers.split(",") if p.strip()]
            providers = [p for p in requested if p in available_providers]
            if "CPUExecutionProvider" in available_providers and "CPUExecutionProvider" not in providers:
                providers.append("CPUExecutionProvider")
        else:
            providers = ["CPUExecutionProvider"]
            if "CUDAExecutionProvider" in available_providers:
                providers = ["CUDAExecutionProvider", "CPUExecutionProvider"]

        sess = rt.InferenceSession(model_path, providers=providers)
        input_name = sess.get_inputs()[0].name
        return sess, input_name, providers, available_providers

    @staticmethod
    def _resolution_from_path(path: str) -> tuple[int, int]:
        rect = re.search(r"_rect_(\d+)_(\d+)_", path)
        if rect:
            return int(rect.group(1)), int(rect.group(2))
        square = re.search(r"(\d+)px", path)
        if square:
            dim = int(square.group(1))
            return dim, dim
        raise ValueError(f"Cannot infer resolution from model path: {path}")

    # ------------------------------------------------------------------
    # Preprocessing  (from ObjectDetector.resize_and_pad)
    # ------------------------------------------------------------------

    def _resize_and_pad(self, frame: np.ndarray) -> np.ndarray:
        ew, eh = self._expected_w, self._expected_h

        if ew == eh:
            h, w = frame.shape[:2]
            dim = min(h, w)
            sx = (w - dim) // 2
            sy = (h - dim) // 2
            frame = frame[sy : sy + dim, sx : sx + dim]

        ratio = min(ew / frame.shape[1], eh / frame.shape[0])
        nw = int(frame.shape[1] * ratio)
        nh = int(frame.shape[0] * ratio)
        frame = cv2.resize(frame, (nw, nh))

        padded = np.zeros((eh, ew, 3), dtype=np.uint8)
        y_off = (eh - nh) // 2
        x_off = (ew - nw) // 2
        padded[y_off : y_off + nh, x_off : x_off + nw] = frame
        return padded

    # ------------------------------------------------------------------
    # Public API
    # ------------------------------------------------------------------

    def predict(self, frame: np.ndarray) -> tuple[sv.Detections, np.ndarray]:
        """Run detection on a BGR frame.

        Returns
        -------
        tuple[sv.Detections, np.ndarray]
            * **detections** -- bounding boxes in the *padded* model
              coordinate space (1280x736) with ``xyxy``, ``confidence``,
              and ``class_id`` fields.
            * **preprocessed** -- the padded BGR frame (1280x736) that the
              boxes correspond to.  Use this for annotation and streaming
              so that boxes and pixels are in the same coordinate space.
        """
        preprocessed = self._resize_and_pad(frame)

        blob = preprocessed.transpose((2, 0, 1))        # HWC -> CHW
        blob = np.expand_dims(blob, 0)                   # add batch dim
        blob = np.ascontiguousarray(blob, dtype=np.float32) / 255.0

        outputs = self._sess.run(None, {self._input_name: blob})[0]

        if len(outputs) == 0:
            return sv.Detections.empty(), preprocessed

        # Legacy output format: each row is (batch_id, x0, y0, x1, y1, cls_id, score)
        xyxy = outputs[:, 1:5].astype(np.float32)
        confidence = outputs[:, 6].astype(np.float32)
        class_id = outputs[:, 5].astype(int)

        detections = sv.Detections(
            xyxy=xyxy,
            confidence=confidence,
            class_id=class_id,
        )
        return detections, preprocessed

    @property
    def input_size(self) -> tuple[int, int]:
        """``(width, height)`` the model expects after padding."""
        return self._expected_w, self._expected_h
