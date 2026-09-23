"""HRNet+ESPCN density-map person-localization detector for the "Crowd Localization"
detection type.

Ported from cvn/app/crowd_loc/crowdloc_run.py (the legacy ``Crowd_local`` class),
which used a U-HRNet backbone with an ESPCN sub-pixel upsampling head
(streamcv/ml/crowd_loc/) to produce a per-pixel confidence map, then an LMDS
(Local-Maxima Detection Strategy) post-processing step to turn that map into
discrete point localizations. This is NOT a bounding-box object detector --
there is no learned notion of an object's extent, only per-pixel confidence
and local maxima extracted from it.

This supersedes an earlier YOLOv8-CrowdHuman ONNX attempt that produced real
bounding boxes but underperformed on real people-containing footage. Kept
out of streamcv/ml/crowd_loc/ (a separate subpackage) since HRNet brings its
own config system (yacs/easydict) entirely unrelated to the WALDO ONNX code
in this file's neighbours.

Differences from the legacy cvn/app/crowd_loc/crowdloc_run.py implementation:
  * Frames come directly from the in-memory frame the caller already has
    (passed into predict()) -- no cv2.imread against a DB-fetched file path.
  * Device is selected via torch.cuda.is_available(), never an unconditional
    .cuda() call, with a startup log mirroring WaldoDetector's device-
    selection log line.
  * Real exceptions are allowed to surface -- no bare except/continue.
  * The frame is resized to a fixed canvas (see _INFERENCE_SIZE) before
    inference. The legacy code ran HRNet at the source frame's native
    resolution (no resize), which the architecture tolerates fine since it
    is fully convolutional -- but streamcv's surrounding code (video
    recording via cv2.VideoWriter, and the GPS pixel-to-lat/lon math in
    main.py) assumes a fixed, known-in-advance frame size, exactly like
    WaldoDetector's own resize-and-pad. Reusing WaldoDetector's 1280x736
    canvas keeps both detectors on the same convention and is divisible by
    32, which HRNet's multi-stage strided downsampling wants.
"""

from __future__ import annotations

import logging
import os
from collections import OrderedDict

import cv2
import numpy as np
import supervision as sv
import torch
import torch.nn.functional as F
from torchvision import transforms

from streamcv.ml.crowd_loc import get_seg_model

logger = logging.getLogger(__name__)

# Single-class output: HRNet+ESPCN localizes people only, no other classes.
CLASS_NAMES: list[str] = ["person"]

_DEFAULT_WEIGHTS_PATH = os.path.join(
    "/app/crowd_loc/weights", "model_best_u-hrnet05.pth"
)

# See module docstring: fixed canvas, matching WaldoDetector's convention.
_INFERENCE_WIDTH = 1280
_INFERENCE_HEIGHT = 736

# LMDS (Local-Maxima Detection Strategy) constants, ported verbatim from
# crowdloc_run.py's LMDS_counting.
_PEAK_SUPPRESSION_THRESHOLD = 0.1  # below this peak sigmoid value, treat frame as empty
_ADAPTIVE_THRESHOLD_FRACTION = 100.0 / 255.0

# Half-width (px) of the synthetic point-carrier box returned in each
# Detections.xyxy row. HRNet+ESPCN produces point localizations, not real
# bounding boxes -- this tiny fixed-size box exists only so the existing
# sv.Detections-shaped consumers (DotAnnotator's box-center dot placement,
# _schedule_crowd_db_write's box-center-to-lat/lon conversion in main.py)
# can be reused unchanged. It carries no size/shape information about the
# detected person.
_POINT_BOX_HALF_WIDTH = 2.0


class CrowdDetector:
    """Thin wrapper around the legacy HRNet+ESPCN PyTorch model.

    Parameters
    ----------
    weights_path:
        Filesystem path to the ``.pth`` checkpoint.
    """

    def __init__(self, weights_path: str = _DEFAULT_WEIGHTS_PATH) -> None:
        self._weights_path = weights_path
        self._device = torch.device("cuda" if torch.cuda.is_available() else "cpu")
        self._model = self._load_model(weights_path)

        logger.info(
            "CrowdDetector ready  model=HRNet+ESPCN  weights=%s  input=%dx%d  device=%s  gpu=%s",
            os.path.basename(weights_path),
            _INFERENCE_WIDTH,
            _INFERENCE_HEIGHT,
            self._device,
            self._device.type == "cuda",
        )
        if self._device.type != "cuda":
            logger.warning(
                "CUDA is not available; HRNet+ESPCN inference will run on CPU"
            )

    # ------------------------------------------------------------------
    # Model loading
    # ------------------------------------------------------------------

    def _load_model(self, weights_path: str) -> torch.nn.Module:
        model = get_seg_model()
        for param in model.parameters():
            param.requires_grad = False
        model = model.to(self._device)
        model.eval()

        if os.path.isfile(weights_path):
            checkpoint = torch.load(weights_path, map_location=self._device)
            state_dict = checkpoint["state_dict"]
            # The checkpoint was saved from a torch.nn.DataParallel-wrapped
            # model, whose state_dict keys are all prefixed with "module.".
            state_dict = OrderedDict(
                (key[len("module.") :] if key.startswith("module.") else key, value)
                for key, value in state_dict.items()
            )
            model.load_state_dict(state_dict)
            logger.info("Loaded HRNet+ESPCN checkpoint from %s", weights_path)
        else:
            logger.warning(
                "HRNet+ESPCN weights not found at %s; model is randomly initialized",
                weights_path,
            )

        return model

    # ------------------------------------------------------------------
    # Preprocessing
    # ------------------------------------------------------------------

    def _preprocess(self, frame: np.ndarray) -> tuple[torch.Tensor, np.ndarray]:
        resized = cv2.resize(frame, (_INFERENCE_WIDTH, _INFERENCE_HEIGHT))
        tensor = transforms.ToTensor()(resized).unsqueeze(0).to(self._device)
        return tensor, resized

    # ------------------------------------------------------------------
    # Postprocessing (LMDS -- ported verbatim from crowdloc_run.py)
    # ------------------------------------------------------------------

    def _lmds(self, density_map: torch.Tensor) -> tuple[int, np.ndarray]:
        """Local-Maxima Detection Strategy.

        Args:
            density_map: Raw model output (1x1xHxW).

        Returns:
            count: Number of surviving local-maxima points.
            point_map: HxW numpy array, 1 where a point was detected, else 0.
        """
        prob = torch.sigmoid(density_map)
        peak = torch.max(prob).item()

        if peak < _PEAK_SUPPRESSION_THRESHOLD:
            prob = prob * 0

        local_max = F.max_pool2d(prob, kernel_size=3, stride=1, padding=1)
        local_max = (local_max == prob).float()
        prob = local_max * prob

        threshold = _ADAPTIVE_THRESHOLD_FRACTION * peak
        prob[prob < threshold] = 0
        prob[prob > 0] = 1

        count = int(torch.sum(prob).item())
        point_map = prob.data.squeeze(0).squeeze(0).cpu().numpy()
        return count, point_map

    # ------------------------------------------------------------------
    # Public API
    # ------------------------------------------------------------------

    def predict(self, frame: np.ndarray) -> tuple[sv.Detections, np.ndarray]:
        """Run person localization on a BGR frame.

        Returns
        -------
        tuple[sv.Detections, np.ndarray]
            * **detections** -- one row per detected point, in the resized
              1280x736 canvas coordinate space. ``xyxy`` is a small
              fixed-size box centered on each point (not a real bounding
              box -- see ``_POINT_BOX_HALF_WIDTH``), ``confidence`` is
              always 1.0 (LMDS output is already binarized), and
              ``class_id`` is always 0 (person).
            * **preprocessed** -- the resized BGR frame (1280x736) that the
              points correspond to. Use this for annotation and streaming
              so that points and pixels are in the same coordinate space.
        """
        tensor, preprocessed = self._preprocess(frame)

        with torch.no_grad():
            output = self._model(tensor)
            count, point_map = self._lmds(output)

        if count == 0:
            return sv.Detections.empty(), preprocessed

        ys, xs = np.nonzero(point_map)
        xs = xs.astype(np.float32)
        ys = ys.astype(np.float32)

        xyxy = np.column_stack(
            [
                xs - _POINT_BOX_HALF_WIDTH,
                ys - _POINT_BOX_HALF_WIDTH,
                xs + _POINT_BOX_HALF_WIDTH,
                ys + _POINT_BOX_HALF_WIDTH,
            ]
        ).astype(np.float32)

        detections = sv.Detections(
            xyxy=xyxy,
            confidence=np.ones(len(xs), dtype=np.float32),
            class_id=np.zeros(len(xs), dtype=int),
        )
        return detections, preprocessed

    @property
    def input_size(self) -> tuple[int, int]:
        """``(width, height)`` of the fixed canvas frames are resized to."""
        return _INFERENCE_WIDTH, _INFERENCE_HEIGHT
