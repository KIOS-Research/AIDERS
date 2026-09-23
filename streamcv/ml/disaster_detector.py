"""EfficientNet-B0 whole-frame disaster classifier for the "Disaster
Classification" detection type.

Ported from cvn/app/Disaster_Classification/Predict_Images.py (the legacy
``DisasterClassification`` class), which fine-tunes a torchvision
EfficientNet-B0 backbone with a custom classifier head into a 4-class
softmax over Earthquake/Fire/Flood/Normal. This is a whole-frame image
classifier with no spatial/localization component at all -- no bounding
boxes, no per-pixel density map -- so ``predict()`` returns a plain
probability dict rather than an ``sv.Detections`` object; there is nothing
to draw a box or dot around.

Differences from the legacy Predict_Images.py implementation:
  * Frames come directly from the in-memory frame the caller already has
    (passed into predict()) -- no ``PIL.Image.open()`` against a
    DB-fetched file path.
  * Device is selected via ``torch.cuda.is_available()``, matching
    CrowdDetector's device-selection idiom. The legacy code here already
    had a CPU fallback (unlike crowd_loc's unconditional ``.cuda()``), just
    via a more verbose ``check_gpu_cuda()`` helper with redundant duplicate
    ``.cuda()`` calls left in the GPU branch -- both simplified away here.
  * Real exceptions are allowed to surface -- no bare except/continue.
"""

from __future__ import annotations

import logging
import os
from collections import OrderedDict

import cv2
import numpy as np
import torch
import torch.nn as nn
from PIL import Image
from torchvision import models, transforms

logger = logging.getLogger(__name__)

# Exact class order from the legacy checkpoint's class_to_idx mapping
# (verified against the actual .pth file, not just the legacy source
# comment): {'Earthquake': 0, 'Fire': 1, 'Flood': 2, 'Normal': 3}.
CLASS_NAMES: list[str] = ["Earthquake", "Fire", "Flood", "Normal"]

_DEFAULT_WEIGHTS_PATH = os.path.join(
    "/app/disaster_classification", "efficientnet_model.pth"
)

# EfficientNet-B0's native input resolution -- fixed by the architecture/
# fine-tuning, independent of any other detector's canvas convention.
_MODEL_INPUT_SIZE = 224

# Fixed canvas used only for video recording/streaming display, since
# cv2.VideoWriter needs a size decided in advance at stream-start time and
# this model (unlike WaldoDetector/CrowdDetector) has no natural canvas of
# its own -- it classifies the whole frame regardless of input resolution.
# Reuses the same 1280x736 convention as the other two detectors purely for
# consistency; this has no bearing on model accuracy, since predict() always
# does its own independent resize down to 224x224 regardless of what
# resolution the frame it's given is in.
_DISPLAY_WIDTH = 1280
_DISPLAY_HEIGHT = 736

# ImageNet normalization stats -- what the checkpoint was fine-tuned with.
_NORM_MEAN = [0.485, 0.456, 0.406]
_NORM_STD = [0.229, 0.224, 0.225]


class DisasterDetector:
    """Thin wrapper around the legacy EfficientNet-B0 disaster classifier.

    Parameters
    ----------
    weights_path:
        Filesystem path to the ``.pth`` checkpoint.
    """

    def __init__(self, weights_path: str = _DEFAULT_WEIGHTS_PATH) -> None:
        self._weights_path = weights_path
        self._device = torch.device("cuda" if torch.cuda.is_available() else "cpu")
        self._model = self._load_model(weights_path)
        self._transform = transforms.Compose([
            transforms.Resize((_MODEL_INPUT_SIZE, _MODEL_INPUT_SIZE)),
            transforms.ToTensor(),
            transforms.Normalize(_NORM_MEAN, _NORM_STD),
        ])

        logger.info(
            "DisasterDetector ready  model=EfficientNet-B0  weights=%s  input=%dx%d  device=%s  gpu=%s",
            os.path.basename(weights_path),
            _MODEL_INPUT_SIZE,
            _MODEL_INPUT_SIZE,
            self._device,
            self._device.type == "cuda",
        )
        if self._device.type != "cuda":
            logger.warning(
                "CUDA is not available; EfficientNet-B0 inference will run on CPU"
            )

    # ------------------------------------------------------------------
    # Model loading
    # ------------------------------------------------------------------

    def _load_model(self, weights_path: str) -> nn.Module:
        model = models.efficientnet_b0(pretrained=False)
        for param in model.parameters():
            param.requires_grad = False

        in_features = model.classifier[-1].in_features
        model.classifier = nn.Sequential(OrderedDict([
            ("fc1", nn.Linear(in_features, 512)),
            ("relu", nn.ReLU()),
            ("drop", nn.Dropout(p=0.5)),
            ("fc2", nn.Linear(512, len(CLASS_NAMES))),
            ("output", nn.LogSoftmax(dim=1)),
        ]))

        if os.path.isfile(weights_path):
            checkpoint = torch.load(weights_path, map_location=self._device)
            model.load_state_dict(checkpoint["model_state_dict"])
            logger.info("Loaded EfficientNet-B0 checkpoint from %s", weights_path)
        else:
            logger.warning(
                "EfficientNet-B0 weights not found at %s; model is randomly initialized",
                weights_path,
            )

        model = model.to(self._device)
        model.eval()
        return model

    # ------------------------------------------------------------------
    # Preprocessing
    # ------------------------------------------------------------------

    def _preprocess(self, frame: np.ndarray) -> torch.Tensor:
        # frame is BGR (OpenCV convention) -- convert to RGB before handing
        # it to PIL, matching legacy's Image.open(...).convert("RGB").
        rgb = cv2.cvtColor(frame, cv2.COLOR_BGR2RGB)
        pil_image = Image.fromarray(rgb)
        tensor = self._transform(pil_image).unsqueeze(0)
        return tensor.to(self._device)

    # ------------------------------------------------------------------
    # Public API
    # ------------------------------------------------------------------

    def predict(self, frame: np.ndarray) -> dict[str, float]:
        """Classify a BGR frame into disaster-class probabilities.

        Parameters
        ----------
        frame:
            The raw camera frame, at whatever resolution the caller has it
            in -- no pre-resize/letterbox is expected, unlike the box-based
            detectors' shared canvas convention. This model does its own
            independent resize to 224x224 internally.

        Returns
        -------
        dict[str, float]
            Percentage probability (0-100, rounded to 2 decimals) for each
            of ``Earthquake``, ``Fire``, ``Flood`` -- ``Normal`` is dropped
            before returning, matching legacy behavior (only the
            "this frame might show a disaster" classes are ever persisted
            or displayed).
        """
        tensor = self._preprocess(frame)

        with torch.no_grad():
            output = self._model(tensor)
            probabilities = torch.exp(output)[0]  # LogSoftmax -> real probabilities

        return {
            class_name: round(probabilities[i].item() * 100, 2)
            for i, class_name in enumerate(CLASS_NAMES)
            if class_name != "Normal"
        }

    @property
    def input_size(self) -> tuple[int, int]:
        """``(width, height)`` of the fixed display/recording canvas.

        Unrelated to the model's actual 224x224 inference resolution --
        exists only so the caller can size a ``cv2.VideoWriter`` up front,
        matching WaldoDetector/CrowdDetector's ``input_size`` contract.
        """
        return _DISPLAY_WIDTH, _DISPLAY_HEIGHT
