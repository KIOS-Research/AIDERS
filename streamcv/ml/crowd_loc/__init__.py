"""HRNet+ESPCN crowd-localization network, ported from cvn/app/crowd_loc/Networks/HR_Net/.

Kept as a self-contained subpackage, separate from the WALDO/YOLO detector
code in streamcv/ml/detectors.py and streamcv/ml/crowd_detector.py, since it
brings its own config system (yacs/easydict) and network architecture.
"""

from .seg_hrnet import get_seg_model

__all__ = ["get_seg_model"]
