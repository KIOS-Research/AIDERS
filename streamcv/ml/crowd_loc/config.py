"""Ported from cvn/app/crowd_loc/Networks/HR_Net/config.py.

PRE_HR_WEIGHTS points at an ImageNet-pretrained backbone checkpoint used
only when get_seg_model(pre_train=True) is called. This codebase always
calls get_seg_model() with the default pre_train=False (CrowdDetector loads
the fully-trained model_best_u-hrnet05.pth checkpoint separately), so this
path is inert -- kept as-is for fidelity with the legacy source rather than
fabricated into something that looks functional but isn't.
"""

from easydict import EasyDict as edict

# init
__C = edict()
cfg = __C

# ------------------------------TRAIN------------------------
# __C.SEED = 1  # random seed,  for reproduction


__C.NET = "HR_Net"
__C.PRE_HR_WEIGHTS = (
    "/home/mconst12/code/Networks/HR_Net/hrnetv2_w48_imagenet_pretrained.pth"
)
