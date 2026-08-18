from dataclasses import dataclass

import numpy as np


@dataclass(frozen=True)
class MarkerPose:
    rvec: np.ndarray
    tvec: np.ndarray
    rotation: np.ndarray
    normal: np.ndarray


@dataclass(frozen=True)
class CubeEstimate:
    marker_id: int
    center: np.ndarray
    normal: np.ndarray
    rotation: np.ndarray
    face_count: int
