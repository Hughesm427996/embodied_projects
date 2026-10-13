# ----------------------------------------------------------------------------
# Copyright (c) 2021-2026 DexForce Technology Co., Ltd.
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#     http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.
# ----------------------------------------------------------------------------

from __future__ import annotations

import os
from typing import Any

import cv2
import numpy as np
import yaml

from embodichain.utils.logger import log_warning

__all__ = ["StereoRectify"]


class StereoRectify:
    def __init__(
        self,
        height: int = 1024,
        width: int = 1280,
        calib_file: str | None = None,
        calib_dict: dict[str, Any] | None = None,
    ) -> None:
        """Initialize stereo rectification from calibration parameters."""
        self.cam_k1: np.ndarray | None = None
        self.cam_k2: np.ndarray | None = None
        self.rect_cam_k: np.ndarray | None = None
        self.baseline: float | None = None
        self.map_x_1: np.ndarray | None = None
        self.map_y_1: np.ndarray | None = None
        self.map_x_2: np.ndarray | None = None
        self.map_y_2: np.ndarray | None = None

        if calib_file and os.path.exists(calib_file):
            self.set_rectify_params(height, width, calib_file, calib_dict)
        else:
            log_warning(f"calib_file path is invalid or not found: {calib_file}.")
            self.set_rectify_params(height, width, None, calib_dict)

    def set_rectify_params(
        self,
        height: int,
        width: int,
        calib_file: str | None,
        calib_dict: dict[str, Any] | None,
    ) -> None:
        """Load calibration parameters and precompute stereo rectification maps."""
        if calib_dict is None:
            if calib_file is None:
                raise ValueError("Either calib_file or calib_dict must be provided.")
            with open(calib_file, "r", encoding="utf-8") as f:
                stereo_res = yaml.load(stream=f, Loader=yaml.FullLoader)
        else:
            stereo_res = calib_dict

        self.cam_k1 = np.array(stereo_res["cam1_k"])
        self.cam_k2 = np.array(stereo_res["cam2_k"])

        cam1_dist = np.array(stereo_res["dist_1"]).reshape(-1)
        cam2_dist = np.array(stereo_res["dist_2"]).reshape(-1)
        R = np.array(stereo_res["R_l_r"])
        t = np.array(stereo_res["t_l_r"])

        R1, R2, P1, P2, _, _, _ = cv2.stereoRectify(
            cameraMatrix1=self.cam_k1,
            distCoeffs1=cam1_dist,
            cameraMatrix2=self.cam_k2,
            distCoeffs2=cam2_dist,
            imageSize=(width, height),
            R=R,
            T=t,
            flags=1024,
            newImageSize=(0, 0),
        )

        rectified_image_size = (width, height)

        map_x_1, map_y_1 = cv2.initUndistortRectifyMap(
            cameraMatrix=self.cam_k1,
            distCoeffs=np.zeros((1, 5)),
            R=R1,
            newCameraMatrix=P1,
            size=rectified_image_size,
            m1type=cv2.CV_32FC1,
        )
        map_x_2, map_y_2 = cv2.initUndistortRectifyMap(
            cameraMatrix=self.cam_k2,
            distCoeffs=np.zeros((1, 5)),
            R=R2,
            newCameraMatrix=P2,
            size=rectified_image_size,
            m1type=cv2.CV_32FC1,
        )

        self.rect_cam_k = P2[:3, :3]
        self.baseline = -P2[0, 3] / P2[0, 0] * 0.001
        self.map_x_1 = map_x_1
        self.map_y_1 = map_y_1
        self.map_x_2 = map_x_2
        self.map_y_2 = map_y_2

    def rectify_imgs(
        self,
        left_image: np.ndarray,
        right_image: np.ndarray,
    ) -> tuple[np.ndarray, np.ndarray]:
        message = (
            "Detect you have not call set_rectify_params before do actual "
            "rectification. Please initialize StereoRectify with calib_file "
            "or call set_rectify_params directly first."
        )
        assert (
            self.map_x_1 is not None
            and self.map_y_1 is not None
            and self.map_x_2 is not None
            and self.map_y_2 is not None
        ), message

        left_image = cv2.remap(
            left_image, self.map_x_1, self.map_y_1, interpolation=cv2.INTER_LINEAR
        )
        right_image = cv2.remap(
            right_image, self.map_x_2, self.map_y_2, interpolation=cv2.INTER_LINEAR
        )
        return left_image, right_image
