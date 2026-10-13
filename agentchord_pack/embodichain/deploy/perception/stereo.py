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

import numpy as np
import torch

__all__ = ["left_right_to_depth"]


@torch.no_grad()
def left_right_to_depth(
    left_img: np.ndarray,
    right_img: np.ndarray,
    model,
    fx: float,
    baseline: float,
    scale: float = 1.0,
    max_disp: int = 416,
    iters: int = 16,
    threshold: float = 0.0,
) -> tuple[np.ndarray, np.ndarray]:
    """Estimate metric depth from a rectified stereo pair."""
    if left_img.shape != right_img.shape:
        raise ValueError("left_img and right_img must have the same shape.")
    if left_img.ndim != 3 or left_img.shape[2] != 3:
        raise ValueError("Stereo images must have shape (H, W, 3).")

    height, width, _ = left_img.shape
    if scale != 1.0:
        import cv2

        left_input = cv2.resize(left_img, (0, 0), fx=scale, fy=scale)
        right_input = cv2.resize(right_img, (0, 0), fx=scale, fy=scale)
    else:
        left_input, right_input = left_img, right_img

    device = torch.device("cuda" if torch.cuda.is_available() else "cpu")
    left_tensor = (
        torch.from_numpy(left_input).float().permute(2, 0, 1).unsqueeze(0).to(device)
    )
    right_tensor = (
        torch.from_numpy(right_input).float().permute(2, 0, 1).unsqueeze(0).to(device)
    )

    disp = model.inference(
        left_tensor,
        right_tensor,
        max_disp=max_disp,
        iters=iters,
        threshold=threshold,
    )
    disp = disp.squeeze().detach().cpu().numpy()

    if scale != 1.0:
        import cv2

        disp = cv2.resize(
            disp / scale,
            (width, height),
            interpolation=cv2.INTER_NEAREST,
        )

    yy, xx = np.meshgrid(np.arange(height), np.arange(width), indexing="ij")
    disp[(xx - disp) < 0] = 0.0

    depth = np.zeros_like(disp, dtype=np.float32)
    valid = disp > 0
    depth[valid] = fx * baseline / disp[valid]
    return depth, disp
