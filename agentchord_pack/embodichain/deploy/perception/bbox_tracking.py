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

__all__ = [
    "box_contains",
    "box_iou",
    "get_multi_obj_xy_pose_from_perception",
    "get_obj_xy_pose_from_perception",
    "is_box_moved",
    "is_box_moved_simple",
]


def box_contains(box_outer, box_inner, eps: float = 0.0) -> bool:
    x1o, y1o, x2o, y2o = box_outer
    x1i, y1i, x2i, y2i = box_inner
    return (
        x1o <= x1i + eps and y1o <= y1i + eps and x2o >= x2i - eps and y2o >= y2i - eps
    )


def box_iou(a, b, eps: float = 1e-6) -> float:
    ax1, ay1, ax2, ay2 = a
    bx1, by1, bx2, by2 = b

    inter_x1 = max(ax1, bx1)
    inter_y1 = max(ay1, by1)
    inter_x2 = min(ax2, bx2)
    inter_y2 = min(ay2, by2)

    inter = max(0.0, inter_x2 - inter_x1) * max(0.0, inter_y2 - inter_y1)
    area_a = max(0.0, ax2 - ax1) * max(0.0, ay2 - ay1)
    area_b = max(0.0, bx2 - bx1) * max(0.0, by2 - by1)
    return inter / (area_a + area_b - inter + eps)


def is_box_moved(
    box_prev: np.ndarray,
    box_curr: np.ndarray,
    eps: float = 0.02,
    iou_thresh: float = 0.6,
) -> bool:
    """Return whether two boxes indicate a meaningful object displacement."""
    box_prev = box_prev.astype(np.float32)
    box_curr = box_curr.astype(np.float32)

    c_prev = 0.5 * (box_prev[:2] + box_prev[2:])
    c_curr = 0.5 * (box_curr[:2] + box_curr[2:])
    center_dist = np.linalg.norm(c_curr - c_prev)
    diag = np.linalg.norm(box_prev[2:] - box_prev[:2]) + 1e-6
    norm_dist = center_dist / diag
    return bool((norm_dist > eps) or (box_iou(box_prev, box_curr) < iou_thresh))


def is_box_moved_simple(
    box_prev: np.ndarray,
    box_curr: np.ndarray,
    pixel_thresh: float = 20.0,
    contain_eps: float = 0.0,
) -> tuple[bool, str | None, str | None]:
    """Return movement plus horizontal and vertical image-space directions."""
    if contain_eps != 0:
        if box_contains(box_prev, box_curr, eps=contain_eps) or box_contains(
            box_curr,
            box_prev,
            eps=contain_eps,
        ):
            return False, None, None

    c_prev = 0.5 * (box_prev[:2] + box_prev[2:])
    c_curr = 0.5 * (box_curr[:2] + box_curr[2:])
    delta = c_curr - c_prev
    if np.linalg.norm(delta) <= pixel_thresh:
        return False, None, None

    dx, dy = delta
    h_dir = "right" if dx > 0 else "left" if abs(dx) > 1e-6 else None
    v_dir = "down" if dy > 0 else "up" if abs(dy) > 1e-6 else None
    return True, h_dir, v_dir


def _capture_bgr(env) -> np.ndarray:
    bgr, _ = env.kingfisher.captureQuarterSize()
    return bgr


def _detect_box(env, bgr: np.ndarray, obj_name: str) -> np.ndarray:
    label = obj_name.replace("_", " ")
    for _ in range(10):
        env.predictor.set_image(bgr)
        if hasattr(env, "sam_results_dir"):
            env.predictor.save_dir = env.sam_results_dir / f"step_{env.current_step}"
        sam_results = env.predictor(text=[label])
        if (
            hasattr(sam_results[0].boxes, "data")
            and len(sam_results[0].boxes.data) != 0
        ):
            return sam_results[0].boxes.xyxy[0].cpu().numpy()
    raise RuntimeError(f"SAM failed to detect a box for {obj_name}.")


def get_obj_xy_pose_from_perception(env, obj_name: str) -> np.ndarray:
    """Detect one object and return its 2D bounding box as ``xyxy``."""
    return _detect_box(env, _capture_bgr(env), obj_name)


def get_multi_obj_xy_pose_from_perception(
    env, obj_names: list[str]
) -> list[np.ndarray]:
    """Detect several objects from one captured frame and return ``xyxy`` boxes."""
    bgr = _capture_bgr(env)
    return [_detect_box(env, bgr, obj_name) for obj_name in obj_names]
