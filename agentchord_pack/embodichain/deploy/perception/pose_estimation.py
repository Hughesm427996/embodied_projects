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

from pathlib import Path
from types import SimpleNamespace
from typing import Any

import numpy as np
import torch

from embodichain.utils.logger import log_info

from .sam3 import get_sam_mask
from .stereo import left_right_to_depth

__all__ = [
    "convert_rgbd_to_pc",
    "filter_and_calculate",
    "get_obj_pose_from_perception",
    "get_rotated_corners",
    "visualize_mask_with_corners",
]


def _depth_to_points(depth: np.ndarray, intrinsic: np.ndarray) -> np.ndarray:
    height, width = depth.shape
    yy, xx = np.meshgrid(np.arange(height), np.arange(width), indexing="ij")
    z = depth.astype(np.float32)
    x = (xx.astype(np.float32) - intrinsic[0, 2]) * z / intrinsic[0, 0]
    y = (yy.astype(np.float32) - intrinsic[1, 2]) * z / intrinsic[1, 1]
    return np.stack([x, y, z], axis=-1)


def convert_rgbd_to_pc(
    rgb: np.ndarray,
    depth: np.ndarray,
    mask: np.ndarray,
    intrinsic: np.ndarray,
) -> Any:
    """Convert a masked RGB-D observation to a point cloud object."""
    rgb = np.asarray(rgb)
    depth = np.asarray(depth, dtype=np.float32)
    mask = np.asarray(mask, dtype=bool)
    intrinsic = np.asarray(intrinsic, dtype=np.float32)

    resolution = rgb.shape[:2]
    if rgb.dtype != np.uint8:
        raise ValueError(f"rgb must have dtype uint8, got {rgb.dtype}.")
    if depth.shape != resolution:
        raise ValueError("depth shape must match rgb resolution.")
    if mask.shape[:2] != resolution:
        raise ValueError("mask shape must match rgb resolution.")
    if intrinsic.shape != (3, 3):
        raise ValueError("intrinsic must have shape (3, 3).")

    points = _depth_to_points(depth, intrinsic).reshape(-1, 3)
    colors = rgb.reshape(-1, 3).astype(np.float32) / 255.0
    valid = np.logical_and(points[:, 2] > 1e-4, mask.reshape(-1))
    points = points[valid].astype(np.float32)
    colors = colors[valid].astype(np.float32)

    try:
        import open3d as o3d
    except ImportError:
        return SimpleNamespace(points=points, colors=colors)

    cloud = o3d.geometry.PointCloud()
    cloud.points = o3d.utility.Vector3dVector(points)
    cloud.colors = o3d.utility.Vector3dVector(colors)
    return cloud


def filter_and_calculate(
    pickposes_world: np.ndarray,
    threshold: float = 1.5,
) -> tuple[float, float, float, float, float, np.ndarray, np.ndarray]:
    """Filter pose outliers in xy and return robust object-center statistics."""
    if pickposes_world.size == 0:
        raise ValueError("Cannot estimate object pose from an empty point cloud.")

    x_coords = pickposes_world[:, 0, 3]
    y_coords = pickposes_world[:, 1, 3]
    z_coords = pickposes_world[:, 2, 3]

    q1_x, q3_x = np.percentile(x_coords, 25), np.percentile(x_coords, 75)
    q1_y, q3_y = np.percentile(y_coords, 25), np.percentile(y_coords, 75)
    iqr_x = q3_x - q1_x
    iqr_y = q3_y - q1_y

    valid = np.logical_and.reduce(
        (
            x_coords >= q1_x - threshold * iqr_x,
            x_coords <= q3_x + threshold * iqr_x,
            y_coords >= q1_y - threshold * iqr_y,
            y_coords <= q3_y + threshold * iqr_y,
        )
    )
    if not np.any(valid):
        valid = np.ones_like(x_coords, dtype=bool)

    x_filtered = x_coords[valid]
    y_filtered = y_coords[valid]
    z_filtered = z_coords[valid]

    def robust_midpoint(values: np.ndarray, low: float = 5, high: float = 95) -> float:
        q_low = np.percentile(values, low)
        q_high = np.percentile(values, high)
        return float((q_low + q_high) / 2)

    avg_x = robust_midpoint(x_filtered)
    avg_y = robust_midpoint(y_filtered)
    avg_z = robust_midpoint(z_filtered)
    mean_x = float(np.mean(x_filtered))
    mean_y = float(np.mean(y_filtered))
    return avg_x, avg_y, avg_z, mean_x, mean_y, x_filtered, y_filtered


def get_rotated_corners(
    mask: np.ndarray,
) -> tuple[tuple[int, int] | None, ...]:
    """Return top-left, bottom-left, top-right, and bottom-right mask corners."""
    import cv2

    mask_u8 = np.asarray(mask, dtype=np.uint8) * 255
    contours, _ = cv2.findContours(mask_u8, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
    if not contours:
        return None, None, None, None

    contour = max(contours, key=cv2.contourArea)
    rect = cv2.minAreaRect(contour)
    box_points = cv2.boxPoints(rect).astype(int)
    row_col_points = [(int(y), int(x)) for x, y in box_points]
    points_with_sum = [(row, col, row + col) for row, col in row_col_points]
    points_with_sum.sort(key=lambda point: point[2])

    top_left = points_with_sum[0][:2]
    bottom_right = points_with_sum[3][:2]
    remaining = [point[:2] for point in points_with_sum[1:3]]
    bottom_left = max(remaining, key=lambda point: point[0])
    top_right = min(remaining, key=lambda point: point[0])
    return top_left, bottom_left, top_right, bottom_right


def visualize_mask_with_corners(
    mask: np.ndarray,
    top_left: tuple[int, int],
    bottom_left: tuple[int, int],
    top_right: tuple[int, int],
    bottom_right: tuple[int, int],
    save_path: str | Path = "images/mask_with_corners.png",
) -> None:
    """Save a debug visualization of a binary mask and its selected corners."""
    import matplotlib.pyplot as plt

    save_path = Path(save_path)
    save_path.parent.mkdir(parents=True, exist_ok=True)

    plt.figure(figsize=(10, 10))
    plt.imshow(mask, cmap="gray")
    plt.scatter(top_left[1], top_left[0], color="yellow", label="top left", s=100)
    plt.scatter(
        bottom_left[1], bottom_left[0], color="blue", label="bottom left", s=100
    )
    plt.scatter(top_right[1], top_right[0], color="green", label="top right", s=100)
    plt.scatter(
        bottom_right[1],
        bottom_right[0],
        color="red",
        label="bottom right",
        s=100,
    )
    plt.legend()
    plt.axis("off")
    plt.savefig(save_path)
    plt.close()


def _observation_value(
    observations: dict[str, Any],
    *keys: str,
    required: bool = True,
) -> Any:
    for key in keys:
        if key in observations:
            return observations[key]
    if required:
        raise KeyError(f"Observation is missing one of these keys: {keys}.")
    return None


def _save_rgb_image(
    rgb: np.ndarray,
    save_dir: str | Path = "images",
    filename: str = "obs.png",
) -> Path:
    save_dir = Path(save_dir)
    save_dir.mkdir(parents=True, exist_ok=True)
    path = save_dir / filename

    try:
        import cv2
    except ImportError:
        from PIL import Image

        Image.fromarray(rgb).save(path)
        return path

    bgr = cv2.cvtColor(rgb, cv2.COLOR_RGB2BGR)
    cv2.imwrite(str(path), bgr)
    return path


def _extract_mask(sam_results: Any) -> np.ndarray | None:
    if sam_results is None:
        return None
    first_result = (
        sam_results[0] if isinstance(sam_results, (list, tuple)) else sam_results
    )
    masks = getattr(first_result, "masks", None)
    mask_data = getattr(masks, "data", None)
    if mask_data is None or len(mask_data) == 0:
        return None
    mask = mask_data[0]
    if isinstance(mask, torch.Tensor):
        mask = mask.detach().cpu().numpy()
    return np.asarray(mask, dtype=bool)


def _resize_mask(mask: np.ndarray, shape: tuple[int, int]) -> np.ndarray:
    if mask.shape[:2] == shape:
        return mask.astype(bool)

    import cv2

    resized = cv2.resize(
        mask.astype(np.uint8),
        (shape[1], shape[0]),
        interpolation=cv2.INTER_NEAREST,
    )
    return resized.astype(bool)


def _select_corner_mask(mask: np.ndarray, kwargs: dict[str, Any]) -> np.ndarray:
    selector = kwargs.get("mask_selector")
    if callable(selector):
        return np.asarray(selector(mask), dtype=bool)

    corner_name = kwargs.get("mask_corner")
    legacy_corner_flags = {
        "left_above_mask": "top_left",
        "right_above_mask": "top_right",
        "left_bottom_mask": "bottom_left",
        "right_bottom_mask": "bottom_right",
    }
    for key, value in legacy_corner_flags.items():
        if kwargs.get(key):
            corner_name = value
            break

    if corner_name is None and not kwargs.get("visualize_mask", False):
        return mask

    top_left, bottom_left, top_right, bottom_right = get_rotated_corners(mask)
    if None in (top_left, bottom_left, top_right, bottom_right):
        return mask

    if kwargs.get("visualize_mask", False):
        visualize_mask_with_corners(
            mask,
            top_left,
            bottom_left,
            top_right,
            bottom_right,
            save_path=kwargs.get(
                "mask_visualization_path", "images/mask_with_corners.png"
            ),
        )

    corners = {
        "top_left": top_left,
        "bottom_left": bottom_left,
        "top_right": top_right,
        "bottom_right": bottom_right,
    }
    if corner_name is None:
        return mask
    if corner_name not in corners:
        raise ValueError(
            f"mask_corner must be one of {sorted(corners)}, got {corner_name!r}."
        )

    corner_mask = np.zeros_like(mask, dtype=bool)
    row, col = corners[corner_name]
    row = int(np.clip(row, 0, mask.shape[0] - 1))
    col = int(np.clip(col, 0, mask.shape[1] - 1))
    corner_mask[row, col] = True
    return corner_mask


def get_obj_pose_from_perception(
    env,
    obj_name: str,
    robot_name: str,
    kwargs: dict[str, Any] | None = None,
) -> np.ndarray:
    """Estimate an object pose from perception for real-world atom actions."""
    kwargs = kwargs or {}
    if kwargs.get("target_obj_pose") is not None:
        return np.asarray(kwargs["target_obj_pose"], dtype=np.float32)

    observations = env.get_obs_for_agent()
    rgb = np.asarray(_observation_value(observations, "left_rgb", "rgb")).copy()
    cam_k = np.asarray(
        _observation_value(observations, "cam_k"), dtype=np.float32
    ).copy()
    cam_pose_key = "T_to_left_arm" if "left" in robot_name else "T_to_right_arm"
    cam_pose = np.asarray(
        _observation_value(observations, cam_pose_key), dtype=np.float32
    )

    depth = _observation_value(observations, "depth", required=False)
    if depth is None:
        right_rgb = np.asarray(_observation_value(observations, "right_rgb")).copy()
        baseline = float(
            np.asarray(_observation_value(observations, "baseline")).reshape(-1)[0]
        )
        stereo_model = kwargs.get("foundation_stereo_model")
        if stereo_model is None:
            stereo_model = getattr(env, "foundation_stereo_model")
        depth, _ = left_right_to_depth(
            rgb,
            right_rgb,
            stereo_model,
            fx=float(cam_k[0, 0]),
            baseline=baseline,
            max_disp=kwargs.get("max_disp", getattr(env, "max_disp", 416)),
            iters=kwargs.get("stereo_iters", 8),
            threshold=kwargs.get("stereo_threshold", 0.0),
        )
    else:
        depth = np.asarray(depth, dtype=np.float32)

    mask = kwargs.get("mask")
    if mask is None:
        image_path = kwargs.get("image_path")
        if image_path is None:
            image_path = _save_rgb_image(
                rgb,
                save_dir=kwargs.get("image_dir", "images"),
                filename=kwargs.get(
                    "image_filename",
                    f"{obj_name}_{getattr(env, 'current_step', 0)}.png",
                ),
            )
        image_path = Path(image_path)
        target_dir = kwargs.get("sam_save_dir")
        if target_dir is None:
            target_dir = image_path.parent / "sam"
        target_dir = Path(target_dir)
        target_dir.mkdir(parents=True, exist_ok=True)

        predictor = kwargs.get("predictor", getattr(env, "predictor", None))
        if predictor is None:
            raise RuntimeError("A SAM predictor is required for real perception.")

        for _ in range(kwargs.get("sam_retries", 10)):
            log_info(f"Using SAM for segmentation of {obj_name}.")
            with torch.no_grad():
                sam_results = get_sam_mask(
                    predictor,
                    obj_name,
                    image_path,
                    save_path=target_dir,
                )
            mask = _extract_mask(sam_results)
            if mask is not None:
                break

    if mask is None:
        raise RuntimeError(f"SAM failed to segment {obj_name}.")

    mask = _resize_mask(np.asarray(mask, dtype=bool), rgb.shape[:2])
    mask = _select_corner_mask(mask, kwargs)

    cloud = convert_rgbd_to_pc(rgb, depth, mask, cam_k)
    pc_xyz = np.asarray(cloud.points, dtype=np.float32)
    if pc_xyz.size == 0:
        raise RuntimeError(f"No valid depth points inside the {obj_name} mask.")

    pick_poses = np.repeat(np.eye(4, dtype=np.float32)[None, :, :], len(pc_xyz), axis=0)
    pick_poses[:, :3, 3] = pc_xyz
    pickposes_world = cam_pose[None, :, :] @ pick_poses

    avg_x, avg_y, avg_z, mean_x, mean_y, *_ = filter_and_calculate(
        pickposes_world,
        threshold=kwargs.get("pose_filter_threshold", 1.1),
    )

    if not hasattr(env, f"{obj_name}_height"):
        setattr(env, f"{obj_name}_height", avg_z)

    target_obj_pose = np.eye(4, dtype=np.float32)
    target_obj_pose[0, 3] = (avg_x + mean_x) / 2
    target_obj_pose[1, 3] = (avg_y + mean_y) / 2
    target_obj_pose[2, 3] = avg_z
    return target_obj_pose
