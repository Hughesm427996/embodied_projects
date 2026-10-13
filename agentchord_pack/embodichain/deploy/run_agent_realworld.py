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

import argparse
import sys
import time
from pathlib import Path
from typing import Any

import numpy as np
import torch

REPO_ROOT = Path(__file__).resolve().parents[2]
if str(REPO_ROOT) not in sys.path:
    sys.path.insert(0, str(REPO_ROOT))

from embodichain.deploy.robots.cobotmagic import ros_bridge  # noqa: E402
from embodichain.deploy.robots.cobotmagic.controller import (  # noqa: E402
    AgentAgilexController,
)
from embodichain.utils.utility import load_json  # noqa: E402

__all__ = ["main"]


DEFAULT_AGENT_CONFIG = (
    REPO_ROOT / "configs/gym/agent/pour_water_agent/agent_config.json"
)
DEFAULT_AFFORDANCE_CONFIG = (
    REPO_ROOT / "configs/gym/agent/pour_water_agent/fast_gym_config.json"
)
DEFAULT_TASK_NAME = "SingleArmPourWater"
DEFAULT_OBJECT_NAMES = "bottle,cup"

# Real robot layout: left arm 6 + left gripper + right arm 6 + right gripper.
DEFAULT_QPOS = np.array(
    [
        0.0,
        0.0,
        0.0,
        0.0,
        0.0,
        0.0,
        1.0,
        0.0,
        0.0,
        0.0,
        0.0,
        0.0,
        0.0,
        1.0,
    ],
    dtype=np.float32,
)


def _resolve_path(path: str | Path | None) -> Path | None:
    if path is None or str(path) == "":
        return None
    resolved = Path(path).expanduser()
    if not resolved.is_absolute():
        resolved = REPO_ROOT / resolved
    return resolved


def _parse_csv(value: str | None) -> list[str]:
    if value is None:
        return []
    return [item.strip() for item in value.split(",") if item.strip()]


def _parse_default_qpos(value: str | None) -> np.ndarray:
    if value is None or value == "":
        return DEFAULT_QPOS.copy()

    path = _resolve_path(value)
    if path is not None and path.exists():
        if path.suffix == ".npy":
            qpos = np.load(path)
        else:
            qpos = np.loadtxt(path, delimiter=",")
        return np.asarray(qpos, dtype=np.float32).reshape(-1)

    return np.asarray([float(x) for x in value.split(",")], dtype=np.float32)


def _get_ros_operator_class(ros_version: str):
    if ros_version == "ros1":
        if getattr(ros_bridge, "rospy", None) is None:
            raise RuntimeError("rospy is not available for ROS 1 mode.")
        return ros_bridge.RosOperator
    if ros_version == "ros2":
        ros2_operator = getattr(ros_bridge, "RosOperatorForRos2", None)
        if ros2_operator is None or getattr(ros_bridge, "rclpy", None) is None:
            raise RuntimeError(
                "rclpy/RosOperatorForRos2 is not available for ROS 2 mode."
            )
        return ros2_operator

    if getattr(ros_bridge, "rospy", None) is not None:
        return ros_bridge.RosOperator

    ros2_operator = getattr(ros_bridge, "RosOperatorForRos2", None)
    if ros2_operator is not None and getattr(ros_bridge, "rclpy", None) is not None:
        return ros2_operator
    raise RuntimeError("No available ROS bridge backend found.")


def _build_ros_operator(ros_version: str):
    operator_cls = _get_ros_operator_class(ros_version)
    return operator_cls()


def _iter_static_affordance_specs(node: Any):
    if isinstance(node, dict):
        if node.get("mode") == "static" and "name" in node and "value" in node:
            yield node
        for value in node.values():
            yield from _iter_static_affordance_specs(value)
    elif isinstance(node, list):
        for value in node:
            yield from _iter_static_affordance_specs(value)


def _entity_uids_from_spec(spec: dict[str, Any]) -> list[str]:
    entity_cfg = spec.get("entity_cfg")
    if isinstance(entity_cfg, dict) and entity_cfg.get("uid"):
        return [entity_cfg["uid"]]

    entity_uids = spec.get("entity_uids")
    if isinstance(entity_uids, str):
        return [] if entity_uids == "all_objects" else [entity_uids]
    if isinstance(entity_uids, list):
        return [uid for uid in entity_uids if isinstance(uid, str)]

    return []


def _as_pose_tensor(value: Any) -> torch.Tensor:
    pose = torch.as_tensor(value, dtype=torch.float32)
    if pose.ndim == 3 and pose.shape[0] == 1:
        pose = pose.squeeze(0)
    return pose


def _load_affordance_datas(path: str | Path | None) -> dict[str, torch.Tensor]:
    resolved = _resolve_path(path)
    if resolved is None:
        return {}

    config = load_json(str(resolved))
    affordance_datas: dict[str, torch.Tensor] = {}

    for spec in _iter_static_affordance_specs(config):
        attr_name = spec["name"]
        if not attr_name.endswith("_pose_object"):
            continue

        for uid in _entity_uids_from_spec(spec):
            affordance_datas[f"{uid}_{attr_name}"] = _as_pose_tensor(spec["value"])

    if affordance_datas:
        return affordance_datas

    if isinstance(config, dict):
        for key, value in config.items():
            if key.endswith("_pose_object"):
                affordance_datas[key] = _as_pose_tensor(value)

    return affordance_datas


def _move_to_default(
    controller: AgentAgilexController,
    default_qpos: np.ndarray,
    *,
    wait: bool,
    interp_num: int,
) -> None:
    if default_qpos.shape[0] != 14:
        raise ValueError(
            f"default_qpos must contain 14 values for the real robot, got "
            f"{default_qpos.shape[0]}."
        )

    left_range = controller.left_gripper_joint_limits
    right_range = controller.right_gripper_joint_limits
    qpos = default_qpos.copy()
    qpos[6] = left_range[0] + (left_range[1] - left_range[0]) * qpos[6]
    qpos[13] = right_range[0] + (right_range[1] - right_range[0]) * qpos[13]
    controller.set_current_qpos(qpos, wait=wait, interp_num=interp_num)


def _parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(
        description="Run an AgentChord agent on the real CobotMagic robot."
    )
    parser.add_argument("--task_name", default=DEFAULT_TASK_NAME)
    parser.add_argument("--agent_config", default=str(DEFAULT_AGENT_CONFIG))
    parser.add_argument("--affordance_config", default=str(DEFAULT_AFFORDANCE_CONFIG))
    parser.add_argument("--object_names", default=DEFAULT_OBJECT_NAMES)
    parser.add_argument("--camera_config", default=None)
    parser.add_argument("--kingfisher_ip", default="192.168.1.188")
    parser.add_argument(
        "--ros_version",
        choices=("auto", "ros1", "ros2"),
        default="auto",
        help="ROS bridge implementation to use.",
    )
    parser.add_argument(
        "--default_qpos",
        default="",
        help="14 comma-separated values, or a .npy/.csv path. Empty uses built-in zero pose.",
    )
    parser.add_argument("--skip_default_pose", action="store_true")
    parser.add_argument("--skip_initial_perception", action="store_true")
    parser.add_argument("--ros_warmup_sec", type=float, default=3.0)
    parser.add_argument("--post_default_sleep", type=float, default=8.0)
    parser.add_argument("--interp_num", type=int, default=0)
    parser.add_argument(
        "--wait",
        action=argparse.BooleanOptionalAction,
        default=True,
        help="Wait for robot motion commands to finish.",
    )
    parser.add_argument("--regenerate", action="store_true")
    parser.add_argument("--recovery", action="store_true")
    parser.add_argument("--interactive_error_injection", action="store_true")
    return parser.parse_args()


def main() -> None:
    args = _parse_args()

    agent_config_path = _resolve_path(args.agent_config)
    if agent_config_path is None:
        raise ValueError("agent_config cannot be empty.")
    agent_config = load_json(str(agent_config_path))
    affordance_datas = _load_affordance_datas(args.affordance_config)
    object_names = _parse_csv(args.object_names)

    ros_operator = _build_ros_operator(args.ros_version)
    time.sleep(args.ros_warmup_sec)

    controller = AgentAgilexController(
        ros_operator=ros_operator,
        init_controller=True,
        agent_config=agent_config,
        task_name=args.task_name,
        agent_config_path=str(agent_config_path),
        object_names=[],
        affordance_datas=affordance_datas,
        camera_config_path=args.camera_config,
        kingfisher_ip=args.kingfisher_ip,
    )

    if not args.skip_default_pose:
        default_qpos = _parse_default_qpos(args.default_qpos)
        _move_to_default(
            controller,
            default_qpos,
            wait=args.wait,
            interp_num=args.interp_num,
        )
        from embodichain.agents.agentchord.atom_action_utils import (  # noqa: PLC0415
            convert_action_from_real_to_sim,
            sync_agent_state_from_sim_action,
        )

        sync_agent_state_from_sim_action(
            controller,
            convert_action_from_real_to_sim(default_qpos),
        )
        time.sleep(args.post_default_sleep)

    controller.object_names = object_names
    if object_names and not args.skip_initial_perception:
        controller.update_obj_info(obj_names=object_names)

    executed_actions = controller.create_demo_action_list(
        regenerate=args.regenerate,
        recovery=args.recovery,
        interactive_error_injection=args.interactive_error_injection,
        wait=args.wait,
        interp_num=args.interp_num,
        real_world=True,
        action_backend="real",
    )

    print(f"[INFO] Finished real-world agent run with {len(executed_actions)} actions.")


if __name__ == "__main__":
    main()
