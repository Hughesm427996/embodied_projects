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

import ast
from functools import partial
from pathlib import Path
from typing import Any

import numpy as np
import torch

RIGHT_ARM_SLICE = slice(4, 8)
LEFT_ARM_SLICE = slice(0, 4)
REPO_ROOT = Path(__file__).resolve().parents[3]
ATOM_ACTIONS_PATH = (
    REPO_ROOT / "embodichain" / "agents" / "agentchord" / "atom_actions.py"
)
ATOM_ACTION_UTILS_PATH = (
    REPO_ROOT / "embodichain" / "agents" / "agentchord" / "atom_action_utils.py"
)


def _finalize_actions(select_qpos_traj, ee_state_list_select):
    return np.concatenate(
        [
            np.array(select_qpos_traj),
            np.array(ee_state_list_select),
            np.array(ee_state_list_select),
        ],
        axis=-1,
    )


def _load_drive_function():
    namespace = {
        "Any": Any,
        "np": np,
        "partial": partial,
        "torch": torch,
        "tqdm": lambda iterable: iterable,
        "finalize_actions": _finalize_actions,
        "resolve_action": lambda action, env, kwargs: (
            action(env=env, **kwargs) if callable(action) else action
        ),
        "log_info": lambda *args, **kwargs: None,
        "log_warning": lambda *args, **kwargs: None,
        "log_error": lambda *args, **kwargs: None,
        "setup_interactive_error_input": lambda enabled=None: None,
        "restore_interactive_error_input": lambda interactive_input: None,
        "interactive_error_requested": lambda interactive_input: False,
        "inject_interactive_error": lambda env: None,
    }

    def _stub_sync_agent_state_from_robot(env) -> None:
        action = env.robot.get_qpos().squeeze(0)
        env.left_arm_current_qpos = action[env.left_arm_joints].numpy()
        env.left_arm_current_xpos = env.robot.compute_fk(
            qpos=env.left_arm_current_qpos,
            name="left_arm",
            to_matrix=True,
        ).squeeze(0)
        env.left_arm_current_gripper_state = action[env.left_eef_joints][0].numpy()[
            None
        ]
        env.right_arm_current_qpos = action[env.right_arm_joints].numpy()
        env.right_arm_current_xpos = env.robot.compute_fk(
            qpos=env.right_arm_current_qpos,
            name="right_arm",
            to_matrix=True,
        ).squeeze(0)
        env.right_arm_current_gripper_state = action[env.right_eef_joints][0].numpy()[
            None
        ]

    namespace["sync_agent_state_from_robot"] = _stub_sync_agent_state_from_robot
    _load_atom_functions(
        [
            "convert_action_from_real_to_sim",
            "convert_action_from_sim_to_real",
            "drive",
            "get_function_name",
            "is_real_backend",
            "set_qpos",
            "sim_action_dim",
            "sync_agent_state_from_sim_action",
        ],
        namespace,
    )
    return namespace


def _load_atom_functions(function_names: list[str], namespace: dict[str, Any]) -> None:
    _load_functions_from_path(ATOM_ACTION_UTILS_PATH, function_names, namespace)
    _load_functions_from_path(ATOM_ACTIONS_PATH, function_names, namespace)


def _load_functions_from_path(
    source_path: Path,
    function_names: list[str],
    namespace: dict[str, Any],
) -> None:
    source = source_path.read_text()
    module = ast.parse(source, filename=str(source_path))
    selected_nodes = []
    for node in module.body:
        if isinstance(node, ast.Assign):
            target_names = [
                target.id for target in node.targets if isinstance(target, ast.Name)
            ]
            if any(name.startswith(("SIM_", "REAL_")) for name in target_names):
                selected_nodes.append(node)
        elif isinstance(node, ast.FunctionDef) and node.name in function_names:
            selected_nodes.append(node)

    atom_module = ast.Module(body=selected_nodes, type_ignores=[])
    exec(compile(atom_module, filename=str(source_path), mode="exec"), namespace)


_drive_namespace = _load_drive_function()
drive = _drive_namespace["drive"]
convert_action_from_sim_to_real = _drive_namespace["convert_action_from_sim_to_real"]
convert_action_from_real_to_sim = _drive_namespace["convert_action_from_real_to_sim"]


def _load_open_gripper_function():
    plan_calls = {"count": 0}

    def _stub_get_arm_states(env, robot_name):
        if "left" in robot_name:
            return (
                True,
                "left_arm",
                env.left_arm_current_qpos,
                torch.eye(4, dtype=torch.float32),
                env.left_arm_current_gripper_state,
            )
        return (
            False,
            "right_arm",
            env.right_arm_current_qpos,
            torch.eye(4, dtype=torch.float32),
            env.right_arm_current_gripper_state,
        )

    def _stub_plan_gripper_trajectory(*args, **kwargs):
        plan_calls["count"] += 1

    namespace = {
        "Any": Any,
        "np": np,
        "torch": torch,
        "get_arm_states": _stub_get_arm_states,
        "plan_gripper_trajectory": _stub_plan_gripper_trajectory,
        "finalize_actions": _finalize_actions,
        "log_info": lambda *args, **kwargs: None,
    }
    _load_atom_functions(["open_gripper"], namespace)
    return namespace["open_gripper"], plan_calls


open_gripper, open_gripper_plan_calls = _load_open_gripper_function()


class _DummyRobot:
    def __init__(self) -> None:
        self.qpos = torch.zeros(8, dtype=torch.float32)

    def get_qpos(self) -> torch.Tensor:
        return self.qpos.unsqueeze(0)

    def compute_fk(self, qpos, name: str, to_matrix: bool = True) -> torch.Tensor:
        pose = torch.eye(4, dtype=torch.float32).unsqueeze(0)
        pose[0, 0, 3] = float(torch.as_tensor(qpos, dtype=torch.float32).sum())
        return pose


class _DummyEnv:
    def __init__(self) -> None:
        self.left_arm_joints = [0, 1]
        self.left_eef_joints = [2, 3]
        self.right_arm_joints = [4, 5]
        self.right_eef_joints = [6, 7]
        self.init_qpos = np.zeros(8, dtype=np.float32)
        self.left_arm_current_qpos = np.array([0.0, 0.0], dtype=np.float32)
        self.right_arm_current_qpos = np.array([0.0, 0.0], dtype=np.float32)
        self.left_arm_current_gripper_state = np.array([0.05], dtype=np.float32)
        self.right_arm_current_gripper_state = np.array([0.05], dtype=np.float32)
        self.robot = _DummyRobot()
        self.step_calls: list[np.ndarray] = []
        self.update_calls = 0

    def step(self, action: torch.Tensor) -> None:
        qpos = action.squeeze(0).detach().cpu()
        self.robot.qpos = qpos
        self.step_calls.append(qpos.numpy())

    def update_obj_info(self) -> None:
        self.update_calls += 1


class _ObjectMovementEnv(_DummyEnv):
    def __init__(self) -> None:
        super().__init__()
        self.current_object_pose = torch.eye(4, dtype=torch.float32)
        self.obj_info = {"cup": {"pose": self.current_object_pose.clone()}}

    def step(self, action: torch.Tensor) -> None:
        super().step(action)
        self.current_object_pose = torch.eye(4, dtype=torch.float32)
        self.current_object_pose[0, 3] = 0.03

    def update_obj_info(self) -> None:
        super().update_obj_info()
        self.obj_info["cup"]["pose"] = self.current_object_pose.clone()


class _RealControllerEnv:
    def __init__(self) -> None:
        self.backend = "real"
        self.left_arm_joints = [0, 2, 4, 6, 8, 10]
        self.right_arm_joints = [1, 3, 5, 7, 9, 11]
        self.left_eef_joints = [12, 13]
        self.right_eef_joints = [14, 15]
        self.init_qpos = np.zeros(16, dtype=np.float32)
        self.left_arm_current_qpos = np.zeros(6, dtype=np.float32)
        self.right_arm_current_qpos = np.zeros(6, dtype=np.float32)
        self.left_arm_current_gripper_state = np.array([1.0], dtype=np.float32)
        self.right_arm_current_gripper_state = np.array([1.0], dtype=np.float32)
        self.left_gripper_joint_limits = (0.2, 0.4)
        self.right_gripper_joint_limits = (1.0, 3.0)
        self.current_step = 0
        self.commands: list[tuple[np.ndarray, int, bool]] = []

    def set_current_qpos(
        self,
        qpos: np.ndarray,
        interp_num: int = 0,
        wait: bool = True,
    ) -> None:
        self.commands.append((qpos.copy(), interp_num, wait))

    def get_arm_fk(self, qpos, is_left: bool):
        pose = np.eye(4, dtype=np.float32)
        pose[0, 3] = float(np.asarray(qpos).sum())
        pose[1, 3] = 1.0 if is_left else -1.0
        return pose


def test_open_gripper_skips_when_skip_condition_is_met() -> None:
    env = _DummyEnv()
    env.open_state = torch.tensor([0.05], dtype=torch.float32)
    env.left_arm_current_qpos = np.array([0.0, 0.0], dtype=np.float32)
    env.left_arm_current_gripper_state = np.array([0.05], dtype=np.float32)
    before_calls = open_gripper_plan_calls["count"]

    actions = open_gripper(robot_name="left_arm", env=env)

    assert actions.shape == (1, 4)
    np.testing.assert_allclose(
        actions[0], np.array([0.0, 0.0, 0.05, 0.05], dtype=np.float32)
    )
    assert open_gripper_plan_calls["count"] == before_calls


def test_drive_stops_failed_trajectory_when_monitor_triggers() -> None:
    env = _DummyEnv()
    main_action = np.array(
        [
            [1.0, 1.1, 1.2, 1.3],
            [2.0, 2.1, 2.2, 2.3],
            [3.0, 3.1, 3.2, 3.3],
        ],
        dtype=np.float32,
    )
    next_action = np.array([[5.0, 5.1, 5.2, 5.3]], dtype=np.float32)

    monitor_calls = {"count": 0}

    def trigger_once() -> bool:
        monitor_calls["count"] += 1
        return monitor_calls["count"] == 1

    monitor = partial(trigger_once)

    drive(
        left_arm_action=None,
        right_arm_action=main_action,
        monitor_sequences=[[monitor]],
        env=env,
    )

    assert len(env.step_calls) == 1
    np.testing.assert_allclose(env.step_calls[0][RIGHT_ARM_SLICE], main_action[0])

    drive(
        left_arm_action=None,
        right_arm_action=next_action,
        env=env,
    )

    assert len(env.step_calls) == 2
    np.testing.assert_allclose(env.step_calls[1][RIGHT_ARM_SLICE], next_action[0])


def test_drive_return_result_syncs_agent_state_at_monitor_trigger() -> None:
    env = _DummyEnv()
    main_action = np.array(
        [
            [1.0, 1.1, 1.2, 1.3],
            [2.0, 2.1, 2.2, 2.3],
        ],
        dtype=np.float32,
    )

    result = drive(
        left_arm_action=None,
        right_arm_action=main_action,
        monitor_sequences=[[partial(lambda: True)]],
        env=env,
        return_result=True,
    )

    assert len(result["actions"]) == 1
    np.testing.assert_allclose(
        env.right_arm_current_qpos,
        np.array([1.0, 1.1], dtype=np.float32),
    )
    np.testing.assert_allclose(
        env.right_arm_current_gripper_state,
        np.array([1.2], dtype=np.float32),
    )
    assert env.right_arm_current_xpos[0, 3] == np.float32(2.1)


def test_drive_checks_monitors_before_overwriting_previous_object_info() -> None:
    env = _ObjectMovementEnv()

    def object_moved(env) -> bool:
        previous_x = env.obj_info["cup"]["pose"][0, 3]
        current_x = env.current_object_pose[0, 3]
        return bool(abs(current_x - previous_x) > 0.02)

    result = drive(
        left_arm_action=None,
        right_arm_action=np.array([[1.0, 1.1, 1.2, 1.3]], dtype=np.float32),
        monitor_sequences=[[partial(object_moved, env=env)]],
        env=env,
        return_result=True,
    )

    assert result["monitor_index"] == 0
    assert env.update_calls == 1
    assert env.obj_info["cup"]["pose"][0, 3] == torch.tensor(0.03)


def test_convert_action_between_sim_and_real_layouts() -> None:
    sim_action = np.arange(16, dtype=np.float32)

    real_action = convert_action_from_sim_to_real(sim_action)
    round_trip = convert_action_from_real_to_sim(real_action)

    np.testing.assert_allclose(
        real_action,
        np.array([0, 2, 4, 6, 8, 10, 12, 1, 3, 5, 7, 9, 11, 14], dtype=np.float32),
    )
    np.testing.assert_allclose(round_trip[:13], sim_action[:13])
    assert round_trip[13] == sim_action[12]
    np.testing.assert_allclose(round_trip[14:], np.array([14, 14], dtype=np.float32))


def test_real_drive_maps_and_sends_controller_qpos() -> None:
    env = _RealControllerEnv()
    right_arm_action = np.array(
        [[1.0, 2.0, 3.0, 4.0, 5.0, 6.0, 0.25, 0.25]],
        dtype=np.float32,
    )

    result = drive(
        left_arm_action=None,
        right_arm_action=right_arm_action,
        env=env,
        return_result=True,
        wait=False,
        interp_num=3,
    )

    assert len(env.commands) == 1
    command, interp_num, wait = env.commands[0]
    assert interp_num == 3
    assert wait is False
    np.testing.assert_allclose(
        result["actions"][0],
        np.array(
            [0, 0, 0, 0, 0, 0, 1.0, 1, 2, 3, 4, 5, 6, 0.25],
            dtype=np.float32,
        ),
    )
    np.testing.assert_allclose(command[:6], np.zeros(6, dtype=np.float32))
    assert command[6] == np.float32(0.4)
    np.testing.assert_allclose(command[7:13], np.arange(1, 7, dtype=np.float32))
    assert command[13] == np.float32(1.5)
    np.testing.assert_allclose(env.right_arm_current_qpos, np.arange(1, 7))
    np.testing.assert_allclose(env.right_arm_current_gripper_state, np.array([0.25]))
    assert env.current_step == 1
