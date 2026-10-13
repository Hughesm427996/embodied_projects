"""Read-only SDK trajectory compilation for the supported static-plate plan.

This checks kinematic targets and trajectory generation. It does not simulate
contacts, collisions, grasp retention, or the physical result of placement.
"""

from __future__ import annotations

import copy
import math
from types import SimpleNamespace


ALLOWLIST = frozenset({
    "grasp", "orient_eef", "move_relative_to_object", "open_gripper",
    "back_to_initial_pose",
})
FORBIDDEN_KEYS = frozenset({
    "env", "target_obj_pose", "force_valid", "perception", "use_perception",
    "use_perception_pose", "real_world", "robot_name", "obj_name",
    "pre_grasp_dis", "direction", "x_offset", "y_offset", "z_offset",
    "action", "left_arm_action", "right_arm_action", "action_overrides",
})
UNSAFE_ACTION_KEYS = frozenset({
    "env", "target_obj_pose", "perception", "use_perception",
    "use_perception_pose", "real_world", "action", "action_overrides",
    "action_backend",
})


class IKFailure(RuntimeError):
    """The native IK call returned false, rather than raising an SDK error."""


class CheckNotApplicable(ValueError):
    """A plan cannot safely use this restricted read-only predictor."""


def clone(value):
    if hasattr(value, "detach") and hasattr(value, "clone"):
        return value.detach().clone()
    if isinstance(value, dict):
        return {k: clone(v) for k, v in value.items()}
    if isinstance(value, list):
        return [clone(v) for v in value]
    if isinstance(value, tuple):
        return tuple(clone(v) for v in value)
    return copy.deepcopy(value)


def serializable(value):
    if hasattr(value, "detach"):
        value = value.detach().cpu()
    if hasattr(value, "tolist"):
        return value.tolist()
    if isinstance(value, dict):
        return {str(k): serializable(v) for k, v in value.items()}
    if isinstance(value, (tuple, list)):
        return [serializable(v) for v in value]
    if value is None or isinstance(value, (str, int, float, bool)):
        return value
    return str(value)


class _RigidObjectReadOnly:
    def __init__(self, obj):
        self._obj = obj

    def get_local_pose(self, *args, **kwargs):
        return clone(self._obj.get_local_pose(*args, **kwargs))

    def __getattr__(self, name):
        raise AttributeError(f"Preflight rigid-object operation blocked: {name}")


class _SimulationReadOnly:
    def __init__(self, sim):
        self._sim = sim

    def get_rigid_object_uid_list(self):
        return clone(self._sim.get_rigid_object_uid_list())

    def get_rigid_object(self, *args, **kwargs):
        return _RigidObjectReadOnly(self._sim.get_rigid_object(*args, **kwargs))

    def __getattr__(self, name):
        raise AttributeError(f"Preflight simulation operation blocked: {name}")


class ShadowEnv:
    """Clone agent state and expose only reviewed read-only SDK operations."""

    def __init__(self, env):
        # A recovery clone may be built from predicted nominal state. Route
        # read-only IK/FK straight to the real backend, not a stale shadow log.
        self._env = env._env if isinstance(env, ShadowEnv) else env
        self.backend = "sim"
        self.is_real = False
        self.real_world = False
        self.robot = SimpleNamespace(uid=clone(env.robot.uid))
        self.sim = _SimulationReadOnly(env.sim)
        self.ik_calls = []
        self.active_edge = None
        self.active_arm = None
        self.active_action = None
        # Deliberately do not delegate other attributes or bound getters.
        attrs = ["open_state", "close_state", "obj_info"]
        for side in ("left", "right"):
            attrs += [f"{side}_arm_{suffix}" for suffix in (
                "current_qpos", "current_xpos", "current_gripper_state",
                "init_qpos", "init_xpos", "base_pose",
            )]
        for name in attrs:
            setattr(self, name, clone(getattr(env, name)))

    def get_current_qpos_agent(self):
        return self.left_arm_current_qpos, self.right_arm_current_qpos

    def get_current_xpos_agent(self):
        return self.left_arm_current_xpos, self.right_arm_current_xpos

    def get_current_gripper_state_agent(self):
        return (self.left_arm_current_gripper_state,
                self.right_arm_current_gripper_state)

    def set_current_qpos_agent(self, arm_qpos, is_left):
        name = "left" if is_left else "right"
        setattr(self, f"{name}_arm_current_qpos", clone(arm_qpos))

    def set_current_xpos_agent(self, arm_xpos, is_left):
        name = "left" if is_left else "right"
        setattr(self, f"{name}_arm_current_xpos", clone(arm_xpos))

    def set_current_gripper_state_agent(self, arm_gripper_state, is_left):
        name = "left" if is_left else "right"
        setattr(self, f"{name}_arm_current_gripper_state", clone(arm_gripper_state))

    def get_arm_ik(self, target_xpos, is_left, qpos_seed=None):
        entry = {
            "edge": self.active_edge,
            "arm": "left_arm" if is_left else "right_arm",
            "action": self.active_action,
            "target_xpos": serializable(target_xpos),
            "qpos_seed": serializable(qpos_seed),
        }
        self.ik_calls.append(entry)
        try:
            ret, qpos = self._env.get_arm_ik(
                target_xpos=clone(target_xpos), is_left=is_left,
                qpos_seed=clone(qpos_seed),
            )
            entry["success"] = bool(ret)
            entry["qpos"] = serializable(qpos)
        except Exception as exc:
            entry["error"] = f"{type(exc).__name__}: {exc}"
            raise
        if not entry["success"]:
            raise IKFailure(
                f"IK returned false at {self.active_edge}/{entry['arm']}/"
                f"{self.active_action} (IK call {len(self.ik_calls)})."
            )
        return ret, clone(qpos)

    def get_arm_fk(self, qpos, is_left):
        return clone(self._env.get_arm_fk(qpos=clone(qpos), is_left=is_left))

    def __getattr__(self, name):
        raise AttributeError(f"Preflight environment operation blocked: {name}")


def _task_spec(bundle):
    if not isinstance(bundle, dict):
        raise CheckNotApplicable("Graph bundle must be an object.")
    if "task_graph" in bundle:
        task = bundle["task_graph"]
    elif "task_spec" in bundle:
        task = bundle["task_spec"]
    else:
        task = bundle
    if not isinstance(task, dict):
        raise CheckNotApplicable("Task graph must be an object.")
    def contains_recovery(container):
        if not isinstance(container, dict):
            return bool(container)
        for key in ("recovery_bindings", "recovery_nodes", "recovery_edges", "recovery_branches", "bindings"):
            if container.get(key):
                return True
        if container.get("recovery_enabled"):
            return True
        for key in ("recovery_spec", "recovery_graph"):
            nested = container.get(key)
            if isinstance(nested, dict):
                if nested.get("nodes") or nested.get("edges") or contains_recovery(nested):
                    return True
            elif nested:
                return True
        return False
    if contains_recovery(bundle) or contains_recovery(task) or contains_recovery(bundle.get("metadata", {})):
        raise CheckNotApplicable("Recovery graphs are outside this predictor.")
    return task


def _path(task):
    nodes, edges = task.get("nodes"), task.get("edges")
    if not isinstance(nodes, list) or not isinstance(edges, list):
        raise CheckNotApplicable("nodes and edges must be lists.")
    node_ids = []
    for node in nodes:
        if not isinstance(node, dict) or not isinstance(node.get("id"), str):
            raise CheckNotApplicable("Invalid node entry.")
        node_ids.append(node["id"])
    if len(set(node_ids)) != len(node_ids):
        raise CheckNotApplicable("Duplicate node IDs.")
    start, goal = task.get("start"), task.get("goal")
    if start not in node_ids or goal not in node_ids or start == goal:
        raise CheckNotApplicable("start and goal must be distinct defined nodes.")
    outgoing, edge_ids = {}, set()
    for edge in edges:
        if not isinstance(edge, dict) or not isinstance(edge.get("id"), str):
            raise CheckNotApplicable("Invalid edge entry.")
        if edge["id"] in edge_ids:
            raise CheckNotApplicable("Duplicate edge IDs.")
        edge_ids.add(edge["id"])
        if edge.get("source") not in node_ids or edge.get("target") not in node_ids:
            raise CheckNotApplicable("Undefined source or target.")
        if edge["source"] in outgoing:
            raise CheckNotApplicable("A branch is outside the supported unique chain.")
        outgoing[edge["source"]] = edge
    path, visited, current = [], set(), start
    while current != goal:
        if current in visited or current not in outgoing:
            raise CheckNotApplicable("Cycle or broken start-to-goal path.")
        visited.add(current)
        edge = outgoing[current]
        path.append(edge)
        current = edge["target"]
    if goal in outgoing or len(path) != len(edges):
        raise CheckNotApplicable("Edges exist outside the unique start-to-goal path.")
    return path


def _validated_runtime(runtime):
    runtime = {} if runtime is None else runtime
    if not isinstance(runtime, dict):
        raise CheckNotApplicable("Runtime kwargs must be an object.")
    inactive_flags = {"force_valid", "perception", "use_perception", "use_perception_pose", "real_world"}
    for key in FORBIDDEN_KEYS:
        if key not in runtime:
            continue
        if key in inactive_flags and (runtime[key] is None or runtime[key] is False):
            continue
        if key == "target_obj_pose" and runtime[key] is None:
            continue
        raise CheckNotApplicable("Runtime action, pose, or perception overrides are unsupported.")
    if runtime.get("action_backend") not in (None, "sim", "simulation"):
        raise CheckNotApplicable("Only the simulation action backend is supported.")
    if runtime.get("backend") not in (None, "sim", "simulation"):
        raise CheckNotApplicable("Only the simulation backend is supported.")
    if runtime.get("recovery") or runtime.get("recovery_enabled"):
        raise CheckNotApplicable("Recovery execution is unsupported.")
    if "sample_num" in runtime:
        value = runtime["sample_num"]
        if isinstance(value, bool) or not isinstance(value, int) or value < 2:
            raise CheckNotApplicable("Runtime sample_num must be an integer of at least two.")
    # These native drive-only options are not forwarded to atom functions.
    return {k: clone(v) for k, v in runtime.items() if k not in {
        "action_backend", "recovery", "recovery_enabled",
        "interactive_error_injection", "log_dir",
    } and k not in FORBIDDEN_KEYS}


def _validate_action(action, expected_arm, runtime, edge_index, grabbed):
    if action is None:
        return None
    if not isinstance(action, dict) or action.get("fn") not in ALLOWLIST:
        raise CheckNotApplicable("An action is outside the reviewed SDK allowlist.")
    fn = action["fn"]
    kwargs = action.get("kwargs", {})
    if not isinstance(kwargs, dict):
        raise CheckNotApplicable("Action kwargs must be an object.")
    if any(key in kwargs for key in UNSAFE_ACTION_KEYS) or kwargs.get("force_valid"):
        raise CheckNotApplicable("Pose overrides, perception, or force_valid are unsupported.")
    if kwargs.get("backend") not in (None, "sim", "simulation"):
        raise CheckNotApplicable("Only simulation action kwargs are supported.")
    if kwargs.get("robot_name") != expected_arm:
        raise CheckNotApplicable("An action must name its corresponding arm explicitly.")
    if fn == "grasp":
        if edge_index != 0:
            raise CheckNotApplicable("Only initial first-edge grasps can be predicted.")
        obj = kwargs.get("obj_name")
        if obj not in ("fork", "spoon") or obj in grabbed:
            raise CheckNotApplicable("Initial grasps must target distinct fork/spoon objects.")
        grabbed.add(obj)
        distance = kwargs.get("pre_grasp_dis", 0.05)
        if isinstance(distance, bool) or not isinstance(distance, (int, float)) or not math.isfinite(distance) or distance <= 0:
            raise CheckNotApplicable("pre_grasp_dis must be a positive finite number.")
    elif fn == "move_relative_to_object":
        if kwargs.get("obj_name") != "plate":
            raise CheckNotApplicable("Later moves may reference only the stationary plate.")
        for key in ("x_offset", "y_offset", "z_offset"):
            value = kwargs.get(key, 0)
            if isinstance(value, bool) or not isinstance(value, (int, float)) or not math.isfinite(value):
                raise CheckNotApplicable("Plate offsets must be finite numbers.")
    elif fn == "orient_eef" and kwargs.get("direction", "front") not in ("front", "down"):
        raise CheckNotApplicable("Invalid end-effector direction.")
    if "sample_num" in kwargs:
        value = kwargs["sample_num"]
        if isinstance(value, bool) or not isinstance(value, int) or value < 2:
            raise CheckNotApplicable("sample_num must be an integer of at least two.")
    merged = clone(kwargs)
    merged.update(runtime)  # Match native resolve_action: runtime overrides action kwargs.
    return fn, merged


def evaluate_plan(bundle, env, runtime=None, actions_module=None):
    """Return passed/failed/skipped/error without advancing the real environment.

    ``failed`` means a native IK result was false. ``error`` retains a native
    trajectory/backend exception. ``skipped`` means the predictor is unsafe or
    unsupported for this plan; it is not a successful feasibility check.
    """
    report = {
        "status": "skipped", "scope": "kinematic_targets_and_sdk_trajectories",
        "note": "No physics, collision, grasp retention, or placement-success check.",
        "edges": [], "ik_calls": [],
    }
    shadow = None
    try:
        task = _task_spec(bundle)
        path = _path(task)
        parameters = _validated_runtime(runtime)
        if (getattr(env, "backend", None) == "real" or
                getattr(env, "is_real", False) or getattr(env, "real_world", False)):
            raise CheckNotApplicable("A real-world environment cannot use this simulation predictor.")
        grabbed, calls = set(), []
        for index, edge in enumerate(path):
            edge_calls = []
            for arm in ("left_arm", "right_arm"):
                action = _validate_action(
                    edge.get(f"{arm}_action"), arm, parameters, index, grabbed,
                )
                if action is not None:
                    edge_calls.append((arm, action[0], action[1]))
            if not edge_calls:
                raise CheckNotApplicable("Every path edge must contain an arm action.")
            calls.append((edge, edge_calls))
        if not grabbed:
            raise CheckNotApplicable("This predictor requires an initial fork/spoon grasp.")
        if actions_module is None:
            from embodichain.agents.agentchord import atom_actions as actions_module
        # Check every callable before executing any SDK function.
        for _, edge_calls in calls:
            for _, fn, _ in edge_calls:
                if not callable(getattr(actions_module, fn, None)):
                    raise CheckNotApplicable(f"SDK action is unavailable: {fn}")
        shadow = ShadowEnv(env)
        for edge, edge_calls in calls:
            edge_report = {"id": edge["id"], "source": edge["source"], "target": edge["target"], "arms": []}
            report["edges"].append(edge_report)
            shadow.active_edge = edge["id"]
            for arm, fn, kwargs in edge_calls:
                shadow.active_arm, shadow.active_action = arm, fn
                arm_report = {"arm": arm, "fn": fn, "kwargs": serializable(kwargs), "status": "running"}
                edge_report["arms"].append(arm_report)
                first_ik = len(shadow.ik_calls)
                try:
                    trajectory = getattr(actions_module, fn)(env=shadow, **clone(kwargs))
                    arm_report["trajectory_steps"] = len(trajectory)
                    arm_report["status"] = "passed"
                except IKFailure as exc:
                    arm_report["status"] = "failed"
                    arm_report["error"] = str(exc)
                    raise
                except Exception as exc:
                    arm_report["status"] = "error"
                    arm_report["error"] = f"{type(exc).__name__}: {exc}"
                    raise
                finally:
                    arm_report["ik_calls"] = copy.deepcopy(shadow.ik_calls[first_ik:])
        report["status"] = "passed"
    except CheckNotApplicable as exc:
        report["status"], report["reason"] = "skipped", str(exc)
    except IKFailure as exc:
        report["status"], report["reason"] = "failed", str(exc)
    except Exception as exc:
        report["status"], report["error"] = "error", f"{type(exc).__name__}: {exc}"
    finally:
        if shadow is not None:
            report["ik_calls"] = copy.deepcopy(shadow.ik_calls)
            report["shadow_final_states"] = {
                arm: {
                    "qpos": serializable(getattr(shadow, f"{arm}_current_qpos")),
                    "xpos": serializable(getattr(shadow, f"{arm}_current_xpos")),
                    "gripper_state": serializable(getattr(shadow, f"{arm}_current_gripper_state")),
                }
                for arm in ("left_arm", "right_arm")
            }
    return report
