"""Compile validated table instructions into reviewed SDK skills.

The language model supplies meaning, not executable Python. Candidate checking
is read-only; a selected graph still needs physical simulation and video review.
"""

from __future__ import annotations

import copy
import hashlib
import itertools
import json
import math

from agentchord_console.grasp_check import preflight_first_grasp
from agentchord_console.plan_check import evaluate_plan


OBJECT_ARMS = {"fork": "left_arm", "spoon": "right_arm"}
REGIONS = {"inside_plate", "beside_plate", "offset_plate", "shift_object"}
SIDES = {"auto", "left", "right", "front", "back", "center"}


class ClarificationRequired(ValueError):
    pass


def _hash(value):
    encoded = json.dumps(value, ensure_ascii=False, sort_keys=True,
                         separators=(",", ":"), allow_nan=False).encode("utf-8")
    return hashlib.sha256(encoded).hexdigest()


def _number(value, name, bound=.30):
    if (isinstance(value, bool) or not isinstance(value, (int, float)) or
            not math.isfinite(value) or abs(value) > bound):
        raise ValueError(f"{name} must be a finite number within ±{bound} metres.")
    return float(value)


def _placements(intent):
    if not isinstance(intent, dict) or intent.get("kind") != "execute":
        raise ValueError("Only a validated execute intent can build a task graph.")
    items = intent.get("placements")
    if not isinstance(items, list) or not 1 <= len(items) <= 2:
        raise ValueError("Select one or two fork/spoon placements.")
    seen = set()
    for item in items:
        if not isinstance(item, dict):
            raise ValueError("Every placement must be an object.")
        obj, region, side = item.get("object"), item.get("region"), item.get("side")
        if obj not in OBJECT_ARMS or obj in seen:
            raise ValueError("Objects must be distinct fork/spoon entries.")
        seen.add(obj)
        if region not in REGIONS or side not in SIDES:
            raise ValueError("Unsupported placement region or side.")
        offset = item.get("offset_m")
        if offset is not None:
            if not isinstance(offset, dict) or set(offset) != {"x", "y"}:
                raise ValueError("offset_m must contain exactly x and y in metres.")
            _number(offset["x"], "offset_m.x")
            _number(offset["y"], "offset_m.y")
        if region in ("offset_plate", "shift_object") and offset is None:
            raise ValueError("Explicit offset and shift instructions require offset_m.")
    return copy.deepcopy(items)


def _pose_xyz(env, obj):
    pose = env.sim.get_rigid_object(obj).get_local_pose(to_matrix=True)
    if hasattr(pose, "detach"):
        pose = pose.detach().cpu()
    if hasattr(pose, "tolist"):
        pose = pose.tolist()
    if isinstance(pose, (list, tuple)) and len(pose) == 1:
        pose = pose[0]
    if not isinstance(pose, (list, tuple)) or len(pose) != 4:
        raise ValueError(f"{obj} pose must be a 4×4 matrix, optionally with one environment dimension.")
    xyz = [float(pose[i][3]) for i in range(3)]
    if not all(math.isfinite(v) for v in xyz):
        raise ValueError(f"{obj} position is not finite.")
    return xyz


def _variants(items):
    flexible = any(item["region"] == "inside_plate" and item["side"] == "auto"
                   and item.get("offset_m") is None for item in items)
    separations = (.06, .08, .04) if flexible else (.06,)
    has_inside = any(item["region"] == "inside_plate" for item in items)
    release_heights = (.025, .035) if has_inside else (.02,)
    return [dict(separation_m=s, hover_height_m=h, inside_release_height_m=r)
            for s, h, r in itertools.product(separations, (.08, .06, .10), release_heights)]


def _target_offset(item, separation, initial, camera_frame=None):
    region, side, obj = item["region"], item["side"], item["object"]
    offset = item.get("offset_m")
    if region == "shift_object":
        return {
            "x": initial[obj][0] + float(offset["x"]) - initial["plate"][0],
            "y": initial[obj][1] + float(offset["y"]) - initial["plate"][1],
        }
    if offset is not None:
        return {"x": float(offset["x"]), "y": float(offset["y"])}
    # Camera-right is world +Y; camera-front is world +X.
    distance = .16 if region == "beside_plate" else (separation if side == "auto" else .06)
    if side == "auto":
        return {"x": 0., "y": distance if obj == "fork" else -distance}
    frame = camera_frame or {"screen_right_world": [0., 1., 0.], "screen_front_world": [1., 0., 0.]}
    if side in ("left", "right", "front", "back"):
        axis = frame["screen_right_world"] if side in ("left", "right") else frame["screen_front_world"]
        sign = -1 if side in ("left", "back") else 1
        return {"x": sign * distance * axis[0], "y": sign * distance * axis[1]}
    return {"x": 0., "y": 0.}


def _action(fn, arm, **kwargs):
    return {"fn": fn, "kwargs": {"robot_name": arm, **kwargs}}


def build_task_bundle(intent, env, variant=0, *, camera_frame=None):
    """Build a fresh graph using only initial object pose reads.

    Explicit sides and XY offsets stay fixed across variants. Variants change
    only unspecified inside spacing and approach/release heights.
    """
    items = _placements(intent)
    variants = _variants(items)
    if isinstance(variant, bool) or not isinstance(variant, int) or not 0 <= variant < len(variants):
        raise ValueError(f"variant must be an integer in [0, {len(variants)-1}].")
    layout = variants[variant]
    initial = {obj: _pose_xyz(env, obj) for obj in {"plate", *(item["object"] for item in items)}}
    targets = {item["object"]: _target_offset(item, layout["separation_m"], initial, camera_frame) for item in items}
    if len(items) == 2:
        a, b = [targets[item["object"]] for item in items]
        if abs(a["x"] - b["x"]) < 1e-8 and abs(a["y"] - b["y"]) < 1e-8:
            raise ClarificationRequired(
                "两件餐具被指定到同一个位置。请分别指定位置，或先只移动一件；本轮尚未执行。"
            )
    source = {"intent": copy.deepcopy(intent), "initial_xyz": initial, "camera_frame": camera_frame}
    source_hash = _hash(source)
    task_name = f"NLDesk_{source_hash[:16]}"
    stages = (
        ("grasp", "指定餐具的计划抓取状态"),
        ("orient", "指定餐具的计划朝下姿态"),
        ("hover", "指定目标上方的计划状态"),
        ("lower", "指定目标释放位置的计划状态"),
        ("release", "指定餐具释放后的计划状态"),
        ("return", "操作臂返回初始姿态的计划状态"),
    )
    nodes = [{"id": "v0_start", "semantic": "当前初始场景；仅操作指定餐具"}]
    nodes += [{"id": f"v{i}_{name}", "semantic": semantic}
              for i, (name, semantic) in enumerate(stages, 1)]
    edges = []
    for i, (stage, _) in enumerate(stages, 1):
        edge = {"id": f"e{i:02d}_{stage}", "source": nodes[i-1]["id"], "target": nodes[i]["id"],
                "left_arm_action": None, "right_arm_action": None}
        for item in items:
            obj, arm = item["object"], OBJECT_ARMS[item["object"]]
            xy = targets[obj]
            if stage == "grasp":
                atom = _action("grasp", arm, obj_name=obj, pre_grasp_dis=.1)
            elif stage == "orient":
                atom = _action("orient_eef", arm, direction="down")
            elif stage in ("hover", "lower"):
                z = layout["hover_height_m"] if stage == "hover" else (
                    layout["inside_release_height_m"] if item["region"] == "inside_plate" else .02)
                atom = _action("move_relative_to_object", arm, obj_name="plate",
                               x_offset=xy["x"], y_offset=xy["y"], z_offset=z)
            elif stage == "release":
                atom = _action("open_gripper", arm)
            else:
                atom = _action("back_to_initial_pose", arm)
            edge[f"{arm}_action"] = atom
        edges.append(edge)
    graph = {"task": task_name, "start": nodes[0]["id"], "goal": nodes[-1]["id"],
             "nodes": nodes, "edges": edges}
    return {
        "task_graph": graph,
        "recovery_spec": {"task": task_name, "recovery_bindings": []},
        "recovery_graph": {"task": task_name, "recovery_nodes": [], "recovery_edges": [], "recovery_branches": []},
        "metadata": {
            "task_graph_hash": _hash(graph), "recovery_enabled": False,
            "console_intent_source": source, "console_intent_source_hash": source_hash,
            "console_task_layout": {"variant": variant, "candidate_count": len(variants),
                                    "target_offsets_m": targets, **layout},
            "console_scope": "initial fork/spoon grasps followed by moves relative to a stationary plate",
        },
    }


def _layout_changes(baseline, chosen):
    before, after = baseline["metadata"]["console_task_layout"], chosen["metadata"]["console_task_layout"]
    if before == after:
        return []
    return [{"type": "ik_checked_task_layout", "old_layout": copy.deepcopy(before),
             "new_layout": copy.deepcopy(after),
             "note": "Explicit XY coordinates and explicit sides remain fixed."}]


def select_task_layout(intent, env, runtime=None, evaluate=None, progress=None, *, grasp_evaluate=None, camera_frame=None):
    """Select only a graph whose first grasp and full SDK preflight both pass.

    Return ``(bundle, report, changes)``. A non-passed report returns no bundle,
    making it impossible for callers to accidentally execute a failed candidate.
    ``grasp_evaluate`` and ``evaluate`` support mock-SDK local verification.
    """
    evaluator = evaluate_plan if evaluate is None else evaluate
    report = {"status": "failed", "attempts": [],
              "scope": "kinematic_targets_and_sdk_trajectories",
              "note": "This does not establish collision avoidance, grasp retention, or physical task success."}
    try:
        items = _placements(intent)
        count = len(_variants(items))
        baseline = build_task_bundle(intent, env, variant=0, camera_frame=camera_frame)
        for index in range(count):
            candidate = baseline if index == 0 else build_task_bundle(intent, env, variant=index, camera_frame=camera_frame)
            detail = copy.deepcopy(candidate["metadata"]["console_task_layout"])
            if progress is not None:
                progress(index + 1, count, detail)
            candidate, grasp_report, grasp_changes = preflight_first_grasp(
                candidate, env, evaluate=grasp_evaluate,
            )
            attempt = {"variant": index, "layout": detail, "grasp_check": grasp_report}
            report["attempts"].append(attempt)
            report["grasp_check"] = grasp_report
            if grasp_report.get("status") != "passed":
                # Initial grasp is identical for every layout. A failed/skipped
                # grasp cannot be repaired by changing future placement offsets.
                report["status"] = "failed" if grasp_report.get("status") == "failed" else "skipped"
                report["reason"] = "Initial grasp preflight did not pass; no candidate was executed."
                return None, report, []
            full = evaluator(candidate, env, runtime=runtime)
            attempt["plan_check"] = full
            attempt["status"] = full.get("status", "error")
            report["plan_check"] = full
            if full.get("status") == "passed":
                report["status"], report["selected_variant"] = "passed", index
                report["selected_layout"] = detail
                return candidate, report, _layout_changes(baseline, candidate) + copy.deepcopy(grasp_changes)
            if full.get("status") != "failed":
                report["status"] = full.get("status", "error")
                report["reason"] = full.get("reason", full.get("error", "SDK preflight did not pass."))
                return None, report, []
        report["reason"] = "All bounded layout candidates failed native IK; no candidate was executed."
        return None, report, []
    except ClarificationRequired as exc:
        report["status"], report["reason"] = "clarify", str(exc)
        report["question"] = "请给叉子和勺子分别指定位置，或先只移动一件。"
        return None, report, []
    except Exception as exc:
        report["status"], report["error"] = "error", f"{type(exc).__name__}: {exc}"
        return None, report, []
