"""Read-only IK preflight for the first grasp edge in a single simulation scene."""
from __future__ import annotations
import copy
import hashlib
import json
import math


class PreflightNotApplicable(ValueError):
    pass


def value_json(value):
    if hasattr(value, "detach"):
        value = value.detach().cpu()
    if hasattr(value, "tolist"):
        return value.tolist()
    return value


def sdk_context():
    import torch
    from embodichain.agents.agentchord.atom_action_utils import get_arm_states, resolve_object_pose
    from embodichain.lab.gym.utils.misc import get_rotation_replaced_pose
    from embodichain.utils.math import get_offset_pose
    return {"states": get_arm_states, "object_pose": resolve_object_pose,
            "rotate": get_rotation_replaced_pose, "offset": get_offset_pose,
            "atan2": torch.atan2, "allclose": torch.allclose}


def evaluate_grasp(env, robot_name, obj_name, distance, parameters, context=None):
    ctx = context or sdk_context()
    is_left, arm, qpos, current_pose, gripper = ctx["states"](env, robot_name)
    initial_pose = env.left_arm_init_xpos if is_left else env.right_arm_init_xpos
    if not ctx["allclose"](current_pose, initial_pose, rtol=1e-5, atol=1e-8):
        raise PreflightNotApplicable("当前末端姿态与本轮初始姿态不同。")
    initial_qpos = env.left_arm_init_qpos if is_left else env.right_arm_init_qpos
    if not ctx["allclose"](qpos, initial_qpos, rtol=1e-5, atol=1e-8):
        raise PreflightNotApplicable("当前关节状态与本轮初始状态不同。")
    if bool(gripper <= env.open_state - 0.01):
        raise PreflightNotApplicable("原抓取函数将先打开夹爪，跳过精确初始预检。")
    if parameters.get("force_valid", False):
        raise PreflightNotApplicable("启用了 force_valid，跳过精确姿态预检。")
    base = env.left_arm_base_pose if is_left else env.right_arm_base_pose
    obj_pose = ctx["object_pose"](env, obj_name, robot_name, parameters, reference=current_pose)
    aim_qpos = copy.deepcopy(qpos)
    delta = obj_pose[:2, 3] - base[:2, 3]
    aim_qpos[0] = ctx["atan2"](delta[1], delta[0])
    local_grasp = env.obj_info[obj_name]["grasp_pose_obj"]
    if float(local_grasp[0, 2]) > 0.5:
        cpu_pose = obj_pose.detach().cpu().numpy() if hasattr(obj_pose, "detach") else obj_pose
        rotated = ctx["rotate"](cpu_pose, float(aim_qpos[0]), "z", "intrinsic")
        obj_pose = obj_pose.new_tensor(rotated) if hasattr(obj_pose, "new_tensor") else rotated
    grasp_pose = obj_pose @ local_grasp
    pre_pose = ctx["offset"](copy.deepcopy(grasp_pose), -distance, "z", "intrinsic")
    pre_ok, pre_qpos = env.get_arm_ik(target_xpos=pre_pose, is_left=is_left, qpos_seed=aim_qpos)
    grasp_ok, grasp_qpos = None, None
    if bool(pre_ok):
        grasp_ok, grasp_qpos = env.get_arm_ik(target_xpos=grasp_pose, is_left=is_left, qpos_seed=pre_qpos)
    return {"pre_grasp_dis": distance, "pre1_success": bool(pre_ok),
            "grasp_success": bool(grasp_ok) if grasp_ok is not None else None,
            "base_pose": value_json(base), "object_pose": value_json(obj_pose),
            "initial_qpos": value_json(qpos), "aim_qpos": value_json(aim_qpos),
            "pre1_pose": value_json(pre_pose), "grasp_pose": value_json(grasp_pose),
            "pre1_qpos": value_json(pre_qpos) if bool(pre_ok) else None,
            "grasp_qpos": value_json(grasp_qpos) if grasp_ok else None}


def first_edge(graph):
    edges = [edge for edge in graph["edges"] if edge["source"] == graph["start"]]
    if len(edges) != 1:
        raise ValueError("首步抓取检查需要唯一首条动作边。")
    return edges[0]


def preflight_first_grasp(bundle, env, evaluate=None):
    """No env.step or agent-state changes; choose only exact IK-valid approach distances."""
    evaluate = evaluate or evaluate_grasp
    copied = copy.deepcopy(bundle)
    graph = copied.get("task_graph", copied)
    edge = first_edge(graph)
    checks, changes = [], []
    for field, arm in (("left_arm_action", "left_arm"), ("right_arm_action", "right_arm")):
        action = edge.get(field)
        if not action or action.get("fn") != "grasp":
            continue
        params = action.setdefault("kwargs", {})
        if params.get("robot_name") != arm or not params.get("obj_name"):
            raise ValueError("首步抓取的机械臂或物体参数不完整。")
        requested = float(params.get("pre_grasp_dis", 0.05))
        if not math.isfinite(requested) or requested <= 0:
            raise ValueError("pre_grasp_dis 必须是正的有限数值。")
        distances = []
        for distance in (requested, 0.05, 0.03):
            if distance <= requested and distance not in distances:
                distances.append(distance)
        attempts, selected, skipped = [], None, None
        for distance in distances:
            try:
                attempt = evaluate(env, arm, params["obj_name"], distance, params)
            except PreflightNotApplicable as exc:
                skipped = str(exc)
                break
            attempts.append(attempt)
            if attempt["pre1_success"] and attempt["grasp_success"]:
                selected = distance
                break
        checks.append({"arm": arm, "object": params["obj_name"], "edge_id": edge["id"],
                       "requested_distance": requested, "selected_distance": selected,
                       "attempts": attempts, "skipped": skipped})
        if selected is not None and selected != requested:
            params["pre_grasp_dis"] = selected
            changes.append({"type": "ik_checked_grasp_distance", "edge_id": edge["id"],
                            "arm": arm, "object": params["obj_name"],
                            "old_distance": requested, "new_distance": selected})
    skipped = [item for item in checks if item["skipped"]]
    failed = [item for item in checks if item["selected_distance"] is None and not item["skipped"]]
    report = {"status": "failed" if failed else ("skipped" if skipped else ("passed" if checks else "not_applicable")),
              "checks": checks, "failed_arms": [item["arm"] for item in failed],
              "note": "只检查首条边的精确预抓取与抓取 IK，不执行物理动作；可达不代表抓取成功。"}
    if skipped or failed:
        return copy.deepcopy(bundle), report, []
    if "task_graph" in copied and changes and not failed:
        metadata = copied.setdefault("metadata", {})
        payload = json.dumps(graph, ensure_ascii=False, sort_keys=True, separators=(",", ":"))
        metadata["task_graph_hash"] = hashlib.sha256(payload.encode("utf-8")).hexdigest()
        metadata["console_grasp_adjustments"] = changes
    return copied, report, changes if not failed else []
