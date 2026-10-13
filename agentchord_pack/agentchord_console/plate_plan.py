"""Explicit plate-layout refinement for the fork/spoon demonstration."""
from __future__ import annotations
import copy
import hashlib
import json
import re

# Values are end-effector offsets in world axes relative to the plate's centre.
LAYOUTS = ((0.06, 0.08, 0.025), (0.08, 0.08, 0.025),
           (0.04, 0.08, 0.025), (0.06, 0.06, 0.025),
           (0.06, 0.10, 0.025), (0.06, 0.08, 0.035))


PLATE_GOAL = "将叉子和勺子放在餐盘里面"
PLATE_ALIASES = frozenset({"整理餐盘", "整理盘子", "收拾餐盘", "收拾盘子",
                           "整理桌子", "整理桌面", "收拾桌子", "收拾桌面", "摆好餐具"})


def resolve_plate_goal(instruction):
    text = instruction.strip().rstrip("。.!！").strip().lower()
    if text in PLATE_ALIASES:
        return PLATE_GOAL
    validate_plate_request(instruction)
    return instruction.strip()


def validate_plate_request(instruction):
    text = instruction.strip().rstrip("。.!！").strip().lower()
    if text in PLATE_ALIASES:
        return
    chinese = all(word in text for word in ("叉子", "勺子", "盘")) and any(
        word in text for word in ("里面", "盘内", "盘中", "盘里"))
    english = all(word in text for word in ("fork", "spoon", "plate")) and "inside" in text
    if not (chinese or english) or re.search(r"不要|别|不放|不能|not |outside", text):
        raise ValueError("当前为盘内双餐具演示模式，可输入‘整理餐盘’或‘将叉子和勺子放在餐盘里面’；其他任务请退出后去掉 --plate-layout 重启。")


def ordered_edges(graph):
    node_ids = [item["id"] for item in graph["nodes"]]
    ids = [item["id"] for item in graph["edges"]]
    if len(set(node_ids)) != len(node_ids) or len(set(ids)) != len(ids):
        raise ValueError("摆放布局需要不重复的节点和边 ID。")
    if graph["start"] not in node_ids or graph["goal"] not in node_ids:
        raise ValueError("摆放布局需要已定义起点与终点。")
    outgoing = {}
    for edge in graph["edges"]:
        if edge["source"] in outgoing or edge["source"] not in node_ids or edge["target"] not in node_ids:
            raise ValueError("摆放布局需要唯一、连续的任务路径。")
        outgoing[edge["source"]] = edge
    node, visited, result = graph["start"], set(), []
    while node != graph["goal"]:
        if node in visited or node not in outgoing:
            raise ValueError("摆放布局任务路径存在环或中断。")
        visited.add(node)
        edge = outgoing[node]
        result.append(edge)
        node = edge["target"]
    if len(result) != len(graph["edges"]):
        raise ValueError("摆放布局任务路径之外存在动作。")
    return result


def validate_source(bundle):
    graph = bundle.get("task_graph", bundle)
    if bundle.get("metadata", {}).get("recovery_enabled"):
        raise ValueError("摆放演示尚不支持恢复图。")
    recovery = bundle.get("recovery_graph") or {}
    spec = bundle.get("recovery_spec") or {}
    if any(recovery.get(key) for key in ("recovery_nodes", "recovery_edges", "recovery_branches")) or spec.get("bindings") or spec.get("recovery_bindings"):
        raise ValueError("摆放演示需要没有恢复绑定的任务图。")
    edges = ordered_edges(graph)
    if not edges:
        raise ValueError("摆放演示需要非空动作图。")
    if graph.get("max_transitions", 1000) < 6:
        raise ValueError("摆放演示需要 max_transitions 至少为 6；原图限制不足。")
    first = edges[0]
    for field, arm, obj in (("left_arm_action", "left_arm", "fork"),
                            ("right_arm_action", "right_arm", "spoon")):
        action = first.get(field) or {}
        params = action.get("kwargs", {})
        if action.get("fn") != "grasp" or params.get("robot_name") != arm or params.get("obj_name") != obj:
            raise ValueError("摆放演示需要首步左臂抓叉子、右臂抓勺子；当前计划不匹配。")
        if any(params.get(key) for key in ("force_valid", "real_world", "use_perception", "use_perception_pose")) or params.get("target_obj_pose") is not None:
            raise ValueError("摆放演示不接受抓取姿态覆盖或真实后端。")
    allowed = {"orient_eef", "move_relative_to_object", "open_gripper", "back_to_initial_pose"}
    moves, releases = set(), set()
    for edge in edges[1:]:
        for field, arm in (("left_arm_action", "left_arm"), ("right_arm_action", "right_arm")):
            action = edge.get(field)
            if not action:
                continue
            fn, params = action.get("fn"), action.get("kwargs", {})
            if fn not in allowed or params.get("robot_name") != arm:
                raise ValueError("当前计划包含摆放演示无法保留的额外动作。")
            if fn == "move_relative_to_object":
                if params.get("obj_name") != "plate":
                    raise ValueError("摆放演示要求移动目标为 plate。")
                moves.add(arm)
            if fn == "open_gripper":
                releases.add(arm)
    if moves != {"left_arm", "right_arm"} or releases != {"left_arm", "right_arm"}:
        raise ValueError("摆放演示要求计划中两臂都移动至盘子并松开夹爪。")
    return edges


def layout_candidate(bundle, separation, hover, release):
    source = validate_source(bundle)
    copied = copy.deepcopy(bundle)
    graph = copied.get("task_graph", copied)
    descriptions = ("本轮初始场景", "计划：两臂分别持有叉子与勺子", "计划：两臂末端朝下",
                    "计划：餐具位于盘子两侧上方", "计划：餐具降低至释放位置",
                    "计划：松开餐具", "计划：两臂返回初始姿态")
    nodes = [{"id": "plate_v" + str(i), "semantic": text} for i, text in enumerate(descriptions)]
    edges = []
    def edge(index, left, right):
        edges.append({"id": "plate_e" + str(index + 1), "source": nodes[index]["id"],
                      "target": nodes[index + 1]["id"], "left_arm_action": left, "right_arm_action": right})
    def action(fn, arm, **parameters):
        return {"fn": fn, "kwargs": {"robot_name": arm, **parameters}}
    edge(0, copy.deepcopy(source[0]["left_arm_action"]), copy.deepcopy(source[0]["right_arm_action"]))
    edge(1, action("orient_eef", "left_arm", direction="down"), action("orient_eef", "right_arm", direction="down"))
    for index, height in ((2, hover), (3, release)):
        edge(index, action("move_relative_to_object", "left_arm", obj_name="plate", x_offset=0.0, y_offset=separation, z_offset=height),
             action("move_relative_to_object", "right_arm", obj_name="plate", x_offset=0.0, y_offset=-separation, z_offset=height))
    edge(4, action("open_gripper", "left_arm"), action("open_gripper", "right_arm"))
    edge(5, action("back_to_initial_pose", "left_arm"), action("back_to_initial_pose", "right_arm"))
    graph.update(start=nodes[0]["id"], goal=nodes[-1]["id"], nodes=nodes, edges=edges)
    details = {"type": "explicit_plate_layout", "source_edge_ids": [item["id"] for item in source],
               "used_edge_ids": [item["id"] for item in edges], "end_effector_offsets": {
                   "left_y": separation, "right_y": -separation, "hover_z": hover, "release_z": release},
               "note": "由显式 --plate-layout 启用本地下层摆放模板；保留原首步抓取参数。目标是餐盘内分开摆放；可达性检查不判定物体实际摆放成功。"}
    if "task_graph" in copied:
        meta = copied.setdefault("metadata", {})
        payload = json.dumps(graph, ensure_ascii=False, sort_keys=True, separators=(",", ":"))
        meta["task_graph_hash"] = hashlib.sha256(payload.encode("utf-8")).hexdigest()
        meta["console_plate_layout"] = details
    return copied, details


def select_plate_layout(bundle, env, runtime=None, evaluate=None, progress=None):
    if evaluate is None:
        from agentchord_console.plan_check import evaluate_plan
        evaluate = evaluate_plan
    validate_source(bundle)
    attempts = []
    for index, (separation, hover, release) in enumerate(LAYOUTS, 1):
        candidate, change = layout_candidate(bundle, separation, hover, release)
        if progress:
            progress(index, len(LAYOUTS), change["end_effector_offsets"])
        report = evaluate(candidate, env, runtime=runtime)
        attempts.append({"layout": change["end_effector_offsets"], "check": report})
        if report["status"] == "passed":
            return candidate, {"status": "passed", "selected_candidate": index, "attempts": attempts,
                               "note": "整段原子动作生成检查通过，未执行物理步进；物体是否抓住、是否落在盘内仍需录像核验。"}, [change]
        if report["status"] != "failed":
            return copy.deepcopy(bundle), {"status": report["status"], "selected_candidate": None, "attempts": attempts}, []
    return copy.deepcopy(bundle), {"status": "failed", "selected_candidate": None, "attempts": attempts}, []
