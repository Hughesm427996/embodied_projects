#!/usr/bin/env python3
"""Interactive AgentChord tabletop simulation; uses the existing project environment."""
from __future__ import annotations

import argparse
import copy
from contextlib import contextmanager
import datetime as dt
import getpass
import hashlib
import json
import math
import os
from pathlib import Path
import re
import shutil
import subprocess
import sys
import traceback
import warnings

# Make the bundled helper modules importable when invoked by script path.
sys.path.insert(0, str(Path(__file__).resolve().parent.parent))

BASELINE_TASK = "Rearrangement_DS_Normal"
CONFIG_DIR = Path("configs/gym/agent/rearrangement_agent")
CACHE_DIR = Path("embodichain/database/agent_generated_content/sim")
ALIASES = {"1", "整理桌子", "整理桌面", "收拾桌子", "收拾桌面", "摆好餐具", "tidy table"}
DEFAULT_GOAL = (
    "整理当前桌面上的叉子和勺子：将叉子摆到盘子中心世界坐标 y 正方向 0.16 米处，"
    "将勺子摆到盘子中心 y 负方向 0.16 米处，两者 x 与盘子中心对齐。"
    "使用双臂抓取并移动餐具，然后松开夹爪，让餐具留在桌面上。"
)


def write_json(path: Path, value):
    secrets = [os.environ[name] for name in ("OPENAI_API_KEY", "DEEPSEEK_API_KEY")
               if os.environ.get(name)]
    def redact(item):
        if isinstance(item, str):
            for secret in secrets:
                item = item.replace(secret, "[API_KEY_REDACTED]")
            return item
        if isinstance(item, dict):
            return {str(k): redact(v) for k, v in item.items()}
        if isinstance(item, (tuple, list)):
            return [redact(v) for v in item]
        return item
    path.write_text(json.dumps(redact(value), ensure_ascii=False, indent=2), encoding="utf-8")


def normalize(text: str) -> str:
    return text.strip().rstrip("。.!！").strip().lower()


def project_paths(root: Path):
    root = root.expanduser().resolve()
    python = root / ".venv/bin/python"
    agent_config = root / CONFIG_DIR / "agent_config.json"
    scene = root / CONFIG_DIR / "first_agent_gym_config.json"
    if not scene.is_file():
        scene = root / CONFIG_DIR / "fast_gym_config.json"
    for path in (python, scene, agent_config):
        if not path.is_file():
            raise RuntimeError(f"未找到 {path}。请将脚本包放进已经跑通的 agentchord_pack 项目。")
    return root, python, scene, agent_config



def recording_frame(params):
    """Horizontal viewer axes from the recorded camera's eye and target."""
    eye, target = params.get("eye", [2, 0, 2]), params.get("target", [0.5, 0, 1])
    dx, dy = float(eye[0]) - float(target[0]), float(eye[1]) - float(target[1])
    length = math.hypot(dx, dy)
    if not math.isfinite(length) or length < 1e-8:
        raise ValueError("录像相机没有明确的水平观看方向，无法解释左右/前后。")
    return {"description": "本轮 cam1 录像视角", "screen_right_world": [-dy / length, dx / length, 0],
            "screen_front_world": [dx / length, dy / length, 0]}


def prepare_run(root: Path, instruction: str, force: bool, resolution, hold: bool, environ,
                seed=0, stable_scene=False, grasp_check=True, plate_layout=False,
                natural_language=False, history=None, pick_carry=False, disturbance_mode=None,
                llm_recovery=False, drop_mode=None):
    if drop_mode not in (None,"off","on"):
        raise ValueError("drop_mode must be off or on.")
    if drop_mode is not None and (disturbance_mode is not None or natural_language or plate_layout or pick_carry):
        raise ValueError("Drop validation has its own isolated mode.")
    if drop_mode is not None and llm_recovery is not (drop_mode == "on"):
        raise ValueError("Drop-on requires API recovery; drop-off must not call API.")
    if disturbance_mode not in (None, "off", "on"):
        raise ValueError("disturbance_mode must be off or on.")
    if disturbance_mode is not None and (natural_language or plate_layout or pick_carry):
        raise ValueError("Disturbance validation has its own isolated mode.")
    if llm_recovery and disturbance_mode != "on" and drop_mode != "on":
        raise ValueError("LLM recovery generation is only available in the recovery-on disturbance mode.")
    if pick_carry and (natural_language or plate_layout):
        raise ValueError("Controlled carry validation cannot be combined with language or plate-layout modes.")
    root, python, scene, agent_config = project_paths(root)
    canonical = not natural_language and normalize(instruction) in ALIASES
    goal = DEFAULT_GOAL if canonical else instruction.strip()
    if plate_layout and not natural_language:
        from agentchord_console.plate_plan import resolve_plate_goal
        goal = resolve_plate_goal(instruction)
        canonical = False  # Plate-mode aliases must never reuse the side-placement baseline.
    scene_data = json.loads(scene.read_text(encoding="utf-8"))
    if scene_data.get("id") != "RearrangementAgent-v3":
        raise RuntimeError("当前配置不是 RearrangementAgent-v3，请使用原 Agent 场景配置。")
    model = environ.get("AGENTCHORD_MODEL", "deepseek-flash")
    base_url = environ.get("LLM_URL", "https://api.deepseek.com")
    fingerprint = hashlib.sha256()
    mode_version = "controlled_drop_v3_"+drop_mode if drop_mode else ("llm_recovery_v3" if llm_recovery else ("controlled_displacement_v4_" + disturbance_mode if disturbance_mode else (
        "controlled_pick_carry_v1" if pick_carry else ("console_nl_v1" if natural_language else "console_v1"))
    ))
    for value in (mode_version, goal, model, base_url):
        fingerprint.update(value.encode("utf-8") + b"\0")
    fingerprint.update(scene.read_bytes())
    fingerprint.update(agent_config.read_bytes())
    for prompt in sorted((root / CONFIG_DIR).glob("*.txt")):
        fingerprint.update(prompt.name.encode() + prompt.read_bytes())
    baseline = root / CACHE_DIR / BASELINE_TASK / "agent_task_graph.json"
    reuse = drop_mode is None and disturbance_mode is None and not pick_carry and not natural_language and not plate_layout and canonical and not force and baseline.is_file()
    task_name = BASELINE_TASK if reuse else "ConsoleDesk_" + fingerprint.hexdigest()[:16]
    stamp = dt.datetime.now().strftime("%Y%m%d_%H%M%S_%f")
    run_dir = root / "outputs/command_console" / stamp
    run_dir.mkdir(parents=True, exist_ok=False)
    cfg = copy.deepcopy(scene_data)
    cfg["max_episodes"] = 1
    if disturbance_mode is not None or drop_mode is not None:
        cfg["max_episode_steps"] = 1200
    removed_pose_events = []
    if stable_scene:
        events = cfg.get("env", {}).get("events", {})
        for name in ("init_fork_pose", "init_spoon_pose", "random_robot_init_eef_pose"):
            if name in events:
                del events[name]
                removed_pose_events.append(name)
    camera = cfg.get("env", {}).get("events", {}).get("record_camera")
    if not camera or camera.get("func") != "record_camera_data":
        raise RuntimeError("场景缺少 record_camera_data 录像配置。")
    params = camera.setdefault("params", {})
    old_w, old_h = params.get("resolution", [640, 480])
    width, height = resolution
    intrinsics = params.get("intrinsics", [600, 600, old_w / 2, old_h / 2])
    sx, sy = width / old_w, height / old_h
    params["resolution"] = [width, height]
    params["intrinsics"] = [intrinsics[0] * sx, intrinsics[1] * sy,
                             intrinsics[2] * sx, intrinsics[3] * sy]
    params["save_path"] = str(run_dir / "videos")
    gym_config = run_dir / "gym_config.json"
    write_json(gym_config, cfg)
    shutil.copy2(agent_config, run_dir / "agent_config.snapshot.json")
    manifest = {
        "project": str(root), "run_dir": str(run_dir),
        "instruction": instruction, "task_prompt": goal,
        "task_name": task_name, "reuse_existing_plan": reuse,
        "regenerate": force, "hold": hold,
        "model": model, "base_url": base_url,
        "gym_config": str(gym_config), "agent_config": str(agent_config),
        "device": "cuda:0", "gpu_id": 0, "num_envs": 1,
        "recovery": disturbance_mode == "on" or drop_mode == "on", "seed": seed, "stable_scene": stable_scene,
        "removed_pose_events": removed_pose_events, "grasp_check": grasp_check,
        "plate_layout": plate_layout and not natural_language,
        "natural_language": natural_language, "dialogue_history": history or [],
        "pick_carry": pick_carry,
        "disturbance_mode": disturbance_mode,
        "llm_recovery": bool(llm_recovery),
        "drop_mode":drop_mode,
        "camera_frame": recording_frame(params) if natural_language else None,
    }
    write_json(run_dir / "run.json", manifest)
    return python, run_dir, manifest


PROXY_VARIABLES = ("HTTP_PROXY", "HTTPS_PROXY", "ALL_PROXY",
                   "http_proxy", "https_proxy", "all_proxy")


def child_environment(python: Path, environ, no_proxy: bool = False):
    env = dict(environ)
    if no_proxy:
        for name in PROXY_VARIABLES:
            env.pop(name, None)
    lib = subprocess.check_output(
        [str(python), "-c", "import sys; print(sys.base_prefix + '/lib')"],
        env=env, text=True, encoding="utf-8",
    ).strip()
    previous = env.get("LD_LIBRARY_PATH", "")
    env["LD_LIBRARY_PATH"] = lib + (os.pathsep + previous if previous else "")
    env["PYTHONUNBUFFERED"] = "1"
    env["PYTHONIOENCODING"] = "utf-8"
    env.setdefault("LLM_URL", "https://api.deepseek.com")
    env.setdefault("AGENTCHORD_MODEL", "deepseek-flash")
    return env


def relay_output(source, log, secret: str, output):
    """Forward CR/LF-delimited records, including prompts, without logging the key."""
    pending = b""
    key = secret.encode("utf-8") if secret else b""
    def emit(record):
        if key:
            record = record.replace(key, b"[API_KEY_REDACTED]")
        log.write(record)
        log.flush()
        output.write(record.decode("utf-8", errors="replace"))
        output.flush()
    while True:
        chunk = source.read1(4096)
        if not chunk:
            break
        pending += chunk
        while True:
            delimiter = re.search(rb"[\r\n]", pending)
            if not delimiter:
                break
            end = delimiter.end()
            emit(pending[:end])
            pending = pending[end:]
    if pending:
        emit(pending)


def run_command(root: Path, instruction: str, force: bool, resolution, hold: bool, no_proxy: bool = False,
                seed=0, stable_scene=False, grasp_check=True, plate_layout=False,
                natural_language=False, history=None, pick_carry=False, disturbance_mode=None,
                llm_recovery=False, drop_mode=None):
    if not os.environ.get("DISPLAY"):
        raise RuntimeError("请在 2080 的 Ubuntu 图形桌面终端运行；当前没有 DISPLAY。")
    root, python, _, _ = project_paths(root)
    env = child_environment(python, os.environ, no_proxy=no_proxy)
    if no_proxy:
        print("本轮仿真子进程将直接联网，已忽略代理环境变量。", flush=True)
    from agentchord_console.credentials import resolve_key
    # Controlled SDK diagnostics do not call the language service. A placeholder
    # satisfies SDK client construction without requesting or persisting a key.
    controlled = (pick_carry or disturbance_mode is not None or drop_mode is not None) and not llm_recovery
    key = "controlled-demo-no-api-call" if controlled else resolve_key(env)
    if not key:
        key = getpass.getpass("输入 DeepSeek API 密钥（不显示，只保留在本次进程）： ").strip()
        if not key:
            raise RuntimeError("未输入 API 密钥。")
    # The parent JSON redactor must also know a key loaded from the private file.
    if not controlled:
        os.environ["DEEPSEEK_API_KEY"] = key
    env["OPENAI_API_KEY"] = key
    python, run_dir, manifest = prepare_run(root, instruction, force, resolution, hold, env,
                                            seed=seed, stable_scene=stable_scene, grasp_check=grasp_check, plate_layout=plate_layout,
                                            natural_language=natural_language, history=history, pick_carry=pick_carry,
                                            disturbance_mode=disturbance_mode, llm_recovery=llm_recovery, drop_mode=drop_mode)
    print("\n正在理解本次输入。" if natural_language else ("\n开始整理桌面。" if manifest["reuse_existing_plan"] else "\n按本次指令规划并运行桌面任务。"), flush=True)
    print(f"本轮结果：{run_dir}", flush=True)
    command = [str(python), "-u", str(Path(__file__).resolve()),
               "--worker", str(run_dir / "run.json")]
    process = subprocess.Popen(command, cwd=root, env=env,
                               stdout=subprocess.PIPE, stderr=subprocess.STDOUT)
    try:
        with (run_dir / "run.log").open("wb") as log:
            relay_output(process.stdout, log, key, sys.stdout)
        code = process.wait()
    except KeyboardInterrupt:
        if process.poll() is None:
            process.terminate()
            try:
                process.wait(timeout=8)
            except subprocess.TimeoutExpired:
                process.kill()
                process.wait()
        code = 130
        print("\n本轮已停止。")
    finally:
        if process.stdout:
            process.stdout.close()
    write_json(run_dir / "process_status.json", {"exit_code": code})
    if code != 0:
        print(f"本轮未正常完成（退出码 {code}），报错保存在 {run_dir / 'run.log'}")
    else:
        result_path = run_dir / "result.json"
        result = json.loads(result_path.read_text(encoding="utf-8")) if result_path.is_file() else {}
        if result.get("interaction_kind") in {"clarify", "reply", "unsupported"}:
            print("可以继续在‘指令 >’输入补充说明或新的要求。")
        else:
            print(f"本轮流程结束。输入“视频”查看录像；结果目录：{run_dir}")
    return run_dir, code


def json_value(value):
    if hasattr(value, "detach"):
        value = value.detach().cpu()
    if hasattr(value, "tolist"):
        return value.tolist()
    if isinstance(value, dict):
        return {str(k): json_value(v) for k, v in value.items()}
    if isinstance(value, (tuple, list)):
        return [json_value(v) for v in value]
    return value


def object_poses(env):
    result = {}
    for name in ("plate", "fork", "spoon"):
        matrix = json_value(env.sim.get_rigid_object(name).get_local_pose(to_matrix=True).squeeze(0))
        result[name] = {"matrix": matrix, "xyz": [matrix[i][3] for i in range(3)]}
    return result


def save_frame(recorder, path: Path):
    from PIL import Image
    recorder.camera.update()
    color = recorder.camera.get_data()["color"]
    if hasattr(color, "detach"):
        color = color.detach().cpu().numpy()
    Image.fromarray(color[0, ..., :3]).save(path)


ACTION_LABELS = {
    "grasp": "抓取", "orient_eef": "调整夹爪朝向",
    "move_relative_to_object": "移动到物体附近的指定位置",
    "move_to_position": "移动到指定位置", "open_gripper": "松开夹爪",
    "close_gripper": "闭合夹爪", "return_to_init": "返回初始姿态",
    "back_to_initial_pose": "返回初始姿态",
}


def graph_structure(graph):
    nodes, edges = graph["nodes"], graph["edges"]
    node_ids = [node["id"] for node in nodes]
    edge_ids = [edge["id"] for edge in edges]
    if len(set(node_ids)) != len(node_ids) or len(set(edge_ids)) != len(edge_ids):
        raise ValueError("动作图的节点或边 ID 重复，请重新规划。")
    if graph["start"] not in node_ids or graph["goal"] not in node_ids:
        raise ValueError("动作图起点或终点未定义，请重新规划。")
    outgoing = {}
    for edge in edges:
        if edge["source"] not in node_ids or edge["target"] not in node_ids:
            raise ValueError("动作图引用了未定义节点，请重新规划。")
        if edge["source"] in outgoing:
            raise ValueError("动作图有多个出边，无法确定唯一执行顺序。")
        if not edge.get("left_arm_action") and not edge.get("right_arm_action"):
            raise ValueError("动作边没有任何机械臂动作，请重新规划。")
        outgoing[edge["source"]] = edge
    return outgoing


def is_return_action(edge):
    found = False
    for field, arm in (("left_arm_action", "left_arm"), ("right_arm_action", "right_arm")):
        action = edge.get(field)
        if action:
            found = True
            if action.get("fn") != "back_to_initial_pose" or action.get("kwargs", {}).get("robot_name") != arm:
                return False
    return found


def normalize_graph_goal(bundle):
    """Repair only a terminal return chain/self-loop; retain every action."""
    graph = bundle.get("task_graph", bundle)
    outgoing = graph_structure(graph)
    try:
        plan_summary(bundle)
        return copy.deepcopy(bundle), []
    except ValueError as original_error:
        message = str(original_error)
    recovery_spec = bundle.get("recovery_spec") or {}
    recovery_graph = bundle.get("recovery_graph") or {}
    if (bundle.get("metadata", {}).get("recovery_enabled") or
        recovery_spec.get("recovery_bindings") or recovery_spec.get("bindings") or
        any(recovery_graph.get(key) for key in ("recovery_nodes", "recovery_edges", "recovery_branches"))):
        raise ValueError(f"{message}；启用了恢复配置，不能自动校正终点。")
    corrected = copy.deepcopy(bundle)
    changed_graph = corrected.get("task_graph", corrected)
    goal = graph["goal"]
    tail = outgoing.get(goal)
    if (tail and tail["target"] == goal and is_return_action(tail) and
        tail.get("left_arm_action") and tail.get("right_arm_action")):
        prefix = copy.deepcopy(graph)
        prefix["edges"] = [edge for edge in prefix["edges"] if edge["id"] != tail["id"]]
        plan_summary(prefix)
        new_goal = str(goal) + "_after_return"
        defined = {node["id"] for node in graph["nodes"]}
        suffix = 2
        while new_goal in defined:
            new_goal = str(goal) + "_after_return_" + str(suffix)
            suffix += 1
        old_goal_node = next(node for node in graph["nodes"] if node["id"] == goal)
        changed_graph["nodes"].append({"id": new_goal, "semantic": "返回初始姿态后的计划状态：" + old_goal_node.get("semantic", "")})
        next(edge for edge in changed_graph["edges"] if edge["id"] == tail["id"])["target"] = new_goal
        changed_graph["goal"] = new_goal
        changes = [{"type": "terminal_return_self_loop", "edge_id": tail["id"],
                    "old_goal": goal, "new_goal": new_goal,
                    "old_target": goal, "new_target": new_goal,
                    "added_node": new_goal}]
    else:
        current = graph["start"]
        visited, traversed, after_goal = set(), [], False
        return_tail = []
        while current in outgoing:
            if current in visited:
                raise ValueError(f"{message}；检测到非终点返回自环，不能自动修复。")
            visited.add(current)
            if current == goal:
                after_goal = True
            edge = outgoing[current]
            traversed.append(edge["id"])
            if after_goal:
                return_tail.append(edge)
            current = edge["target"]
        if (len(traversed) != len(graph["edges"]) or not return_tail or
            not all(is_return_action(edge) for edge in return_tail)):
            raise ValueError(f"{message}；无法确认所有额外边都是连续返回动作，请重新规划。")
        changed_graph["goal"] = current
        changes = [{"type": "early_goal_before_return", "old_goal": goal,
                    "new_goal": current, "return_edges": [edge["id"] for edge in return_tail]}]
    plan_summary(corrected)
    if "task_graph" in corrected:
        metadata = corrected.setdefault("metadata", {})
        payload = json.dumps(changed_graph, ensure_ascii=False, sort_keys=True, separators=(",", ":"))
        metadata["task_graph_hash"] = hashlib.sha256(payload.encode("utf-8")).hexdigest()
        metadata["console_graph_correction"] = changes
    return corrected, changes


def plan_summary(bundle):
    """Summarize public graph actions, never internal model reasoning fields."""
    graph = bundle.get("task_graph", bundle)
    nodes = {node["id"]: node.get("semantic", "") for node in graph["nodes"]}
    outgoing = graph_structure(graph)
    current, goal = graph["start"], graph["goal"]
    steps, visited = [], set()
    while current != goal:
        if current in visited or current not in outgoing:
            raise ValueError("计划路径存在环或中断。")
        visited.add(current)
        edge = outgoing[current]
        step = {"edge_id": edge["id"], "source": current, "target": edge["target"],
                "target_description": nodes.get(edge["target"], ""), "actions": []}
        for arm, key in (("左臂", "left_arm_action"), ("右臂", "right_arm_action")):
            action = edge.get(key)
            if action:
                fn = str(action["fn"])
                step["actions"].append({"arm": arm, "function": fn,
                                        "label": ACTION_LABELS.get(fn, fn),
                                        "parameters": action.get("kwargs", {})})
        steps.append(step)
        current = edge["target"]
    if len(steps) != len(graph["edges"]):
        raise ValueError("计划存在主路径之外的边。")
    return {"task": graph.get("task", ""), "initial_description": nodes.get(graph["start"], ""),
            "goal_description": nodes.get(goal, ""), "steps": steps,
            "note": "节点文字是计划中的目标状态；实际执行结果需看视频和最终判定。"}


class PlanningDisplay:
    def __init__(self, run_dir: Path, grasp_check=False, plate_layout=False):
        self.run_dir = run_dir
        self.grasp_check = grasp_check
        self.plate_layout = plate_layout
        self.raw = None
        self.events = []
        self.plan = None

    def emit(self, phase, message, **details):
        self.events.append({"time": dt.datetime.now().isoformat(timespec="seconds"),
                            "phase": phase, "message": message, **details})
        print(f"\n[{phase}] {message}", flush=True)
        write_json(self.run_dir / "planning_trace.json", {"events": self.events, "plan": self.plan})

    def prepare_graph(self, graph_file_path, runtime_kwargs=None):
        original_path = Path(graph_file_path)
        bundle = json.loads(original_path.read_text(encoding="utf-8"))
        shutil.copy2(original_path, self.run_dir / "agent_compiled_graph.original.json")
        corrected, changes = normalize_graph_goal(bundle)
        if self.grasp_check:
            from agentchord_console.grasp_check import preflight_first_grasp
            runtime = runtime_kwargs or {}
            override_keys = {"force_valid", "robot_name", "obj_name", "pre_grasp_dis", "target_obj_pose",
                             "perception", "use_perception", "use_perception_pose", "real_world"}
            if (any(runtime.get(key) is not None and runtime.get(key) is not False for key in override_keys) or
                runtime.get("action_backend") not in (None, "sim", "simulation")):
                self.emit("检查", "运行参数覆盖抓取属性，跳过首步精确 IK 预检。")
            else:
                self.emit("检查", "计算首步精确预抓取与抓取 IK，暂不执行物理动作。")
                try:
                    corrected, grasp_report, grasp_changes = preflight_first_grasp(corrected, self.raw)
                except Exception as exc:
                    write_json(self.run_dir / "grasp_check.json", {"status": "error", "error": f"{type(exc).__name__}: {exc}"})
                    raise
                write_json(self.run_dir / "grasp_check.json", grasp_report)
                if grasp_report["status"] == "failed":
                    arms = ", ".join(grasp_report["failed_arms"])
                    raise RuntimeError(f"首步抓取可达性检查未通过（{arms}）。请查看 grasp_check.json；本轮未执行抓取。")
                for check in grasp_report["checks"]:
                    if check["skipped"]:
                        self.emit("检查", f"{check['arm']} 跳过预检：{check['skipped']}")
                    elif check["selected_distance"] is not None:
                        suffix = "本轮未应用候选调整。" if grasp_report["status"] == "skipped" else "本轮采用此距离。"
                        self.emit("检查", f"{check['arm']} / {check['object']} 预抓取与抓取均可解，接近距离 {check['selected_distance']:.3f} 米；{suffix}")
                changes += grasp_changes
        if self.plate_layout:
            from agentchord_console.plate_plan import select_plate_layout
            self.emit("布局", "本轮启用本地下层摆放模板：末端朝下，两臂目标分开，执行前检查整段动作。")
            def progress(index, total, offsets):
                self.emit("检查", f"检查摆放候选 {index}/{total}（世界坐标偏移，单位米）。", offsets=offsets)
            try:
                from embodichain.agents.hierarchy.compile_agent import _runtime_kwargs
                effective = _runtime_kwargs(runtime_kwargs or {}, getattr(self.raw.compile_agent, "prompt_kwargs", {}))
                runtime_env = effective.pop("env", self.raw)
                if runtime_env is not self.raw:
                    raise RuntimeError("动作运行环境与当前场景不同，停止布局预检。")
                corrected, plan_report, layout_changes = select_plate_layout(
                    corrected, self.raw, runtime=effective, progress=progress)
            except Exception as exc:
                write_json(self.run_dir / "plan_check.json", {"status": "error", "error": f"{type(exc).__name__}: {exc}"})
                raise
            write_json(self.run_dir / "plan_check.json", plan_report)
            if plan_report["status"] != "passed":
                raise RuntimeError(f"整段摆放检查未通过（{plan_report['status']}），请查看 plan_check.json；本轮未执行物理动作。")
            changes += layout_changes
            self.emit("检查", "选定布局的原子动作均可生成；尚未验证碰撞、实际夹持和盘内摆放效果。")
        used_path = self.run_dir / "agent_compiled_graph.used.json"
        write_json(used_path, corrected)
        if changes:
            write_json(self.run_dir / "graph_correction.json", {
                "source": str(original_path), "used_graph": str(used_path), "changes": changes,
                "note": "记录终点连接、IK 检查的抓取距离及显式启用的本地摆放布局；原缓存未修改。",
            })
            self.emit("修复", "本轮执行图调整已记录到 graph_correction.json。", changes=changes)
        self.emit("检查", "动作图路径检查通过；执行时仍使用项目原校验器。")
        return used_path

    def show_plan(self, graph_file_path):
        bundle = json.loads(Path(graph_file_path).read_text(encoding="utf-8"))
        self.plan = plan_summary(bundle)
        self.emit("计划", f"已读取本轮实际动作图，共 {len(self.plan['steps'])} 步。",
                  graph_file=str(graph_file_path))
        for index, step in enumerate(self.plan["steps"], 1):
            parts = []
            for action in step["actions"]:
                params = json.dumps(action["parameters"], ensure_ascii=False)
                parts.append(f"{action['arm']}：{action['label']} ({action['function']}; {params})")
            print(f"  第 {index} 步 [{' / '.join(parts)}]", flush=True)
            if step["target_description"]:
                print(f"    计划目标：{step['target_description']}", flush=True)
            actions = step["actions"]
            if len(actions) == 2 and all(action["function"] == "move_relative_to_object" for action in actions):
                left, right = (action["parameters"] for action in actions)
                if (left.get("obj_name") == right.get("obj_name") and
                    all(left.get(key, 0.0) == right.get(key, 0.0) for key in ("x_offset", "y_offset", "z_offset"))):
                    self.emit("提示", f"第 {index} 步两臂目标位置相同。请通过录像检查夹爪碰撞或物体重叠；当前保留原计划参数。")
        print("  注：以上是计划，物体是否抓住、摆放是否合格，以实际结果为准。", flush=True)
        write_json(self.run_dir / "plan_summary.json", self.plan)

    def attach(self, raw):
        self.raw = raw
        agent = getattr(raw, "compile_agent", None)
        original = getattr(agent, "act", None)
        if not callable(original):
            self.emit("提示", "当前版本没有可用的计划展示接口，继续运行原流程。")
            return lambda: None
        def act_with_display(graph_file_path, **kwargs):
            graph_file_path = self.prepare_graph(graph_file_path, kwargs)
            try:
                self.show_plan(graph_file_path)
            except Exception as exc:
                self.emit("提示", f"计划摘要读取失败，继续原动作流程：{exc}")
            self.emit("执行", "开始执行动作图；下方原项目日志显示轨迹进度。")
            result = original(graph_file_path, **kwargs)
            self.emit("执行", "动作图执行函数已返回，随后检查本轮结果。")
            return result
        agent.act = act_with_display
        def restore():
            agent.act = original
        return restore



@contextmanager
def cpu_planner_inputs():
    """Use CPU planner inputs and align measured grippers with SDK thresholds.

    Native monitor-triggered synchronization reads CUDA robot joint values.
    Its cached grippers must share the open-state threshold's device before
    the next native grasp. Their measured values and shapes are preserved;
    arm joint/pose caches remain on their original device for IK/FK.
    """
    from embodichain.agents.agentchord import atom_actions, atom_action_utils
    original_actions = atom_actions.plan_trajectory
    original_utils = atom_action_utils.plan_trajectory
    if original_actions is not original_utils:
        raise RuntimeError("轨迹接口存在额外包装，无法保证预检与执行一致。")
    original_sync = atom_actions.sync_agent_state_from_robot
    original_utils_sync = atom_action_utils.sync_agent_state_from_robot
    if original_sync is not original_utils_sync:
        raise RuntimeError("机器人状态同步接口存在额外包装，无法保证原生同步一致。")
    def cpu_waypoints(env, select_arm, qpos_list, sample_num, gripper, qpos_output, gripper_output):
        waypoints = [item.detach().cpu() if hasattr(item, "detach") else item for item in qpos_list]
        return original_actions(env, select_arm, waypoints, sample_num, gripper, qpos_output, gripper_output)
    def sync_grippers_on_threshold_device(env):
        result = original_sync(env)
        for side in ("left", "right"):
            name = f"{side}_arm_current_gripper_state"
            state = getattr(env, name)
            normalized = state.detach().to(device=env.open_state.device, dtype=env.open_state.dtype)
            setattr(env, name, normalized)
            if state.device != normalized.device:
                print(f"[兼容] 原生状态同步后 {name}：{state.device} → {normalized.device}，保留实测夹爪数值。", flush=True)
        return result
    atom_actions.plan_trajectory = cpu_waypoints
    atom_action_utils.plan_trajectory = cpu_waypoints
    atom_actions.sync_agent_state_from_robot = sync_grippers_on_threshold_device
    atom_action_utils.sync_agent_state_from_robot = sync_grippers_on_threshold_device
    try:
        yield
    finally:
        atom_actions.plan_trajectory = original_actions
        atom_action_utils.plan_trajectory = original_utils
        atom_actions.sync_agent_state_from_robot = original_sync
        atom_action_utils.sync_agent_state_from_robot = original_utils_sync


def simulation_worker(manifest_path: Path):
    manifest = json.loads(manifest_path.read_text(encoding="utf-8"))
    root = Path(manifest["project"])
    run_dir = Path(manifest["run_dir"])
    os.chdir(root)
    sys.path.insert(0, str(root))
    env = None
    recorder = None
    raw = None
    motion_trace = None
    recovery_observer = None
    display = PlanningDisplay(run_dir, grasp_check=manifest.get("grasp_check", True) and not manifest.get("natural_language", False) and not manifest.get("pick_carry", False) and manifest.get("disturbance_mode") is None and manifest.get("drop_mode") is None,
                              plate_layout=manifest.get("plate_layout", False))
    planner_context = None
    restore_display = lambda: None
    report = {"instruction": manifest["instruction"], "execution_completed": False,
              "original_scene_success": None,
              "metric_note": "原 RearrangementAgent-v3 判定只检查叉子/勺子相对盘子的 y 位置（±0.16 米，容差 0.02 米）；未检查 x、z、朝向或是否稳放桌面。执行完成另行记录。"}
    exit_code = 1
    interpreted = None
    try:
        display.emit("目标", manifest["task_prompt"], instruction=manifest["instruction"])
        if manifest.get("pick_carry", False):
            display.emit("测试", "受控抓取、抬升、搬运、释放、返回测试；本轮不调用规划 API，不注入扰动。")
        if manifest.get("disturbance_mode") is not None:
            message = ("官方 RecoveryAgent 调用 DeepSeek 生成恢复方案；执行前编译和预检。"
                       if manifest.get("llm_recovery") else "使用原监测与图运行时，不调用 API。")
            display.emit("测试", f"受控物体移位对照：恢复 {manifest['disturbance_mode']}；{message}")
        if manifest.get("drop_mode") is not None:
            display.emit("测试", f"松爪掉落对照：恢复 {manifest['drop_mode']}。只改变左夹爪指令，由物理仿真产生掉落；原位置监测器检测。")
        if manifest.get("natural_language", False):
            from openai import OpenAI
            from agentchord_console.language import resolve_intent
            display.emit("理解", "正在把你的自然语言要求交给 DeepSeek 理解。")
            text_client = OpenAI(api_key=os.environ["OPENAI_API_KEY"], base_url=manifest["base_url"],
                                 timeout=90, max_retries=1)
            try:
                interpreted = resolve_intent(text_client, manifest["instruction"],
                    history=manifest.get("dialogue_history", []), model=manifest["model"],
                    camera_frame=manifest.get("camera_frame"))
            finally:
                text_client.close()
            write_json(run_dir / "intent.json", interpreted)
            report["interaction_kind"] = interpreted["kind"]
            report["understanding"] = interpreted
            display.emit("理解", interpreted["summary"], intent_file="intent.json")
            for assumption in interpreted["assumptions"]:
                display.emit("约定", assumption)
            if interpreted["kind"] != "execute":
                message = interpreted["question"] if interpreted["kind"] == "clarify" else interpreted["reply"]
                display.emit("补充" if interpreted["kind"] == "clarify" else "回复", message)
                report["simulation_started"] = False
                return 0
        import torch
        if not torch.cuda.is_available():
            raise RuntimeError("PyTorch 未检测到可用 CUDA。")
        torch.cuda.set_device(0)
        from langchain_openai import ChatOpenAI
        from embodichain.agents.hierarchy import llm
        client = ChatOpenAI(
            model=manifest["model"], api_key=os.environ["OPENAI_API_KEY"],
            base_url=manifest["base_url"], temperature=0,
            use_responses_api=False, timeout=180, max_retries=0 if manifest.get("llm_recovery") else 1,
            extra_body={"thinking": {"type": "disabled"}},
        )
        llm.task_llm = llm.recovery_llm = llm.compile_llm = client
        import gymnasium
        from embodichain.lab.gym.utils.gym_utils import (
            add_env_launcher_args_to_parser, build_env_cfg_from_args,
        )
        from embodichain.lab.scripts.run_env import generate_and_execute_action_list
        parser = argparse.ArgumentParser()
        add_env_launcher_args_to_parser(parser)
        args = parser.parse_args([
            "--gym_config", manifest["gym_config"],
            "--device", "cuda:0", "--gpu_id", "0", "--num_envs", "1",
            "--filter_dataset_saving", "--filter_visual_rand",
        ])
        cfg, gym_config, action_config = build_env_cfg_from_args(args)
        cfg.seed = manifest.get("seed", 0)
        agent_config = json.loads(Path(manifest["agent_config"]).read_text(encoding="utf-8"))
        env = gymnasium.make(
            id=gym_config["id"], cfg=cfg, agent_config=agent_config,
            agent_config_path=manifest["agent_config"], task_name=manifest["task_name"],
            **action_config,
        )
        env.reset(seed=manifest.get("seed", 0))
        raw = env.unwrapped
        recorder = raw.event_manager.get_functor("record_camera")
        if not callable(getattr(recorder, "save_and_clear", None)):
            raise RuntimeError("当前录像接口不支持 save_and_clear。")
        report["seed"] = manifest.get("seed", 0)
        report["stable_scene"] = manifest.get("stable_scene", False)
        display.emit("场景", f"随机种子：{report['seed']}；固定初始姿态：{report['stable_scene']}。")
        report["initial_objects"] = object_poses(raw)
        save_frame(recorder, run_dir / "initial.png")
        display.emit("场景", "仿真已初始化，初始物体坐标已记录。", objects=report["initial_objects"])
        cache_graph = root / CACHE_DIR / manifest["task_name"] / "agent_task_graph.json"
        if manifest.get("disturbance_mode") is not None or manifest.get("drop_mode") is not None:
            display.emit("规划", "先预检名义动作及预测恢复路径，再由原运行时执行受控对照。", source="controlled_native_recovery_diagnostic")
        elif manifest.get("pick_carry", False):
            display.emit("规划", "使用受控 SDK 动作图，先检查完整轨迹，再记录每步实际物体坐标。", source="controlled_sdk_diagnostic")
        elif manifest.get("natural_language", False):
            display.emit("规划", "将根据已理解的目标选择机器人技能，并检查完整动作。", source="deepseek_intent_to_sdk_skills")
        elif cache_graph.is_file() and not manifest["regenerate"]:
            display.emit("规划", "本轮复用已有任务计划。", source="existing_plan")
        else:
            display.emit("规划", "将把本次指令交给 DeepSeek 生成任务计划。", source="new_api_plan")
        if manifest.get("plate_layout", False) or manifest.get("natural_language", False) or manifest.get("pick_carry", False) or manifest.get("disturbance_mode") is not None or manifest.get("drop_mode") is not None:
            planner_context = cpu_planner_inputs()
            planner_context.__enter__()
            report["plate_layout"] = manifest.get("plate_layout", False)
            report["planner_joint_inputs"] = "cpu"
            report["measured_gripper_sync_device"] = "match_sdk_open_state_threshold_v4"
        restore_display = display.attach(raw)
        report["simulation_started"] = True
        if manifest.get("drop_mode") is not None:
            from agentchord_console.drop_demo import prepare_bundle, DropObserver
            from agentchord_console.carry_demo import MotionTrace
            enabled=manifest["drop_mode"]=="on"
            bundle,check=prepare_bundle(raw,run_dir,enabled,write_json=write_json,
                model=manifest["model"],base_url=manifest["base_url"],progress=lambda message:display.emit("规划",message))
            write_json(run_dir/"plan_check.json",check)
            if bundle is None or check.get("status")!="passed":
                raise RuntimeError("掉落 demo 预检未通过；尚未执行。请传回诊断包。")
            task_source=run_dir/"agent_compiled_graph.drop.json"
            write_json(task_source,bundle)
            write_json(run_dir/"result.initial.json",report["initial_objects"])
            report.update(controlled_drop=True,recovery_enabled=enabled,llm_recovery_generated=enabled,
                drop_recovery_contract=bundle["metadata"]["drop_recovery_contract"],
                right_grip_policy=bundle["metadata"]["right_grip_policy"],
                requested_lift_m=bundle["metadata"]["requested_lift_m"])
            report["initial_robot_qpos"]=json_value(raw.robot.get_qpos())
            recovery_observer=DropObserver(raw,run_dir,enabled,write_json,snapshot=lambda name:save_frame(recorder,run_dir/name))
            motion_trace=MotionTrace(raw,run_dir/"object_motion.jsonl",after_step=recovery_observer.after_step)
            recovery_observer.trace=motion_trace;motion_trace.attach();recovery_observer.attach()
            actions=raw.compile_agent.act(task_source,env=raw,interactive_error_injection=False)
            valid=len(actions)>0
            if valid:motion_trace.settle()
            report.update(object_motion="object_motion.jsonl",recovery_events="recovery_events.json",
                disturbance_injected=recovery_observer.injected,native_monitor_detected=recovery_observer.detected,
                physical_drop_observed=recovery_observer.drop_observed)
            report["final_robot_qpos"]=json_value(raw.robot.get_qpos())
        elif manifest.get("disturbance_mode") is not None:
            from agentchord_console.disturbance_demo import prepare_bundle, RecoveryObserver
            from agentchord_console.carry_demo import MotionTrace
            enabled = manifest["disturbance_mode"] == "on"
            if manifest.get("llm_recovery"):
                from agentchord_console.llm_recovery import prepare_llm_bundle
                bundle, check = prepare_llm_bundle(raw, run_dir, write_json=write_json,
                    model=manifest["model"], base_url=manifest["base_url"],
                    progress=lambda message: display.emit("规划", message))
            else:
                bundle, check = prepare_bundle(raw, enabled, progress=lambda lift:
                    display.emit("检查", f"预检名义抬升 {lift * 100:.0f} 厘米；随后检查预测的恢复动作。"))
            write_json(run_dir / "plan_check.json", check)
            if bundle is None or check.get("status") != "passed":
                raise RuntimeError("名义动作或预测恢复预检未通过；尚未执行。详见 plan_check.json。")
            task_source = run_dir / "agent_compiled_graph.disturbance.json"
            write_json(task_source, bundle)
            report.update(controlled_disturbance=True, recovery_enabled=enabled,
                          requested_lift_m=bundle["metadata"]["requested_lift_m"],
                          llm_recovery_generated=bool(manifest.get("llm_recovery")),
                          llm_recovery_contract=bundle["metadata"].get("llm_recovery_contract"))
            report["initial_robot_qpos"] = json_value(raw.robot.get_qpos())
            recovery_observer = RecoveryObserver(raw, run_dir, enabled, write_json,
                snapshot=lambda name: save_frame(recorder, run_dir / name))
            motion_trace = MotionTrace(raw, run_dir / "object_motion.jsonl", after_step=recovery_observer.after_step)
            recovery_observer.trace = motion_trace
            motion_trace.attach()
            recovery_observer.attach()
            actions = raw.compile_agent.act(task_source, env=raw, interactive_error_injection=False)
            valid = len(actions) > 0
            if valid:
                motion_trace.settle()
            report.update(object_motion="object_motion.jsonl", recovery_events="recovery_events.json",
                          disturbance_injected=recovery_observer.injected,
                          native_monitor_detected=recovery_observer.detected)
            report["final_robot_qpos"] = json_value(raw.robot.get_qpos())
        elif manifest.get("pick_carry", False):
            from agentchord_console.carry_demo import select_bundle, MotionTrace
            bundle, carry_check = select_bundle(raw, progress=lambda lift:
                display.emit("检查", f"预检抬升 {lift * 100:.0f} 厘米的完整动作；暂不执行。"))
            write_json(run_dir / "plan_check.json", carry_check)
            if carry_check.get("grasp_check") is not None:
                write_json(run_dir / "grasp_check.json", carry_check["grasp_check"])
            if bundle is None or carry_check.get("status") != "passed":
                raise RuntimeError("受控搬运完整预检未通过；尚未执行。请传回证据包中的 plan_check.json。")
            task_source = run_dir / "agent_compiled_graph.carry.json"
            write_json(task_source, bundle)
            report["controlled_pick_carry"] = True
            report["requested_lift_m"] = carry_check["selected_lift_m"]
            motion_trace = MotionTrace(raw, run_dir / "object_motion.jsonl").attach()
            actions = raw.compile_agent.act(task_source, env=raw, interactive_error_injection=False)
            valid = len(actions) > 0
            if valid:
                motion_trace.settle()
            report["object_motion"] = "object_motion.jsonl"
        elif interpreted is not None:
            from agentchord_console.task_plan import select_task_layout
            def task_progress(index, total, details):
                display.emit("检查", f"检查动作候选 {index}/{total}；暂不执行物理动作。", details=details)
            bundle, task_check, task_changes = select_task_layout(interpreted, raw, progress=task_progress, camera_frame=manifest.get("camera_frame"))
            write_json(run_dir / "plan_check.json", task_check)
            if task_check.get("grasp_check") is not None:
                write_json(run_dir / "grasp_check.json", task_check["grasp_check"])
            if task_check["status"] == "clarify":
                question = task_check.get("question", task_check.get("reason", "请补充餐具的摆放位置。"))
                report["interaction_kind"] = "clarify"
                report["understanding"] = {**interpreted, "kind": "clarify", "placements": [], "question": question, "reply": ""}
                display.emit("补充", question)
                return 0
            if task_check["status"] != "passed":
                raise RuntimeError("你的要求已理解，但当前场景下的动作检查未通过。原因已保存在 plan_check.json；尚未执行物理动作。")
            task_source = run_dir / "agent_compiled_graph.language.json"
            write_json(task_source, bundle)
            write_json(run_dir / "task_plan.json", {"intent": interpreted, "details": task_changes})
            display.emit("检查", "完整动作生成检查通过，开始执行；实际夹持与摆放效果随后由视频核验。")
            actions = raw.compile_agent.act(task_source, env=raw, interactive_error_injection=False)
            valid = len(actions) > 0
        else:
            valid = generate_and_execute_action_list(
                env, 0, False, task_prompt=manifest["task_prompt"],
                regenerate=manifest["regenerate"], recovery=False,
                interactive_error_injection=False,
            )
        report["execution_completed"] = bool(valid)
        report["final_objects"] = object_poses(raw)
        try:
            report["original_scene_success"] = json_value(raw.is_task_success())
            report["official_success_tolerance_m"] = float(
                getattr(raw, "metadata", {}).get("success_params", {}).get("tolerance", 0.02)
            )
        except Exception as exc:
            report["metric_error"] = str(exc)
        display.emit("结果", "动作执行状态与原场景判定已记录。",
                     execution_completed=report["execution_completed"],
                     original_scene_success=report["original_scene_success"])
        save_frame(recorder, run_dir / "final.png")
        recorder.save_and_clear()
        report["videos"] = [str(p.relative_to(run_dir)) for p in sorted((run_dir / "videos").glob("*.mp4"))]
        cache = root / CACHE_DIR / manifest["task_name"]
        for name in ("agent_task_graph.json", "agent_compiled_graph.json"):
            if (cache / name).is_file():
                shutil.copy2(cache / name, run_dir / name)
        if not valid:
            raise RuntimeError("本次未生成有效动作序列。")
        if not report["videos"]:
            raise RuntimeError("动作已执行，但本轮没有导出视频，请检查 run.log。")
        display.emit("录像", "本轮录像和初末截图已保存。", videos=report["videos"])
        report["planning_trace"] = "planning_trace.json"
        if (run_dir / "agent_compiled_graph.used.json").is_file():
            report["executed_graph"] = "agent_compiled_graph.used.json"
        if (run_dir / "graph_correction.json").is_file():
            report["graph_correction"] = "graph_correction.json"
        if display.plan is not None:
            report["plan_summary"] = "plan_summary.json"
        write_json(run_dir / "result.json", report)
        print("\n动作执行完成，视频和结束截图已保存。", flush=True)
        if interpreted is None:
            print(f"原场景判定：{report['original_scene_success']}", flush=True)
        else:
            print("本轮要求的实际摆放效果，请结合录像和最终坐标核验。", flush=True)
        print(f"视频：{run_dir / report['videos'][0]}", flush=True)
        if manifest["hold"] and sys.stdin.isatty():
            print("当前窗口保留结束画面。可直接输入下一条要求，或按回车返回控制台。", flush=True)
            try:
                next_text = input().strip()
                if next_text:
                    report["next_instruction"] = next_text
            except EOFError:
                pass
        exit_code = 0
    except KeyboardInterrupt:
        report["error"] = "用户中断本轮仿真"
        display.emit("停止", report["error"])
        exit_code = 130
    except Exception as exc:
        report["error"] = f"{type(exc).__name__}: {exc}"
        display.emit("报错", report["error"])
        traceback.print_exc()
    finally:
        if recovery_observer is not None:
            recovery_observer.close()
            report.update(disturbance_injected=recovery_observer.injected,
                          native_monitor_detected=recovery_observer.detected)
            if manifest.get("drop_mode") is not None:
                report["physical_drop_observed"]=recovery_observer.drop_observed
        if motion_trace is not None:
            motion_trace.close()
        restore_display()
        if planner_context is not None:
            planner_context.__exit__(None, None, None)
        if (run_dir / "plan_check.json").is_file():
            report["plan_check"] = "plan_check.json"
        for filename, key in (("agent_compiled_graph.used.json", "executed_graph"),
                              ("graph_correction.json", "graph_correction"),
                              ("plan_summary.json", "plan_summary"),
                              ("planning_trace.json", "planning_trace")):
            if (run_dir / filename).is_file():
                report[key] = filename
        if raw is not None and "final_objects" not in report:
            try:
                report["final_objects"] = object_poses(raw)
            except Exception as exc:
                report["final_pose_error"] = str(exc)
        if recorder is not None and not (run_dir / "final.png").is_file():
            try:
                save_frame(recorder, run_dir / "final.png")
            except Exception as exc:
                report["final_frame_error"] = str(exc)
        if (run_dir / "grasp_check.json").is_file():
            report["grasp_check"] = "grasp_check.json"
        if (run_dir / "agent_compiled_graph.original.json").is_file():
            report["original_compiled_graph"] = "agent_compiled_graph.original.json"
        if recorder is not None:
            try:
                recorder.save_and_clear()
            except Exception as exc:
                report["video_flush_error"] = str(exc)
        report["videos"] = [str(p.relative_to(run_dir)) for p in sorted((run_dir / "videos").glob("*.mp4"))]
        write_json(run_dir / "result.json", report)
        if env is not None:
            try:
                env.close()
            except Exception as exc:
                print(f"关闭窗口时的提示：{exc}", flush=True)
    return exit_code


def open_video(run_dir: Path | None):
    if run_dir is None:
        print("请先执行一轮整理任务。")
        return
    videos = sorted((run_dir / "videos").glob("*.mp4"))
    if not videos:
        print(f"本轮暂无视频，请查看 {run_dir / 'run.log'}")
        return
    opener = shutil.which("xdg-open")
    if opener:
        subprocess.Popen([opener, str(videos[0])], stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL)
    print(f"视频文件：{videos[0]}")


def main():
    parser = argparse.ArgumentParser(description="输入餐具整理指令，运行 AgentChord 桌面仿真并保存视频。")
    parser.add_argument("--project", type=Path, default=Path(__file__).resolve().parent.parent)
    parser.add_argument("--save-api-key", action="store_true", help="隐藏输入并保存 DeepSeek 密钥，以后自动读取；配置后退出")
    parser.add_argument("--instruction", help="直接执行一条指令，然后退出")
    parser.add_argument("--no-hold", action="store_true", help="结束后直接关闭仿真窗口")
    parser.add_argument("--no-proxy", action="store_true", help="仅本轮仿真子进程忽略代理环境变量，直接访问 API")
    parser.add_argument("--resolution", nargs=2, type=int, default=[640, 480], metavar=("W", "H"))
    parser.add_argument("--seed", type=int, default=0, help="固定初始化与重置随机种子，默认 0")
    parser.add_argument("--stable-scene", action="store_true", help="关闭叉子、勺子和机械臂初始姿态随机化")
    parser.add_argument("--legacy-plan", action="store_true", help="使用旧版任务图生成流程；默认由 DeepSeek 理解自然语言并自动选择技能")
    parser.add_argument("--plate-layout", action="store_true", help="旧版布局选项；自然语言模式根据语义自动选布局")
    parser.add_argument("--skip-grasp-check", action="store_true", help="跳过首步抓取 IK 预检")
    parser.add_argument("--worker", type=Path, help=argparse.SUPPRESS)
    options = parser.parse_args()
    if options.worker:
        return simulation_worker(options.worker)
    if sys.platform != "linux":
        print("请将此脚本放到 2080 的 Ubuntu agentchord_pack 项目运行。")
        return 1
    if options.save_api_key:
        from agentchord_console.credentials import save_key
        try:
            with warnings.catch_warnings():
                warnings.simplefilter("error", getpass.GetPassWarning)
                key = getpass.getpass("输入 DeepSeek API 密钥（不显示，保存到本机用户配置）： ")
            path = save_key(key)
        except getpass.GetPassWarning:
            print("请在 Ubuntu 终端运行配置命令，以便隐藏输入密钥。")
            return 1
        except (EOFError, KeyboardInterrupt):
            print("\n未保存密钥。")
            return 1
        except (OSError, RuntimeError, UnicodeError) as exc:
            print(f"无法保存密钥：{exc}")
            return 1
        print(f"密钥已保存到：{path}")
        print("文件权限为 600。下次启动自动读取；更新密钥时再次运行此命令。")
        if os.environ.get("DEEPSEEK_API_KEY") or os.environ.get("OPENAI_API_KEY"):
            print("当前已设置密钥环境变量，运行时仍优先使用环境变量中的密钥。")
        return 0
    if any(n <= 0 for n in options.resolution):
        parser.error("视频宽高必须大于 0")
    try:
        root, _, _, _ = project_paths(options.project)
    except RuntimeError as exc:
        print(str(exc))
        return 1
    last_run = None
    dialogue_history = []
    pending_text = None
    print("AgentChord 桌面整理控制台")
    if not options.legacy_plan:
        print("自然语言模式：直接描述摆放要求，由 DeepSeek 理解目标。也可提问或补充上一条要求。")
        print("当前可搬运叉子和勺子，盘子作为位置参照。每次执行从初始场景开始新一轮。")
    elif options.plate_layout:
        print("当前模式：盘内双餐具演示。输入‘整理餐盘’、‘整理盘子’或‘整理桌子’，都会将叉子和勺子分开放入餐盘。")
    else:
        print("输入：整理桌子\n也可输入具体餐具整理要求；新要求由 DeepSeek 规划。")
    print("命令：重新规划 [要求]、视频、结果、退出。当前场景包含盘子、叉子和勺子。")
    while True:
        try:
            if options.instruction is not None:
                text = options.instruction
            elif pending_text is not None:
                text, pending_text = pending_text, None
                print(f"\n指令 > {text}")
            else:
                text = input("\n指令 > ").strip()
            if not text:
                if options.instruction is not None:
                    return 1
                continue
            command = normalize(text)
            if command in {"退出", "exit", "quit", "q"}:
                return 0
            if command in {"视频", "播放视频", "video"}:
                open_video(last_run)
                if options.instruction is not None:
                    return 0
                continue
            if command in {"结果", "results"}:
                print(last_run if last_run else "请先执行一轮任务。")
                if options.instruction is not None:
                    return 0
                continue
            force = text.startswith("重新规划")
            if force:
                text = text[len("重新规划"):].strip() or "整理桌子"
            current_run, code = run_command(root, text, force, options.resolution, not options.no_hold,
                                         no_proxy=options.no_proxy, seed=options.seed,
                                         stable_scene=options.stable_scene, grasp_check=not options.skip_grasp_check,
                                         plate_layout=options.plate_layout,
                                         natural_language=not options.legacy_plan, history=dialogue_history[-8:])
            current_result_path = current_run / "result.json"
            current_result = json.loads(current_result_path.read_text(encoding="utf-8")) if current_result_path.is_file() else {}
            if options.legacy_plan or current_result.get("videos"):
                last_run = current_run
            pending_text = current_result.get("next_instruction")
            if not options.legacy_plan and current_result.get("understanding"):
                dialogue_history.append({"role": "user", "content": text})
                dialogue_history.append({"role": "assistant", "content": json.dumps(current_result["understanding"], ensure_ascii=False)})
            if options.instruction is not None:
                return code
        except (EOFError, KeyboardInterrupt):
            print("\n已退出控制台。")
            return 0
        except Exception as exc:
            print(f"无法开始本轮：{exc}")
            if options.instruction is not None:
                return 1


if __name__ == "__main__":
    raise SystemExit(main())
