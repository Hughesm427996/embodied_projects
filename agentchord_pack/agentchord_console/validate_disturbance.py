#!/usr/bin/env python3
"""Run the matched off/on controlled displacement pair and collect evidence."""
from __future__ import annotations

import argparse
import copy
import datetime as dt
import json
import math
from pathlib import Path
import sys
import zipfile

sys.path.insert(0, str(Path(__file__).resolve().parent.parent))
from agentchord_console import console
from agentchord_console.carry_demo import assess_rows
from agentchord_console.disturbance_demo import EDGE, EDGE_STEP, SHIFT, recovery_edges, assess_injection
from agentchord_console.task_plan import _hash
from agentchord_console.validate_demo import assess_run, read_json


def _close(a, b, tolerance):
    if isinstance(a, bool) or isinstance(b, bool): return a is b
    if isinstance(a, (int, float)) and isinstance(b, (int, float)):
        return math.isfinite(a) and math.isfinite(b) and abs(a-b) <= tolerance
    if isinstance(a, list) and isinstance(b, list):
        return len(a) == len(b) and all(_close(x, y, tolerance) for x, y in zip(a, b))
    if isinstance(a, dict) and isinstance(b, dict):
        return a.keys() == b.keys() and all(_close(a[k], b[k], tolerance) for k in a)
    return type(a) is type(b) and a == b


def _scene_hash(cfg):
    value = copy.deepcopy(cfg)
    value.get("env", {}).get("events", {}).get("record_camera", {}).get("params", {}).pop("save_path", None)
    return _hash(value)


def _poses_match(row, objects):
    return all(_close(row["objects_xyz_m"][name], objects[name]["xyz"], 1e-6)
               for name in ("plate", "fork", "spoon"))


def _robot_return(initial, final):
    def flatten(value):
        if isinstance(value, list) and len(value) == 1 and isinstance(value[0], list): value = value[0]
        if not isinstance(value, list) or len(value) != 16 or not all(
            not isinstance(v, bool) and isinstance(v, (int, float)) and math.isfinite(v) for v in value):
            raise ValueError("Expected 16 measured robot joint positions.")
        return value
    a, b = flatten(initial), flatten(final)
    arm_error = max(abs(a[i]-b[i]) for i in range(12))
    gripper_error = max(abs(a[i]-b[i]) for i in range(12,16))
    return {"max_arm_joint_change_rad": arm_error, "max_gripper_joint_change_m": gripper_error,
            "arm_tolerance_rad": .01, "gripper_tolerance_m": .002,
            "passed": arm_error <= .01 and gripper_error <= .002}


def assess_trial(run_dir, *, allow_llm=False):
    run_dir = Path(run_dir).resolve()
    report = assess_run(run_dir)
    manifest = read_json(run_dir / "run.json") or read_json(run_dir / "validation_context.json")
    result = read_json(run_dir / "result.json")
    cfg = read_json(run_dir / "gym_config.json")
    bundle = read_json(run_dir / "agent_compiled_graph.used.json")
    source = read_json(run_dir / "agent_compiled_graph.disturbance.json")
    plan = read_json(run_dir / "plan_check.json")
    events = read_json(run_dir / "recovery_events.json").get("events", [])
    mode = manifest.get("disturbance_mode")
    llm = bool(allow_llm and mode == "on" and manifest.get("llm_recovery") is True)
    injections = [e for e in events if e.get("event") == "injection"]
    detections = [e for e in events if e.get("event") == "detection"]
    starts = [e for e in events if e.get("event") == "recovery_edge_start"]
    ends = [e for e in events if e.get("event") == "graph_edge_end"]
    report.update(test="controlled_native_displacement_recovery", mode=mode,
                  llm_planning_test=llm, recovery_agent_generation_test=llm,
                  injection_count=len(injections), detection_count=len(detections),
                  recovery_edges_started=[e.get("edge") for e in starts],
                  recovery_edges_completed=[e.get("edge") for e in ends if str(e.get("edge", "")).startswith("r") and e.get("monitor_index") is None],
                  scene_config_hash=_scene_hash(cfg),
                  note="Locally authored explicit recovery graph, native position monitor and graph switching. A single matched pair is not a success-rate or full-paper reproduction.")
    correct_mode = (mode in ("off", "on") and manifest.get("recovery") is (mode == "on")
                    and manifest.get("natural_language") is False and manifest.get("plate_layout") is False
                    and not manifest.get("pick_carry") and manifest.get("task_prompt") == console.DEFAULT_GOAL
                    and cfg.get("id") == "RearrangementAgent-v3")
    correct_mode = correct_mode and (llm if allow_llm and mode == "on" else not manifest.get("llm_recovery"))
    provenance_ok = True
    if llm:
        from agentchord_console.validate_llm_recovery import assess_generation
        report["generation_checks"] = assess_generation(run_dir, source)
        provenance_ok = report["generation_checks"]["passed"] and result.get("llm_recovery_generated") is True
        report["note"] = "Official RecoveryAgent API generation before execution, native CompileAgent and runtime; controlled nominal graph and one fork displacement. No runtime LLM replanning or rate estimate."
    report["checked_graph_executed"] = bundle == source and bool(source) and plan.get("status") == "passed"
    injection_ok = False
    if len(injections) == 1:
        e = injections[0]
        report["injection"] = e
        report["injection_checks"] = assess_injection(e)
        injection_ok = report["injection_checks"]["valid"]
    report["valid_injection"] = injection_ok
    native_detection = (len(detections) == 1 and len(injections) == 1
                        and detections[0].get("fn") == "monitor_object_moved"
                        and detections[0].get("result") is True
                        and detections[0].get("edge") == EDGE
                        and detections[0].get("step") == injections[0].get("step"))
    report["native_detection_verified"] = native_detection
    rows = None
    try:
        rows = [json.loads(line) for line in (run_dir / "object_motion.jsonl").read_text(encoding="utf-8").splitlines() if line.strip()]
        report["whole_motion"] = assess_rows(rows)
        if not _poses_match(rows[0], result["initial_objects"]) or not _poses_match(rows[-1], result["final_objects"]):
            raise ValueError("Trace and scene snapshots disagree.")
        if mode == "on" and native_detection:
            step = detections[0]["step"]
            part = copy.deepcopy(rows[step:])
            for index, row in enumerate(part): row["step"] = index
            part[0]["phase"] = "initial"
            report["post_detection_motion"] = assess_rows(part)
            if llm:
                metrics = report["post_detection_motion"]["objects"]
                report["post_detection_motion"].update(
                    motion_checks_passed=metrics["fork"]["measured_carry_check"] and all(
                        m["final_pose_check"] and m["stable_after_return_check"] for m in metrics.values()),
                    required_carried_objects=["fork"],
                    scope="Displaced fork must be carried; unchanged spoon may remain stationary, but both must finish correctly and stably.")
    except (OSError, KeyError, ValueError, TypeError, IndexError) as exc:
        report["motion_error"] = f"{type(exc).__name__}: {exc}"
        rows = None
    recovery_ok = not starts if mode == "off" else False
    if mode == "on" and source.get("task_graph"):
        expected = source.get("recovery_graph", {}).get("recovery_edges", []) if llm else recovery_edges(source)[1]
        expected_ids = [e["id"] for e in expected]
        recovery_ok = (report["recovery_edges_started"] == expected_ids
                       and report["recovery_edges_completed"] == expected_ids
                       and any(e.get("edge") == EDGE and e.get("monitor_index") == 0 for e in ends))
    report["runtime_branch_verified"] = recovery_ok
    try:
        report["measured_robot_return"] = _robot_return(result.get("initial_robot_qpos"), result.get("final_robot_qpos"))
    except (TypeError, ValueError):
        report["measured_robot_return"] = {"passed": False, "error": "Measured robot joint positions missing or invalid."}
    evidence_ok = bool(report["videos"]) and all((run_dir / name).is_file() for name in
        ("initial.png", "final.png", "before_disturbance.png", "after_disturbance.png"))
    report["trial_evidence_valid"] = (correct_mode and report["execution_completed"]
        and report["process_exit_code"] == 0 and report["checked_graph_executed"]
        and injection_ok and native_detection and recovery_ok and provenance_ok and rows is not None and evidence_ok
        and report["measured_robot_return"]["passed"]
        and isinstance(report["official_function_success"], bool)
        and report["coordinate_check_passed"] == report["official_function_success"])
    if not report["trial_evidence_valid"]:
        status = "trial_incomplete_or_invalid"
    elif mode == "off":
        status = "off_group_completed_goal_met" if report["coordinate_check_passed"] else "off_group_completed_goal_unmet"
    elif not report["coordinate_check_passed"]:
        status = "recovery_completed_final_goal_unmet"
    elif not report.get("post_detection_motion", {}).get("motion_checks_passed"):
        status = "recovery_motion_checks_failed"
    else:
        status = "native_recovery_passed_awaiting_video_review"
    report["status"] = status
    return report


def assess_pair(run_dirs, *, allow_llm=False):
    trials = {mode: assess_trial(path, allow_llm=allow_llm) for mode, path in run_dirs.items()}
    report = {"test": "controlled_displacement_off_on_pair", "trials": trials,
              "visual_pair_success": None, "paper_reproduction_completed": False,
              "status": "pair_incomplete", "matched_conditions": False,
              "scope": "One locally authored recovery graph; native SDK monitor, graph runtime, displacement injection, and physical skills. No LLM recovery generation or rate estimate."}
    if allow_llm:
        report.update(test="native_llm_recovery_off_on_pair",
            scope="One official RecoveryAgent API generation test before execution with native CompileAgent and runtime. Controlled nominal graph and known one-time fork displacement; no online LLM replanning, rate estimate or full simulation reproduction.")
    if set(trials) != {"off", "on"}: return report
    a, b = trials["off"], trials["on"]
    ar, br = (read_json(Path(run_dirs[k]) / "result.json") for k in ("off", "on"))
    ag, bg = (read_json(Path(run_dirs[k]) / "agent_compiled_graph.used.json").get("task_graph", {}) for k in ("off", "on"))
    ag, bg = copy.deepcopy(ag), copy.deepcopy(bg)
    ag.pop("task", None); bg.pop("task", None)
    initial_a = {k: v["xyz"] for k, v in ar.get("initial_objects", {}).items()}
    initial_b = {k: v["xyz"] for k, v in br.get("initial_objects", {}).items()}
    conditions = {"same_scene_config": a["scene_config_hash"] == b["scene_config_hash"],
                  "same_seed_and_fixed_scene": a.get("seed") == b.get("seed") == 0 and a.get("stable_scene") is True and b.get("stable_scene") is True,
                  "initial_objects_within_10_micrometres": bool(initial_a) and _close(initial_a, initial_b, 1e-5),
                  "initial_joints_within_0_0001_radians": bool(ar.get("initial_robot_qpos")) and _close(ar.get("initial_robot_qpos"), br.get("initial_robot_qpos"), 1e-4),
                  "same_nominal_actions_within_10_micrometres": bool(ag) and _close(ag, bg, 1e-5),
                  "same_injection_step": a.get("injection", {}).get("step") == b.get("injection", {}).get("step") and a.get("valid_injection") and b.get("valid_injection"),
                  "same_injection_protocol": a.get("injection_checks", {}).get("protocol") == b.get("injection_checks", {}).get("protocol"),
                  "measured_displacements_within_3_mm": bool(a.get("valid_injection") and b.get("valid_injection")) and math.dist(a["injection"]["measured_xyz_m"], b["injection"]["measured_xyz_m"]) <= .003,
                  "pre_injection_objects_within_3_mm": _close(a.get("injection", {}).get("before_objects"), b.get("injection", {}).get("before_objects"), .003)}
    report["conditions"] = conditions
    report["matched_conditions"] = all(conditions.values())
    if not all(t["trial_evidence_valid"] for t in trials.values()):
        report["status"] = "pair_incomplete_or_invalid"
    elif not report["matched_conditions"]:
        report["status"] = "conditions_not_matched"
    elif b["status"] != "native_recovery_passed_awaiting_video_review":
        report["status"] = "recovery_not_validated"
    elif a["coordinate_check_passed"]:
        report["status"] = "both_goals_met_no_recovery_benefit_established"
    else:
        report["status"] = "matched_recovery_benefit_awaiting_video_review"
    return report


def save_pair(pair_dir, run_dirs, report):
    pair_dir = Path(pair_dir).resolve()
    console.write_json(pair_dir / "pair_report.json", report)
    llm = report.get("test") == "native_llm_recovery_off_on_pair"
    target = pair_dir / ("llm_recovery_validation_evidence.zip" if llm else "disturbance_validation_evidence.zip")
    names = ["result.json", "run.log", "initial.png", "final.png", "before_disturbance.png",
             "after_disturbance.png", "plan_summary.json", "plan_check.json", "object_motion.jsonl",
             "agent_compiled_graph.used.json", "agent_compiled_graph.disturbance.json",
             "recovery_events.json", "monitor_checks.json", "process_status.json"]
    if llm:
        names += ["llm_recovery_generation.json", "llm_recovery_calls.json", "agent_recovery_spec.json",
                  "agent_compiled_graph.sdk.json", "agent_recovery_spec.raw_attempt_1.json",
                  "agent_recovery_spec.raw_attempt_2.json"]
    with zipfile.ZipFile(target, "w", compression=zipfile.ZIP_DEFLATED) as archive:
        archive.write(pair_dir / "pair_report.json", "pair_report.json")
        for mode, directory in run_dirs.items():
            directory = Path(directory).resolve()
            manifest = read_json(directory / "run.json")
            context = {k: manifest.get(k) for k in ("instruction", "task_prompt", "disturbance_mode", "recovery", "pick_carry", "llm_recovery",
                "natural_language", "plate_layout", "seed", "stable_scene", "device", "gpu_id", "num_envs")}
            console.write_json(directory / "validation_context.json", context)
            for name in [*names, "validation_context.json", "gym_config.json", *report["trials"][mode]["videos"]]:
                source = directory / name
                if source.is_file() and not source.is_symlink() and source.resolve().is_relative_to(directory):
                    archive.write(source, f"recovery_{mode}/{name}")
    return target


def main():
    parser = argparse.ArgumentParser(description="自动跑一组受控物体移位的恢复关闭／开启对照")
    parser.add_argument("--project", type=Path, default=Path(__file__).resolve().parent.parent)
    parser.add_argument("--review", type=Path, help="只核验已有 pair.json 目录")
    args = parser.parse_args()
    run_dirs = {}
    pair_dir = None
    try:
        if args.review:
            pair_dir = args.review.expanduser().resolve()
            runs = read_json(pair_dir / "pair.json").get("runs", {})
            if not runs: raise RuntimeError("该目录没有 pair.json 实验索引。")
            run_dirs = {k: Path(v) for k, v in runs.items()}
        else:
            if sys.platform != "linux": raise RuntimeError("请在 2080 的 Ubuntu 图形终端运行。")
            root, _, _, _ = console.project_paths(args.project)
            pair_dir = root / "outputs/disturbance_validation" / dt.datetime.now().strftime("%Y%m%d_%H%M%S_%f")
            pair_dir.mkdir(parents=True, exist_ok=False)
            print("自动对照：恢复关闭，再恢复开启；种子 0、单环境、GPU 0，不调用 API。")
            print(f"扰动：名义整理完成、原返回边第 {EDGE_STEP} 步，提交叉子世界 +Y 6 cm 位移；独立检查物理更新后的真实位移。", flush=True)
            for mode in ("off", "on"):
                print(f"\n=== 恢复 {mode} ===", flush=True)
                run, code = console.run_command(root, "整理桌子", False, [640, 480], False,
                    no_proxy=True, seed=0, stable_scene=True, grasp_check=True,
                    natural_language=False, plate_layout=False, disturbance_mode=mode)
                run_dirs[mode] = run
                console.write_json(pair_dir / "pair.json", {"runs": {k: str(v) for k, v in run_dirs.items()}})
                trial = assess_trial(run)
                if code != 0 or not trial["trial_evidence_valid"]:
                    print("本组未形成有效证据，停止后续组并保留诊断。", flush=True)
                    break
        report = assess_pair(run_dirs)
        target = save_pair(pair_dir, run_dirs, report)
        print("\n=== 扰动恢复对照 ===")
        for mode, trial in report["trials"].items():
            print(f"{mode}：动作完成 {trial['execution_completed']}；检测 {trial['native_detection_verified']}；官方最终判定 {trial['official_function_success']}；恢复边完成 {len(trial['recovery_edges_completed'])}；状态 {trial['status']}")
        print("条件匹配：", report["matched_conditions"])
        print("对照状态：", report["status"])
        print("录像核验尚未完成；这是一组受控接口测试，不是论文成功率。")
        print("请传回证据包：", target)
        return 0 if report["status"] == "matched_recovery_benefit_awaiting_video_review" else 2
    except (EOFError, KeyboardInterrupt):
        print("\n对照已停止。")
        return 130
    except Exception as exc:
        print(f"无法完成对照：{type(exc).__name__}: {exc}")
        if pair_dir is not None and run_dirs:
            try:
                report = assess_pair(run_dirs)
                report["runner_error"] = f"{type(exc).__name__}: {exc}"
                print("诊断包：", save_pair(pair_dir, run_dirs, report))
            except Exception:
                print("已产生的实验目录：", {k: str(v) for k, v in run_dirs.items()})
        return 1


if __name__ == "__main__": raise SystemExit(main())
