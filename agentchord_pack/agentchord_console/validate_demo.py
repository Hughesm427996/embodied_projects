#!/usr/bin/env python3
"""Run one nominal rollout and collect evidence for the pinned task criterion."""
from __future__ import annotations

import argparse
import json
import math
from pathlib import Path
import sys
import zipfile

sys.path.insert(0, str(Path(__file__).resolve().parent.parent))
from agentchord_console import console


def read_json(path):
    return json.loads(path.read_text(encoding="utf-8")) if path.is_file() else {}


def offsets(objects):
    values = {}
    for name in ("plate", "fork", "spoon"):
        xyz = objects[name]["xyz"]
        if not isinstance(xyz, list) or len(xyz) != 3 or any(
            isinstance(v, bool) or not isinstance(v, (float, int)) or not math.isfinite(v)
            for v in xyz
        ):
            raise ValueError("Missing or invalid object coordinates.")
        values[name] = xyz
    plate = values["plate"]
    return {name: {
        "relative_xyz_m": [values[name][i] - plate[i] for i in range(3)],
        "target_relative_y_m": target,
        "y_error_m": abs(values[name][1] - plate[1] - target),
    } for name, target in (("fork", .16), ("spoon", -.16))}


def assess_run(run_dir):
    run_dir = Path(run_dir).resolve()
    manifest = read_json(run_dir / "run.json") or read_json(run_dir / "validation_context.json")
    result = read_json(run_dir / "result.json")
    process = read_json(run_dir / "process_status.json")
    cfg = read_json(run_dir / "gym_config.json")
    videos = [str(p.relative_to(run_dir)) for p in sorted((run_dir / "videos").glob("*.mp4"))
              if not p.is_symlink() and p.is_file() and p.stat().st_size > 0
              and p.resolve().is_relative_to(run_dir)]
    report = {
        "run_dir": str(run_dir), "test": "nominal_plate_side_rearrangement",
        "execution_completed": result.get("execution_completed") is True,
        "process_exit_code": process.get("exit_code"),
        "official_function_success": result.get("original_scene_success"),
        "coordinate_check_passed": None,
        "seed": manifest.get("seed"), "stable_scene": manifest.get("stable_scene"),
        "model": manifest.get("model"), "task_name": manifest.get("task_name"),
        "reused_plan": manifest.get("reuse_existing_plan"),
        "recovery": manifest.get("recovery"), "videos": videos,
        "visual_task_success": None, "paper_reproduction_completed": False,
        "note": "A single nominal trial. Video must confirm grasp retention, placement and absence of falls; no disturbance/recovery result is established.",
    }
    correct_task = (manifest.get("natural_language") is False
                    and not manifest.get("pick_carry", False)
                    and manifest.get("disturbance_mode") is None
                    and manifest.get("plate_layout") is False
                    and manifest.get("recovery") is False
                    and manifest.get("task_prompt") == console.DEFAULT_GOAL
                    and cfg.get("id") == "RearrangementAgent-v3")
    tolerance = result.get("official_success_tolerance_m")
    valid_tolerance = (not isinstance(tolerance, bool) and isinstance(tolerance, (int, float))
                       and math.isfinite(tolerance) and tolerance > 0)
    if valid_tolerance:
        report["tolerance_m"] = tolerance
        try:
            report["initial"] = offsets(result["initial_objects"])
            report["final"] = offsets(result["final_objects"])
            report["coordinate_check_passed"] = all(
                item["y_error_m"] <= tolerance for item in report["final"].values()
            )
            report["initial_metric_already_passed"] = all(
                item["y_error_m"] <= tolerance for item in report["initial"].values()
            )
            report["displacement_xyz_m"] = {
                name: [result["final_objects"][name]["xyz"][i] - result["initial_objects"][name]["xyz"][i]
                       for i in range(3)] for name in ("fork", "spoon")
            }
            report["possible_drop_warning"] = any(
                delta[2] < -.10 for delta in report["displacement_xyz_m"].values()
            )
        except (KeyError, TypeError, ValueError):
            report["coordinate_error"] = "Object coordinates were missing or invalid."
    if not correct_task:
        report["status"] = "wrong_task_or_mode"
    elif process.get("exit_code") != 0 or not report["execution_completed"]:
        report["status"] = "execution_incomplete"
    elif not valid_tolerance or report["coordinate_check_passed"] is None or not isinstance(report["official_function_success"], bool):
        report["status"] = "metric_evidence_incomplete"
    elif report["official_function_success"] != report["coordinate_check_passed"]:
        report["status"] = "metric_disagreement"
    elif not report["coordinate_check_passed"]:
        report["status"] = "official_metric_failed"
    elif not videos or not all((run_dir / name).is_file() for name in ("initial.png", "final.png")):
        report["status"] = "visual_evidence_incomplete"
    else:
        report["status"] = "official_metric_passed_awaiting_video_review"
    return report


def save_evidence(run_dir, report):
    run_dir = Path(run_dir).resolve()
    console.write_json(run_dir / "validation_report.json", report)
    selected = ["validation_report.json", "result.json", "run.log", "initial.png", "final.png",
                "plan_summary.json", "grasp_check.json", "graph_correction.json",
                "agent_compiled_graph.used.json", "plan_check.json", "object_motion.jsonl",
                "agent_compiled_graph.carry.json", *report["videos"]]
    target = run_dir / "validation_evidence.zip"
    with zipfile.ZipFile(target, "w", compression=zipfile.ZIP_DEFLATED) as archive:
        for name in selected:
            source = run_dir / name
            if source.is_file() and not source.is_symlink() and source.resolve().is_relative_to(run_dir):
                archive.write(source, name)
    return target


def main():
    parser = argparse.ArgumentParser(description="验证一轮官方盘旁整理指标并打包录像。")
    parser.add_argument("--project", type=Path, default=Path(__file__).resolve().parent.parent)
    parser.add_argument("--review", type=Path, help="仅检查已有控制台实验目录，不启动仿真")
    parser.add_argument("--seed", type=int, default=0)
    parser.add_argument("--randomized", action="store_true", help="保留原配置初始姿态随机化；默认固定场景")
    args = parser.parse_args()
    try:
        if args.review:
            run_dir = args.review.expanduser().resolve()
            if not (run_dir / "run.json").is_file():
                raise RuntimeError("请指定包含 run.json 的控制台实验目录。")
        else:
            if sys.platform != "linux":
                raise RuntimeError("请在 2080 的 Ubuntu 图形桌面终端运行。")
            print("验证：官方盘旁整理；不注入扰动；恢复关闭。", flush=True)
            print(f"种子 {args.seed}；固定初始场景 {not args.randomized}。", flush=True)
            run_dir, code = console.run_command(
                args.project, "整理桌子", False, [640, 480], False, no_proxy=True,
                seed=args.seed, stable_scene=not args.randomized,
                grasp_check=True, plate_layout=False, natural_language=False,
            )
        report = assess_run(run_dir)
        evidence = save_evidence(run_dir, report)
        print("\n=== 验证结果 ===")
        print("动作完成：", report["execution_completed"])
        print("官方函数判定：", report["official_function_success"])
        print("最终坐标复核：", report["coordinate_check_passed"])
        for name, values in report.get("final", {}).items():
            print(f"{name}：Y 误差 {values['y_error_m'] * 100:.3f} 厘米")
        print("状态：", report["status"])
        if report.get("initial_metric_already_passed"):
            print("注意：初始位置已符合官方指标，需录像核对实际操作。")
        if report.get("possible_drop_warning"):
            print("注意：餐具高度明显下降，需要录像检查是否掉落。")
        print("视觉成功仍待录像核验；本轮不能作为论文成功率或恢复结果。")
        print("报告：", run_dir / "validation_report.json")
        print("请传回证据包：", evidence)
        return 0 if report["status"] == "official_metric_passed_awaiting_video_review" else 2
    except (EOFError, KeyboardInterrupt):
        print("\n验证已停止。")
        return 130
    except Exception as exc:
        print(f"无法完成验证：{exc}")
        return 1


if __name__ == "__main__":
    raise SystemExit(main())
