#!/usr/bin/env python3
"""Launch the next controlled physical demo and collect actual motion evidence."""
from __future__ import annotations

import argparse
import json
from pathlib import Path
import sys

sys.path.insert(0, str(Path(__file__).resolve().parent.parent))
from agentchord_console import console
from agentchord_console.carry_demo import assess_trace
from agentchord_console.validate_demo import assess_run, read_json, save_evidence


def assess_carry(run_dir):
    run_dir = Path(run_dir).resolve()
    report = assess_run(run_dir)
    manifest = read_json(run_dir / "run.json")
    result = read_json(run_dir / "result.json")
    cfg = read_json(run_dir / "gym_config.json")
    preflight = read_json(run_dir / "plan_check.json")
    bundle = read_json(run_dir / "agent_compiled_graph.used.json")
    source = read_json(run_dir / "agent_compiled_graph.carry.json")
    report.update(test="controlled_pick_lift_carry_release_return", llm_planning_test=False,
                  recovery=False, requested_lift_m=result.get("requested_lift_m"),
                  note="Additional controlled physical diagnostic. Motion thresholds are our checks, not official benchmark criteria; video review is still required.")
    correct_mode = (manifest.get("pick_carry") is True and manifest.get("natural_language") is False
                    and manifest.get("plate_layout") is False and manifest.get("recovery") is False
                    and manifest.get("task_prompt") == console.DEFAULT_GOAL and cfg.get("id") == "RearrangementAgent-v3")
    source_edges = source.get("task_graph", {}).get("edges", [])
    expected_ids = [f"e{i:02d}_{stage}" for i, stage in enumerate(("grasp", "orient", "lift", "carry", "lower", "release", "return"), 1)]
    graph_ok = (bundle == source and [e.get("id") for e in source_edges] == expected_ids
                and preflight.get("status") == "passed")
    report["checked_graph_executed"] = graph_ok
    trace = run_dir / "object_motion.jsonl"
    try:
        report["motion"] = assess_trace(trace)
        # Verify the recorded initial/final poses match the independently saved
        # scene snapshots, so a stale trace cannot validate another rollout.
        rows = [json.loads(line) for line in trace.read_text(encoding="utf-8").splitlines() if line.strip()]
        for obj in ("plate", "fork", "spoon"):
            for row, key in ((rows[0], "initial_objects"), (rows[-1], "final_objects")):
                if any(abs(row["objects_xyz_m"][obj][i] - result[key][obj]["xyz"][i]) > 1e-6 for i in range(3)):
                    raise ValueError("Trace and scene snapshots differ.")
    except (OSError, KeyError, TypeError, ValueError) as exc:
        report["motion_error"] = f"{type(exc).__name__}: {exc}"
        report["motion"] = None
    if not correct_mode:
        status = "wrong_task_or_mode"
    elif not report["execution_completed"] or report["process_exit_code"] != 0:
        status = "execution_incomplete"
    elif not graph_ok or report["motion"] is None:
        status = "motion_evidence_incomplete"
    elif report["official_function_success"] is not True or report["coordinate_check_passed"] is not True:
        status = "final_metric_failed"
    elif not report["motion"]["motion_checks_passed"]:
        status = "physical_motion_check_failed"
    elif not report["videos"] or not all((run_dir / name).is_file() for name in ("initial.png", "final.png")):
        status = "visual_evidence_incomplete"
    else:
        status = "measured_carry_passed_awaiting_video_review"
    report["status"] = status
    return report


def main():
    parser = argparse.ArgumentParser(description="下一轮：实际抓取、抬升、搬运、释放和退离测试")
    parser.add_argument("--project", type=Path, default=Path(__file__).resolve().parent.parent)
    parser.add_argument("--review", type=Path, help="只核验已有本轮目录，不启动仿真")
    args = parser.parse_args()
    try:
        if args.review:
            run_dir = args.review.expanduser().resolve()
            if not (run_dir / "run.json").is_file():
                raise RuntimeError("指定目录中没有 run.json。")
        else:
            if sys.platform != "linux":
                raise RuntimeError("请在 2080 的 Ubuntu 图形桌面终端运行。")
            print("下一轮：双臂抓取→竖直抬升→盘旁搬运→释放→返回→静置观察。")
            print("固定种子 0、GPU 0、单环境；不调用 API；恢复关闭。", flush=True)
            run_dir, _ = console.run_command(
                args.project, "整理桌子", False, [640, 480], False, no_proxy=True,
                seed=0, stable_scene=True, grasp_check=True, plate_layout=False,
                natural_language=False, pick_carry=True,
            )
        report = assess_carry(run_dir)
        evidence = save_evidence(run_dir, report)
        target = run_dir / "carry_validation_evidence.zip"
        evidence.replace(target)
        print("\n=== 抬升搬运验证 ===")
        print("动作完成：", report["execution_completed"])
        print("官方最终位置判定：", report["official_function_success"])
        for name, item in (report.get("motion") or {}).get("objects", {}).items():
            print(f"{name}：实际最大抬升 {item['peak_rise_m'] * 100:.2f} cm；腾空期间水平移动 {item['elevated_xy_travel_m'] * 100:.2f} cm；搬运检查 {item['measured_carry_check']}；退离后稳定 {item['stable_after_return_check']}")
        print("状态：", report["status"])
        print("这是新增物理诊断，视觉效果待核验；尚未测试扰动恢复。")
        print("请传回证据包：", target)
        return 0 if report["status"] == "measured_carry_passed_awaiting_video_review" else 2
    except (EOFError, KeyboardInterrupt):
        print("\n测试已停止。")
        return 130
    except Exception as exc:
        print(f"无法完成测试：{type(exc).__name__}: {exc}")
        return 1


if __name__ == "__main__":
    raise SystemExit(main())
