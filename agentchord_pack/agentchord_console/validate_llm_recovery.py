#!/usr/bin/env python3
"""Matched off/on validation for native API-generated preplanned recovery."""
from __future__ import annotations
import argparse
import copy
import datetime as dt
import json
from pathlib import Path
import sys

sys.path.insert(0, str(Path(__file__).resolve().parent.parent))
from agentchord_console import console
from agentchord_console.llm_recovery import MAX_CALLS, TERMINAL_CONTRACTS, validate_spec, validate_compiled
from agentchord_console.disturbance_demo import assess_predicted_robot_return, EDGE
from types import SimpleNamespace
from agentchord_console.task_plan import _hash
from agentchord_console.validate_demo import read_json
from agentchord_console.validate_disturbance import assess_trial, assess_pair, save_pair


def _response_json(text):
    text = text.strip()
    if text.startswith("```json\n") and text.endswith("```"):
        text = text[8:-3].strip()
    elif text.startswith("```\n") and text.endswith("```"):
        text = text[4:-3].strip()
    return json.loads(text)


class _EvidenceNormalization:
    """Independently reconstruct defaults for the restricted authoring forms.

    This is only an evidence cross-check, never the execution compiler.
    Native CompileAgent and graph_spec still create and execute the graph.
    """
    @staticmethod
    def normalize_recovery_spec(task, raw):
        canonical = copy.deepcopy(raw)
        for step in canonical["recovery_bindings"][0]["recovery"]:
            if step["type"] == "action":
                step.setdefault("left_arm_action", None)
                step.setdefault("right_arm_action", None)
                step.pop("force_valid", None)
            elif step["type"] in {"regrasp", "regrasp_both"}:
                step.setdefault("pre_grasp_dis", .1)
                step.setdefault("force_valid", False)
        return canonical, []


def _expected_actions(step, nominal_edges):
    kind = step["type"]
    if kind == "action":
        return {f"{arm}_action": step.get(f"{arm}_action") for arm in ("left_arm", "right_arm")}
    if kind == "replay_edge":
        result = {f"{arm}_action": copy.deepcopy(nominal_edges[step["edge_id"]].get(f"{arm}_action"))
                  for arm in ("left_arm", "right_arm")}
        if "force_valid" in step:
            for action in result.values():
                if action is not None: action["kwargs"]["force_valid"] = step["force_valid"]
        return result
    arms = step["arms"] if kind == "regrasp_both" else {step["robot_name"]: step["obj_name"]}
    return {f"{arm}_action": ({"fn":"grasp", "kwargs":{"robot_name":arm,
        "obj_name":arms[arm], "pre_grasp_dis":step["pre_grasp_dis"], "force_valid":step["force_valid"]}}
        if arm in arms else None) for arm in ("left_arm", "right_arm")}


def assess_generation(run_dir, used, *, binding_edge=EDGE):
    """Verify public API output, native compiler artifact and executed graph agree."""
    root = Path(run_dir)
    checks = {}
    try:
        report = read_json(root/"llm_recovery_generation.json")
        ledger = read_json(root/"llm_recovery_calls.json")
        sdk = read_json(root/"agent_compiled_graph.sdk.json")
        canonical_file = read_json(root/"agent_recovery_spec.json")
        index = report.get("selected_attempt")
        if type(index) is not int or index not in (1,2): raise ValueError("Missing selected model attempt.")
        raw = read_json(root/f"agent_recovery_spec.raw_attempt_{index}.json")
        calls = ledger.get("calls", [])
        generation_calls = [call for call in calls if call.get("stage") == "RecoveryAgent"]
        selected = generation_calls[index-1]
        checks["bounded_returned_api_calls"] = (ledger.get("max_invocations") == MAX_CALLS
            and 1 <= len(calls) <= MAX_CALLS and len(generation_calls) == index
            and report.get("api_invocations") == len(calls)
            and all(call.get("status") == "returned" and call.get("index") == i+1
                and call.get("request_hash") == _hash(call.get("messages"))
                and call.get("response_hash") == _hash(call.get("content")) for i,call in enumerate(calls)))
        checks["selected_api_output_matches_raw_spec"] = _response_json(selected["content"]) == raw
        checks["native_prompt_context_present"] = bool(selected.get("messages")) and any(
            "lightweight recovery" in str(m.get("content", "")) and binding_edge in str(m.get("content", ""))
            for m in selected["messages"])
        checks["model_and_endpoint_recorded"] = bool(report.get("model")) and bool(report.get("base_url")) and all(
            c.get("model") == report["model"] and c.get("base_url") == report["base_url"] for c in calls)
        checks["generation_before_execution_no_fallback"] = (report.get("status") == "passed"
            and report.get("local_recovery_fallback") is False and report.get("generation_phase") == "before_execution"
            and used.get("metadata", {}).get("llm_recovery_generated") is True
            and used["metadata"].get("generation_phase") == "before_execution")
        canonical = validate_spec(raw, used, _EvidenceNormalization,binding_edge=binding_edge)
        terminal_contract = used.get("metadata",{}).get("llm_recovery_contract")
        edges = validate_compiled(used, used,terminal_contract=terminal_contract,binding_edge=binding_edge)
        checks["raw_to_canonical_agree"] = canonical == canonical_file == sdk.get("recovery_spec") == used.get("recovery_spec")
        checks["native_artifact_is_used"] = (sdk.get("task_graph") == used.get("task_graph")
            and sdk.get("recovery_graph") == used.get("recovery_graph"))
        hashes = sdk.get("metadata", {})
        checks["native_hashes_agree"] = (hashes.get("schema_version") == "recovery_bindings_v1"
            and hashes.get("recovery_enabled") is True
            and hashes.get("task_graph_hash") == _hash(used["task_graph"])
            and hashes.get("raw_recovery_spec_hash") == report.get("raw_spec_hash") == _hash(raw)
            and hashes.get("recovery_spec_hash") == report.get("canonical_spec_hash") == _hash(canonical)
            and report.get("compiled_recovery_hash") == _hash(used["recovery_graph"]))
        binding = canonical["recovery_bindings"][0]
        nominal = {e["id"]:e for e in used["task_graph"]["edges"]}
        checks["model_steps_match_compiled_actions"] = len(edges) == len(binding["recovery"]) and all(
            _expected_actions(step,nominal) == {f"{arm}_action": edge.get(f"{arm}_action")
                                               for arm in ("left_arm", "right_arm")}
            for step,edge in zip(binding["recovery"],edges))
        checks["compiled_branch_path_and_merge"] = (edges[0]["source"] == nominal[binding["edge_id"]]["source"]
            and edges[-1]["target"] == nominal[binding["edge_id"]][binding["merge"]]
            and all(a["target"] == b["source"] for a,b in zip(edges,edges[1:])))
        attempts = report.get("attempts", [])
        checks["selected_prediction_passed"] = (len(attempts) == index and attempts[-1].get("status") == "passed"
            and attempts[-1].get("recovery_prediction", {}).get("status") == "passed"
            and report.get("recovery_prediction") == attempts[-1]["recovery_prediction"]
            and [e["id"] for e in report["recovery_prediction"]["attempts"][-1]["edges"]] == [e["id"] for e in edges])
        if terminal_contract is not None:
            checks["dual_arm_terminal_contract"] = (terminal_contract == report.get("contract_version")
                and terminal_contract in TERMINAL_CONTRACTS
                and report["recovery_prediction"].get("terminal_return_required") is True)
            terminal = report["recovery_prediction"]["attempts"][-1]["predicted_robot_return"]
            state,original = SimpleNamespace(),SimpleNamespace(open_state=terminal["arms"]["left"]["open_gripper_target"])
            for side in ("left","right"):
                arm = terminal["arms"][side]
                setattr(state,f"{side}_arm_current_qpos",arm["predicted_qpos"])
                setattr(state,f"{side}_arm_current_gripper_state",arm["predicted_gripper_state"])
                setattr(original,f"{side}_arm_init_qpos",arm["initial_qpos_target"])
            calculated = assess_predicted_robot_return(state,original)
            checks["dual_arm_terminal_prediction_verified"] = terminal == calculated and calculated["passed"] is True
        return {"passed":all(checks.values()), "checks":checks, "selected_attempt":index,
                "api_invocations":len(calls), "model":report.get("model"),
                "scope":"Public client invocation and artifact provenance, not independent server attestation."}
    except (OSError, ValueError, KeyError, TypeError, IndexError, AttributeError) as exc:
        return {"passed":False, "checks":checks, "error":f"{type(exc).__name__}: {exc}"}


def main():
    parser = argparse.ArgumentParser(description="官方 RecoveryAgent 调用 DeepSeek 的受控恢复对照")
    parser.add_argument("--project", type=Path, default=Path(__file__).resolve().parent.parent)
    parser.add_argument("--review", type=Path, help="只核验已有 pair.json 目录")
    args = parser.parse_args()
    runs, pair_dir = {}, None
    try:
        if args.review:
            pair_dir = args.review.expanduser().resolve()
            runs = {k:Path(v) for k,v in read_json(pair_dir/"pair.json").get("runs", {}).items()}
            if not runs: raise RuntimeError("该目录没有 pair.json 实验索引。")
        else:
            if sys.platform != "linux": raise RuntimeError("请在 2080 的 Ubuntu 图形终端运行。")
            root, _, _, _ = console.project_paths(args.project)
            pair_dir = root/"outputs/llm_recovery_validation"/dt.datetime.now().strftime("%Y%m%d_%H%M%S_%f")
            pair_dir.mkdir(parents=True, exist_ok=False)
            print("先跑恢复关闭对照，再由官方 RecoveryAgent 调用 DeepSeek 生成开启组恢复方案。")
            print("方案在执行前生成；固定种子 0、单环境、GPU 0、相同名义动作和叉子扰动。")
            print("最多两次 API 请求；生成或预检失败则停止。自动读取已保存的密钥。", flush=True)
            for mode in ("off", "on"):
                print(f"\n=== 恢复 {mode} ===", flush=True)
                run, code = console.run_command(root, "整理桌子", False, [640,480], False,
                    no_proxy=True, seed=0, stable_scene=True, grasp_check=True,
                    natural_language=False, plate_layout=False, disturbance_mode=mode, llm_recovery=mode == "on")
                runs[mode] = run
                console.write_json(pair_dir/"pair.json", {"runs":{k:str(v) for k,v in runs.items()}})
                trial = assess_trial(run, allow_llm=True)
                if code != 0 or not trial["trial_evidence_valid"]:
                    print("本组未形成有效证据，停止并保留诊断。", flush=True)
                    break
        report = assess_pair(runs, allow_llm=True)
        target = save_pair(pair_dir,runs,report)
        print("\n=== DeepSeek 恢复对照 ===")
        for mode,trial in report["trials"].items():
            print(f"{mode}：动作完成 {trial['execution_completed']}；检测 {trial['native_detection_verified']}；官方最终判定 {trial['official_function_success']}；状态 {trial['status']}")
            if mode == "on": print("API/编译来源核对：",trial.get("generation_checks", {}).get("passed",False))
        print("条件匹配：",report["matched_conditions"])
        print("对照状态：",report["status"])
        print("成功状态仍需录像核验；本轮不是实时 LLM 重规划，也不是论文成功率。")
        print("请传回证据包：",target)
        return 0 if report["status"] == "matched_recovery_benefit_awaiting_video_review" else 2
    except (EOFError,KeyboardInterrupt):
        print("\n对照已停止。")
        return 130
    except Exception as exc:
        print(f"无法完成对照：{type(exc).__name__}: {exc}")
        if pair_dir is not None and runs:
            try:
                report = assess_pair(runs,allow_llm=True)
                report["runner_error"] = f"{type(exc).__name__}: {exc}"
                print("诊断包：",save_pair(pair_dir,runs,report))
            except Exception:
                print("已产生的实验目录：",{k:str(v) for k,v in runs.items()})
        return 1


if __name__ == "__main__": raise SystemExit(main())
