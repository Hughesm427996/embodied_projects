"""Native RecoveryAgent -> CompileAgent, with bounded model output checks.

The nominal graph and disturbance are controlled. Recovery choices come from
the API; there is no locally authored recovery path on generation failure.
"""
from __future__ import annotations
import copy
import json
import math
from pathlib import Path

from agentchord_console.carry_demo import select_bundle
from agentchord_console.disturbance_demo import (
    EDGE, EDGE_STEP, OBJECT, SHIFT, THRESHOLD, STEP_LIMIT, INJECTION_PROTOCOL,
    SETTLED_Y_RANGE, AXIS_TOLERANCE, MIN_TARGET_ERROR, with_reset_targets,
    preflight_recovery,
)
from agentchord_console.plan_check import serializable
from agentchord_console.task_plan import _hash

MAX_CALLS = 2
MAX_EDGES = 12
CONTRACT_VERSION = "dual_arm_joint_goal_v3"
TERMINAL_CONTRACTS = frozenset({"dual_arm_return_v2", CONTRACT_VERSION})


class RecoveryContractError(ValueError): pass


def _require(condition, message):
    if not condition: raise RecoveryContractError(message)


def _number(value, lower, upper):
    return (not isinstance(value, bool) and isinstance(value, (int, float))
            and math.isfinite(value) and lower <= value <= upper)


def validate_action(action, arm, nominal):
    if action is None: return
    _require(isinstance(action, dict) and set(action) == {"fn", "kwargs"}, "Action needs only fn and kwargs.")
    fn, kw = action["fn"], action["kwargs"]
    _require(isinstance(kw, dict) and kw.get("robot_name") == arm, "Action arm must match its field.")
    allowed = {
        "grasp": {"obj_name", "pre_grasp_dis"},
        "orient_eef": {"direction"},
        "move_relative_to_object": {"obj_name", "x_offset", "y_offset", "z_offset", "sample_num"},
        "open_gripper": set(), "back_to_initial_pose": set(),
        "move_to_target_pose": {"target_pose", "sample_num"},
    }
    _require(isinstance(fn, str) and fn in allowed, "Unsupported action function.")
    _require(set(kw) <= allowed[fn] | {"robot_name", "force_valid"}, "Unsupported action kwargs.")
    _require(kw.get("force_valid", False) is False, "force_valid must remain false.")
    if "sample_num" in kw:
        _require(type(kw["sample_num"]) is int and 10 <= kw["sample_num"] <= 50, "sample_num must be 10–50.")
    if fn == "grasp":
        obj = "fork" if arm == "left_arm" else "spoon"
        _require(kw.get("obj_name") == obj, "Keep the verified object-to-arm allocation.")
        _require(_number(kw.get("pre_grasp_dis", .05), .03, .1), "Grasp approach must be 3–10 cm.")
    elif fn == "orient_eef":
        _require(kw.get("direction") == "down", "Only the verified down orientation is supported.")
    elif fn == "move_relative_to_object":
        _require(kw.get("obj_name") in {"plate", "fork", "spoon"}, "Unknown reference object.")
        for name, bound in (("x_offset", .1), ("y_offset", .3), ("z_offset", .15)):
            _require(_number(kw.get(name, 0.), -bound, bound), f"Invalid {name}.")
    elif fn == "move_to_target_pose":
        target = kw.get("target_pose")
        initial = nominal["metadata"]["recovery_reset_targets"][arm]
        _require(isinstance(target, list) and len(target) == 4 and all(
            isinstance(row, list) and len(row) == 4 for row in target), "Reset pose must be a 4x4 matrix.")
        _require(all(_number(target[i][j], -2., 2.) and abs(target[i][j]-initial[i][j]) <= 1e-5
                     for i in range(4) for j in range(4)), "Reset target must match the recorded initial pose.")


def validate_spec(spec, nominal, graph_spec, *, binding_edge=EDGE):
    _require(isinstance(spec, dict) and set(spec) == {"task", "recovery_bindings"}, "Use the official lightweight recovery_bindings schema.")
    _require(spec["task"] == nominal["task_graph"]["task"], "Task name must stay unchanged.")
    bindings = spec["recovery_bindings"]
    _require(isinstance(bindings, list) and len(bindings) == 1, "This test requires exactly one recovery binding.")
    binding = bindings[0]
    _require(isinstance(binding, dict) and set(binding) == {
        "edge_id", "failure_name", "monitors", "recovery", "merge", "repeat_until_success"}, "Unexpected binding fields.")
    _require(binding["edge_id"] == binding_edge and binding["merge"] == "target", "Bind only the selected interrupted edge and merge to its target.")
    _require(binding["repeat_until_success"] is False, "Single-injection test requires repeat_until_success=false.")
    _require(isinstance(binding["failure_name"], str) and 0 < len(binding["failure_name"]) <= 128, "Invalid failure label.")
    _require(binding["monitors"] == [{"type":"object_moved", "objects":[OBJECT], "threshold":THRESHOLD}], "Keep the fork displacement monitor and 2 cm threshold.")
    steps = binding["recovery"]
    _require(isinstance(steps, list) and 1 <= len(steps) <= MAX_EDGES, "Recovery must have 1–12 steps.")
    nominal_ids = {edge["id"] for edge in nominal["task_graph"]["edges"]}
    for step in steps:
        _require(isinstance(step, dict), "Recovery step must be an object.")
        kind = step.get("type")
        fields = {"type", "name", "semantic", "force_valid"}
        if kind == "action":
            fields |= {"left_arm_action", "right_arm_action"}
            _require(any(step.get(f"{arm}_action") is not None for arm in ("left_arm", "right_arm")), "Empty recovery action.")
            for arm in ("left_arm", "right_arm"):
                validate_action(step.get(f"{arm}_action"), arm, nominal)
        elif kind == "replay_edge":
            fields.add("edge_id")
            _require(step.get("edge_id") in nominal_ids, "Unknown replay edge.")
        elif kind in {"regrasp", "regrasp_both"}:
            fields |= {"pre_grasp_dis"}
            _require(_number(step.get("pre_grasp_dis", .1), .03, .1), "Grasp approach must be 3–10 cm.")
            if kind == "regrasp_both":
                fields.add("arms")
                _require(step.get("arms") == {"left_arm":"fork", "right_arm":"spoon"}, "Invalid dual grasp allocation.")
            else:
                fields |= {"robot_name", "obj_name"}
                _require((step.get("robot_name"),step.get("obj_name")) in {
                    ("left_arm","fork"),("right_arm","spoon")}, "Invalid grasp allocation.")
        else:
            raise RecoveryContractError("Unsupported recovery step type.")
        _require(set(step) <= fields and step.get("force_valid",False) is False, "Unknown step fields or forced IK.")
        for key in ("name", "semantic"):
            if key in step: _require(isinstance(step[key],str) and len(step[key]) <= 512, "Invalid step label.")
    canonical, issues = graph_spec.normalize_recovery_spec(nominal["task_graph"], spec)
    _require(not issues, "Official schema normalization failed: " + "; ".join(issues))
    return canonical


def validate_compiled(bundle, nominal, *, require_dual_return=False, terminal_contract=None, binding_edge=EDGE):
    _require(terminal_contract is None or terminal_contract in TERMINAL_CONTRACTS,
             "Unknown terminal return contract.")
    _require(bundle.get("task_graph") == nominal["task_graph"], "Compiler changed the nominal task.")
    graph = bundle.get("recovery_graph", {})
    edges, branches = graph.get("recovery_edges"), graph.get("recovery_branches")
    _require(isinstance(edges,list) and 1 <= len(edges) <= MAX_EDGES, "Invalid compiled recovery edges.")
    _require(isinstance(branches,list) and len(branches) == 1, "Extra or missing compiled recovery branches.")
    branch = branches[0]
    _require(branch.get("edge_id") == binding_edge and branch.get("monitor_sequence") == [
        {"fn":"monitor_object_moved","kwargs":{"obj_name":OBJECT,"threshold":THRESHOLD}}], "Compiled monitor changed.")
    _require(branch.get("recovery_edges") == [edge["id"] for edge in edges], "Compiled path contains unrelated recovery edges.")
    fork_grasp = False
    last_actions = {}
    for edge in edges:
        for arm in ("left_arm", "right_arm"):
            action = edge.get(f"{arm}_action")
            validate_action(action, arm, nominal)
            if action is not None: last_actions[arm] = action["fn"]
            fork_grasp |= bool(action and action["fn"] == "grasp" and action["kwargs"]["obj_name"] == OBJECT)
    _require(fork_grasp, "Recovery must include grasping the displaced fork.")
    if require_dual_return or terminal_contract == "dual_arm_return_v2":
        _require(all(last_actions.get(arm) == "back_to_initial_pose" for arm in ("left_arm","right_arm")),
            "Interrupted e07_return applies to BOTH arms. merge=target does not resume the unmentioned arm. "
            "The last non-idle action on each arm must be native back_to_initial_pose (directly or via replay_edge). "
            "Spoon may remain untouched, but right_arm must finish its return explicitly. "
            "A move_to_target_pose reset does not guarantee the initial joint configuration. "
            "Observed last actions: "+json.dumps(last_actions))
    elif terminal_contract == CONTRACT_VERSION:
        _require(all(last_actions.get(arm) in {"back_to_initial_pose","move_to_target_pose"}
                     for arm in ("left_arm","right_arm")),
            "Interrupted e07_return applies to BOTH arms; merge=target does not resume an idle arm. "
            "Each arm needs an explicit native return: back_to_initial_pose or move_to_target_pose "
            "to its recorded initial matrix. right_arm must finish its return even if the spoon stays still. "
            "Either route must pass terminal joint and open-gripper prediction; a matching pose alone is insufficient. "
            "Observed last actions: "+json.dumps(last_actions))
    return edges


def prediction_feedback(prediction):
    """Keep full diagnostics on disk; send the model the specific failed route."""
    attempts = prediction.get("attempts", [])
    last = attempts[-1] if attempts else {}
    failed = next((call for call in reversed(last.get("ik_calls", []))
                   if call.get("success") is False), None)
    summary = {"status":prediction.get("status"), "reason":last.get("reason",prediction.get("reason")),
        "failed_edge":last.get("failed_edge"), "failed_ik":failed,
        "generated_prefix":[{"id":edge["id"], "generated_arms":edge["arms"]}
                            for edge in last.get("edges", [])],
        "predicted_robot_return":last.get("predicted_robot_return")}
    guidance = ("Preserve unrelated actions that already passed, especially measured-pose resets. "
        "Revise the failing arm's route using allowed native skills; do not bypass IK. "
        "Native back_to_initial_pose adds an 8 cm tool-axis retraction even after an interrupted return "
        "already completed that segment. A move_to_target_pose to the recorded initial matrix is an "
        "allowed return alternative, but final joint/open-gripper tolerances are still mandatory. "
        "Native grasp can also retract automatically from an interrupted pose; a measured initial "
        "pose reset is available before regrasp. Do not remove a successful reset merely to shorten the plan.")
    return json.dumps(summary,ensure_ascii=False)+"\n"+guidance


class AuditedLLM:
    def __init__(self, client, path, write_json, model, base_url):
        self.client, self.path, self.write_json = client, Path(path), write_json
        self.model, self.base_url, self.calls = model, base_url, []
        self.stage = "RecoveryAgent"

    def invoke(self, prompt):
        _require(len(self.calls) < MAX_CALLS, "Recovery API invocation budget exceeded.")
        public_messages = prompt.to_messages() if hasattr(prompt, "to_messages") else prompt
        messages = [{"role":getattr(m,"type",type(m).__name__), "content":m.content}
                    for m in public_messages]
        call = {"index":len(self.calls)+1, "stage":self.stage, "model":self.model,
                "base_url":self.base_url, "messages":messages, "request_hash":_hash(messages), "status":"started"}
        self.calls.append(call)
        self.write_json(self.path, {"max_invocations":MAX_CALLS,"calls":self.calls})
        try:
            response = self.client.invoke(prompt)
            _require(isinstance(response.content,str) and len(response.content) <= 128_000, "Model response is not bounded JSON text.")
            call.update(status="returned", content=response.content, response_hash=_hash(response.content))
            usage = getattr(response,"usage_metadata",None) or {}
            call["tokens"] = {k:usage[k] for k in ("input_tokens","output_tokens","total_tokens")
                              if type(usage.get(k)) is int}
            return response
        except Exception as exc:
            call.update(status="error", error_type=type(exc).__name__)
            raise
        finally:
            self.write_json(self.path, {"max_invocations":MAX_CALLS,"calls":self.calls})


def experiment_context(nominal):
    return (
        "Single controlled displacement experiment. Preserve the nominal graph and its task name. "
        f"After cutlery placement and release, the fork is displaced world +Y by about 0.055 m during {EDGE}, "
        f"native return sample {EDGE_STEP}. Both grippers are open; the return is partially executed. "
        "Generate exactly one binding on that edge, object_moved on fork with threshold 0.02, "
        "merge=target and repeat_until_success=false. The goal remains fork at plate world Y+0.16, "
        "spoon at Y-0.16, X aligned with plate; release and return arms. The fork must be picked up, "
        "lifted 3–5 cm and carried back, not pushed across the table. The spoon may stay in place or be regrasped. "
        "Choose the recovery steps yourself using official replay_edge/regrasp/regrasp_both/action forms. "
        "Allowed actions are grasp, orient_eef(direction=down), move_relative_to_object, open_gripper, "
        "back_to_initial_pose and move_to_target_pose. Keep left_arm=fork and right_arm=spoon; "
        "use only plate/fork/spoon references; pre_grasp_dis is 0.03–0.10 m; "
        "sample_num is 10–50; relative offsets are bounded by |X|<=0.10, |Y|<=0.30, |Z|<=0.15 m. "
        "force_valid stays false. At most 12 recovery steps. "
        "Native back_to_initial_pose first retracts 8 cm along the tool; replaying it as an initial reset "
        "during an interrupted return can retract twice. If resetting before regrasp, use move_to_target_pose "
        "with the measured initial matrices below; these are constraints, not a prescribed recovery sequence. "
        "Native grasp can automatically retract from the interrupted pose before its approach; that "
        "extra motion can be unreachable. A recorded initial-pose reset before regrasp is available. "
        "For reference, lower-to-table in the verified nominal graph uses plate z_offset=0.02. "
        "The actual contact outcome still needs simulation.\nMeasured initial arm reset matrices:\n" +
        json.dumps(nominal["metadata"]["recovery_reset_targets"],ensure_ascii=False) +
        "\nInitial object XYZ:\n" + json.dumps(nominal["metadata"]["initial_xyz"]) +
        "\nImportant terminal goal: e07_return was interrupted for BOTH arms. An idle arm remains at the "
        "measured interrupted sample, even when its object needs no repair. merge=target terminates the edge "
        "and never resumes the remaining nominal return trajectory. Therefore the last non-idle action for "
        "EACH arm must explicitly return via native back_to_initial_pose (directly or replay_edge) OR "
        "move_to_target_pose to its recorded initial matrix. Do not omit the right arm return. Its spoon "
        "may stay in place, and the right arm can move without grasping it. An early initial-pose reset "
        "can also serve as that arm's final return if no later action changes its pose or joints. "
        "A move_to_target_pose reset is allowed only when the predicted final joints also meet the "
        "initial joint goal; matching the matrix alone is not sufficient. Choose a kinematically feasible "
        "route for both arms. Do not call back_to_initial_pose from an already retracted arm simply "
        "to satisfy an action-name requirement; its extra 8 cm pre-retraction may be unreachable. "
        "The cloned prediction must finish both arm joint vectors within 0.01 rad of initial targets and "
        "both grippers within 0.002 m of the open target. Post-execution measured return is still checked. "
        "\nRequired root fields: task and recovery_bindings. Required binding fields: edge_id, failure_name, "
        "monitors, recovery, merge, repeat_until_success. Output only the official authoring JSON."
    )


def prepare_llm_bundle(env, run_dir, *, write_json, model, base_url, progress=None,
                       select=None, predictor=None):
    from embodichain.agents.agentchord import graph_spec
    run_dir = Path(run_dir)
    nominal, nominal_check = (select or select_bundle)(env)
    report = {"status":nominal_check["status"], "nominal":nominal_check,
              "test":"native_recovery_agent_generation", "attempts":[],
              "generation_phase":"before_execution", "local_recovery_fallback":False,
              "contract_version":CONTRACT_VERSION}
    if nominal is None: return None, report
    nominal = with_reset_targets(nominal,env)
    agent, compiler = env.recovery_agent, env.compile_agent
    old_agent_llm, old_compiler_llm = agent.llm, compiler.llm
    audit = AuditedLLM(old_agent_llm, run_dir/"llm_recovery_calls.json", write_json, model, base_url)
    generation_path = run_dir/"llm_recovery_generation.json"
    agent.llm = compiler.llm = audit
    feedback = ""
    try:
        for index in (1,2):
            attempt = {"index":index,"status":"started"}
            report["attempts"].append(attempt)
            directory = run_dir/"llm_generation"/f"attempt_{index}"
            directory.mkdir(parents=True,exist_ok=False)
            if progress: progress(f"官方 RecoveryAgent 调用 DeepSeek 生成恢复方案（尝试 {index}/2）。")
            audit.stage = "RecoveryAgent"
            inputs = agent.get_composed_observations(task_graph=json.dumps(nominal["task_graph"]),
                task_prompt="Place fork at plate world Y+0.16 m and spoon at Y-0.16 m, X aligned with plate. Release both cutlery items and return both arms to their initial poses.",
                log_dir=directory, regenerate=True)
            inputs["recovery_rules"] = str(inputs.get("recovery_rules",""))+"\n"+experiment_context(nominal)+feedback
            try:
                content = agent.generate(**inputs)
                raw_spec = json.loads(content)
                write_json(run_dir/f"agent_recovery_spec.raw_attempt_{index}.json",raw_spec)
                canonical = validate_spec(raw_spec,nominal,graph_spec)
                audit.stage = "CompileAgent_canonicalization"
                path, _, _ = compiler.generate(log_dir=directory, regenerate=True,
                    task_graph=nominal["task_graph"], recovery_spec=raw_spec, recovery_enabled=True)
                bundle = json.loads(Path(path).read_text(encoding="utf-8"))
                _require(bundle.get("recovery_spec") == canonical, "Compiler unexpectedly changed the checked recovery spec.")
                edges = validate_compiled(bundle,nominal,terminal_contract=CONTRACT_VERSION)
                graph_spec.compile_agent_graph_spec(bundle["task_graph"],bundle["recovery_graph"],env=env)
                prediction = (predictor or preflight_recovery)(env,nominal,recovery_plan=edges,require_return=True)
                attempt["recovery_prediction"] = prediction
                if prediction["status"] == "error":
                    raise RuntimeError("Native recovery prediction raised an error; see generation report.")
                _require(prediction["status"] == "passed", "Generated recovery IK/trajectory prediction failed: "+prediction_feedback(prediction))
                _require(prediction.get("terminal_return_required") is True
                    and prediction["attempts"][-1].get("predicted_robot_return",{}).get("passed") is True,
                    "Both arm/gripper terminal predictions must be checked before execution.")
                predicted_steps = EDGE_STEP+200+30+sum(max(a["trajectory_steps"] for a in e["arms"])
                    for e in prediction["attempts"][-1]["edges"])
                _require(predicted_steps <= STEP_LIMIT, "Predicted recovery exceeds the physics step budget.")
                native_bundle = copy.deepcopy(bundle)
                write_json(run_dir/"agent_compiled_graph.sdk.json",native_bundle)
                write_json(run_dir/"agent_recovery_spec.json",bundle["recovery_spec"])
                bundle["metadata"].update(copy.deepcopy(nominal["metadata"]))
                bundle["metadata"].update(recovery_enabled=True, llm_recovery_generated=True,
                    llm_recovery_contract=CONTRACT_VERSION,
                    recovery_author="native RecoveryAgent via API and native CompileAgent",
                    console_scope="controlled nominal task with model-generated preplanned recovery",
                    nominal_graph_hash=_hash(bundle["task_graph"]), generation_phase="before_execution",
                    disturbance={"edge":EDGE,"edge_step":EDGE_STEP,"object":OBJECT,"relative_xyz_m":SHIFT,
                        "method":"native misplaced_object","native_settle_physics_steps":100,
                        "protocol":INJECTION_PROTOCOL,"settled_y_range_m":list(SETTLED_Y_RANGE),
                        "cross_axis_tolerance_m":AXIS_TOLERANCE,"min_target_error_m":MIN_TARGET_ERROR})
                attempt["status"] = "passed"
                report.update(status="passed", recovery_prediction=prediction,
                    selected_attempt=index, api_invocations=len(audit.calls),
                    model=model, base_url=base_url, raw_spec_hash=_hash(raw_spec),
                    canonical_spec_hash=_hash(canonical), compiled_recovery_hash=_hash(bundle["recovery_graph"]))
                write_json(generation_path,report)
                if progress: progress("模型恢复方案已通过 schema、原编译器、完整轨迹及双臂回位预检，开始仿真验证。")
                return bundle, report
            except (RecoveryContractError,ValueError) as exc:
                attempt.update(status="rejected",reason=str(exc))
                if not audit.calls or audit.calls[-1]["status"] != "returned": raise
                feedback = ("\nYour previous output was rejected. Revise the recovery JSON; do not use a fixed fallback.\n"
                    "Previous output:\n"+audit.calls[-1]["content"]+"\nDiagnostic:\n"+str(exc))
                if progress: progress("恢复方案未通过检查；记录原因后允许模型修正一次。" if index==1 else "第二次仍未通过，停止本轮。")
                write_json(generation_path,report)
        report.update(status="failed",api_invocations=len(audit.calls))
    except Exception as exc:
        report.update(status="error",error_type=type(exc).__name__,api_invocations=len(audit.calls))
        if report["attempts"]: report["attempts"][-1].update(status="error",error_type=type(exc).__name__)
    finally:
        agent.llm, compiler.llm = old_agent_llm, old_compiler_llm
        write_json(generation_path,report)
    return None, report
