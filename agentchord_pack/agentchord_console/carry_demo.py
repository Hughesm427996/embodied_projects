"""Controlled lift/carry test using native skills and measured rigid-body poses.

This is an added physical diagnostic, not the paper's benchmark or an LLM test.
"""
from __future__ import annotations

import copy
import json
import math
from pathlib import Path

from agentchord_console.grasp_check import preflight_first_grasp, value_json
from agentchord_console.plan_check import evaluate_plan
from agentchord_console.task_plan import _action, _hash, _pose_xyz, OBJECT_ARMS

LIFT_CANDIDATES = (.05, .04, .03)
SETTLE_STEPS = 30


def build_bundle(env, lift_m=.05):
    if lift_m not in LIFT_CANDIDATES:
        raise ValueError("Only reviewed 3–5 cm lift candidates are supported.")
    initial = {name: _pose_xyz(env, name) for name in ("plate", "fork", "spoon")}
    # This controlled target construction requires the pinned zero-translation
    # grasp frames. A changed asset must not silently alter the intended lift.
    for name in OBJECT_ARMS:
        grasp = value_json(env.obj_info[name]["grasp_pose_obj"])
        if len(grasp) != 4 or any(abs(float(grasp[i][3])) > 1e-6 for i in range(3)):
            raise ValueError(f"{name}: controlled lift requires a zero-translation grasp frame.")
    stages = ("grasp", "orient", "lift", "carry", "lower", "release", "return")
    nodes = [{"id": "v0_start", "semantic": "Fixed initial table scene"}]
    nodes += [{"id": f"v{i}_{stage}", "semantic": f"Planned {stage} state; physical effect requires measurement"}
              for i, stage in enumerate(stages, 1)]
    edges = []
    for i, stage in enumerate(stages, 1):
        edge = {"id": f"e{i:02d}_{stage}", "source": nodes[i-1]["id"], "target": nodes[i]["id"]}
        for obj, arm in OBJECT_ARMS.items():
            xyz, plate = initial[obj], initial["plate"]
            if stage == "grasp":
                action = _action("grasp", arm, obj_name=obj, pre_grasp_dis=.1)
            elif stage == "orient":
                action = _action("orient_eef", arm, direction="down")
            elif stage in ("lift", "carry", "lower"):
                x = xyz[0] - plate[0] if stage == "lift" else 0.
                y = xyz[1] - plate[1] if stage == "lift" else (.16 if obj == "fork" else -.16)
                z = .02 if stage == "lower" else xyz[2] + lift_m - plate[2]
                action = _action("move_relative_to_object", arm, obj_name="plate",
                                 x_offset=x, y_offset=y, z_offset=z)
            elif stage == "release":
                action = _action("open_gripper", arm)
            else:
                action = _action("back_to_initial_pose", arm)
            edge[f"{arm}_action"] = action
        edges.append(edge)
    task = "ControlledPickCarry_" + _hash({"initial": initial, "lift_m": lift_m})[:16]
    graph = {"task": task, "start": nodes[0]["id"], "goal": nodes[-1]["id"], "nodes": nodes, "edges": edges}
    return {"task_graph": graph,
            "recovery_spec": {"task": task, "recovery_bindings": []},
            "recovery_graph": {"task": task, "recovery_nodes": [], "recovery_edges": [], "recovery_branches": []},
            "metadata": {"task_graph_hash": _hash(graph), "recovery_enabled": False,
                         "console_scope": "controlled physical lift/carry diagnostic; no LLM planning",
                         "initial_xyz": initial, "requested_lift_m": lift_m}}


def select_bundle(env, *, evaluate=None, grasp_check=None, progress=None):
    evaluator = evaluate or evaluate_plan
    checker = grasp_check or preflight_first_grasp
    report = {"status": "failed", "attempts": [], "test": "controlled_pick_carry"}
    for lift in LIFT_CANDIDATES:
        if progress:
            progress(lift)
        bundle = build_bundle(env, lift)
        bundle, grasp, changes = checker(bundle, env)
        attempt = {"lift_m": lift, "grasp_check": grasp}
        report["attempts"].append(attempt)
        report["grasp_check"] = grasp
        if grasp.get("status") != "passed":
            report["status"] = grasp.get("status", "error")
            return None, report
        full = evaluator(bundle, env)
        attempt["plan_check"] = full
        if full.get("status") == "passed":
            report.update(status="passed", selected_lift_m=lift, grasp_changes=changes)
            return bundle, report
        if full.get("status") != "failed":
            report["status"] = full.get("status", "error")
            return None, report
    return None, report


class MotionTrace:
    """Observe each native step once. Never set poses or replay native actions."""
    def __init__(self, env, path, after_step=None):
        self.env, self.path = env, Path(path)
        self.original_step = env.step
        self.stream = None
        self.step_index = 0
        self.phase = "native_graph"
        self.last_action = None
        self.after_step = after_step
        self.had_instance_step = "step" in env.__dict__
        self.instance_step = env.__dict__.get("step")

    def sample(self):
        objects = {name: _pose_xyz(self.env, name) for name in ("plate", "fork", "spoon")}
        self.stream.write(json.dumps({"step": self.step_index, "phase": self.phase,
                                     "objects_xyz_m": objects}, allow_nan=False) + "\n")
        self.stream.flush()

    def attach(self):
        self.stream = self.path.open("w", encoding="utf-8", newline="\n")
        try:
            self.phase = "initial"
            self.sample()
            self.phase = "native_graph"
            def observed_step(action, *args, **kwargs):
                result = self.original_step(action, *args, **kwargs)
                self.step_index += 1
                self.last_action = action
                try:
                    if self.after_step is not None:
                        self.after_step(self)
                finally:
                    # Preserve the real state of a completed physics step even
                    # when an observation/injection callback stops execution.
                    self.sample()
                return result
            self.env.step = observed_step
        except BaseException:
            self.close()
            raise
        return self

    def settle(self):
        if self.last_action is None:
            raise RuntimeError("Native graph produced no measured physics steps.")
        self.phase = "post_return_settle"
        # Hold the final native joint action; only advance physics for observation.
        for _ in range(SETTLE_STEPS):
            self.env.step(self.last_action)

    def close(self):
        if self.had_instance_step:
            self.env.step = self.instance_step
        else:
            self.env.__dict__.pop("step", None)
        if self.stream is not None:
            self.stream.close()
            self.stream = None


def assess_trace(path):
    """Additional conservative motion checks; all visual verdicts remain pending."""
    rows = [json.loads(line) for line in Path(path).read_text(encoding="utf-8").splitlines() if line.strip()]
    return assess_rows(rows)


def assess_rows(rows):
    """Assess measured rows, including a renumbered post-detection segment."""
    if not rows or rows[0].get("phase") != "initial":
        raise ValueError("Initial measured pose is missing.")
    for index, row in enumerate(rows):
        if row.get("step") != index:
            raise ValueError("Trace has missing or repeated physics steps.")
        for name in ("plate", "fork", "spoon"):
            xyz = row["objects_xyz_m"][name]
            if len(xyz) != 3 or any(isinstance(v, bool) or not isinstance(v, (int, float)) or not math.isfinite(v) for v in xyz):
                raise ValueError("Invalid measured position.")
    tail = [row for row in rows if row["phase"] == "post_return_settle"]
    if tail and tail != rows[-len(tail):]:
        raise ValueError("Settlement samples must form the final contiguous segment.")
    metrics = {}
    for name, target_y in (("fork", .16), ("spoon", -.16)):
        initial = rows[0]["objects_xyz_m"][name]
        pts = [row["objects_xyz_m"][name] for row in rows]
        longest, run, run_pts, travel = 0, 0, [], 0.
        # Horizontal displacement must happen within a continuous elevated interval;
        # table sliding followed by a separate bounce cannot pass this test.
        for p in pts:
            if p[2] - initial[2] >= .02:
                run += 1
                run_pts.append(p)
                longest = max(longest, run)
                if run >= 5:
                    travel = max(travel, max(math.hypot(p[0]-q[0], p[1]-q[1]) for q in run_pts))
            else:
                run, run_pts = 0, []
        final, plate = pts[-1], rows[-1]["objects_xyz_m"]["plate"]
        stable_points = [row["objects_xyz_m"][name] for row in tail]
        drift = (max(math.dist(p, q) for p in stable_points for q in stable_points)
                 if len(stable_points) >= SETTLE_STEPS else None)
        metrics[name] = {"peak_rise_m": max(p[2]-initial[2] for p in pts),
                         "longest_elevated_interval_steps": longest,
                         "elevated_xy_travel_m": travel,
                         "measured_carry_check": longest >= 5 and travel >= .02,
                         "final_x_error_m": abs(final[0]-plate[0]),
                         "final_y_error_m": abs(final[1]-plate[1]-target_y),
                         "final_z_change_m": final[2]-initial[2],
                         "settle_drift_m": drift,
                         "final_pose_check": abs(final[0]-plate[0]) <= .02 and abs(final[1]-plate[1]-target_y) <= .02 and abs(final[2]-initial[2]) <= .02,
                         "stable_after_return_check": drift is not None and drift <= .005}
    return {"physics_steps": len(rows)-1, "settle_steps": len(tail), "objects": metrics,
            "motion_checks_passed": all(m["measured_carry_check"] and m["final_pose_check"] and m["stable_after_return_check"] for m in metrics.values()),
            "thresholds": {"min_rise_m": .02, "min_elevated_steps": 5, "min_elevated_xy_travel_m": .02,
                           "final_xy_tolerance_m": .02, "final_z_change_tolerance_m": .02, "settle_max_drift_m": .005},
            "note": "Added diagnostic thresholds, not official paper criteria. Object poses are measured, not commanded. Gripper contact and visual success require video review."}
