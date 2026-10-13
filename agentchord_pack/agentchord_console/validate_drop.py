#!/usr/bin/env python3
"""Controlled physical finger-release drop, recovery and nominal continuation."""
from __future__ import annotations
import argparse
import copy
import datetime as dt
import json
import math
from pathlib import Path
import sys
from types import SimpleNamespace
import zipfile

sys.path.insert(0,str(Path(__file__).resolve().parent.parent))
from agentchord_console import console
from agentchord_console.drop_demo import (EDGE, EDGE_STEP, OPEN_STEPS, CONTRACT, HOLD_POLICY, HOLD_STEPS,
    RIGHT_GRIP_POLICY, RIGHT_CARRY_EDGES, branch_goal, checked_edges, fixed_hold_trajectory)
from agentchord_console.carry_demo import assess_rows
from agentchord_console.disturbance_demo import assess_predicted_robot_return
from agentchord_console.validate_demo import assess_run, read_json
from agentchord_console.validate_disturbance import _close, _scene_hash, _poses_match, _robot_return
from agentchord_console.validate_llm_recovery import assess_generation


def assess_commands(event,ledger,rows):
    """Check recorded commands against measured pre-release state; never infer a fall from an open command."""
    checks={}
    try:
        commands=ledger['commands'];start=event['commanded_step']
        opened=event['requested_open_state'][0]
        checks['single_controlled_release']=(event['object']=='fork' and event['protocol']==ledger['protocol']==CONTRACT
            and event['edge']==EDGE and event['edge_step']==EDGE_STEP and event['finger_indices']==[12,13]
            and event['object_pose_setter_calls']==0 and start==event['step']+1
            and 1<=len(commands)<=OPEN_STEPS)
        checks['only_left_finger_commands_changed']=True
        for i,item in enumerate(commands):
            a,b=item['original_action'][0],item['executed_action'][0]
            valid=(len(a)==len(b)==16 and all(type(x) in (int,float) and math.isfinite(x) for x in a+b)
                and item['step']==start+i and item['edge']==EDGE and item['edge_step']==EDGE_STEP+i
                and _close(item['finger_target_m'],opened,1e-6)
                and all(_close(b[j],opened,1e-6) for j in (12,13))
                and all(a[j]==b[j] for j in range(16) if j not in (12,13))
                and all(a[j]<=opened-.01 for j in (12,13))
                and rows[item['step']]['phase']==EDGE)
            checks['only_left_finger_commands_changed'] &= valid
        checks['before_release_pose_matches_trace']=_close(event['before_objects'],rows[start-1]['objects_xyz_m'],1e-6)
        checks['before_release_fingers_closed']=all(event['before_robot_qpos'][0][j]<=opened-.01 for j in (12,13))
        return {'passed':all(checks.values()),'checks':checks,'command_count':len(commands)}
    except (KeyError,ValueError,TypeError,IndexError) as exc:
        return {'passed':False,'checks':checks,'error':f'{type(exc).__name__}: {exc}'}


def assess_prediction(report,source):
    """Recompute held-branch and final-return checks from their recorded vectors."""
    checks={}
    try:
        edges=checked_edges(source,source)
        check=report['recovery_prediction']['attempts'][-1]
        h=check['held_branch_goal']
        state=SimpleNamespace(left_arm_current_xpos=h['left_pose'],right_arm_current_qpos=h['right_qpos'],
            left_arm_current_gripper_state=h['grippers']['left'],right_arm_current_gripper_state=h['grippers']['right'])
        held=SimpleNamespace(left_arm_current_xpos=h['left_pose_target'],right_arm_current_qpos=h['right_qpos_target'],close_state=h['closed_target'])
        recomputed=branch_goal(state,held)
        checks['held_goal_recomputed']=recomputed==h and h['passed'] is True
        terminal=check['predicted_robot_return'];state=SimpleNamespace()
        original=SimpleNamespace(open_state=terminal['arms']['left']['open_gripper_target'])
        for side in ('left','right'):
            arm=terminal['arms'][side]
            setattr(state,f'{side}_arm_current_qpos',arm['predicted_qpos'])
            setattr(state,f'{side}_arm_current_gripper_state',arm['predicted_gripper_state'])
            setattr(original,f'{side}_arm_init_qpos',arm['initial_qpos_target'])
        checks['remaining_nominal_return_recomputed']=terminal==assess_predicted_robot_return(state,original) and terminal['passed'] is True
        path=source['task_graph']['edges'];i=next(i for i,e in enumerate(path) if e['id']==EDGE)
        prefix=check['nominal_prefix']
        checks['nominal_prefix_checked']=[e['id'] for e in prefix]==[e['id'] for e in path[:i+1]]
        holdarms=prefix[-1]['arms']
        checks['predicted_fixed_hold_verified']=len(holdarms)==2 and all(
            a['trajectory_steps']==HOLD_STEPS and a['stationary_hold']==fixed_hold_trajectory(
                a['stationary_hold']['raw_trajectory'],a['stationary_hold']['seed_qpos'],a['stationary_hold']['closed_target'])[1]
            for a in holdarms)
        checks['original_nominal_tail_checked']=[e['id'] for e in check['remaining_nominal_edges']]==[e['id'] for e in path[i+1:]]
        checks['recovery_actions_checked']=[e['id'] for e in check['edges']]==[e['id'] for e in edges]
        checks['drop_contract_matches']=report['drop_contract_version']==source['metadata']['drop_recovery_contract']==CONTRACT
        return {'passed':all(checks.values()),'checks':checks}
    except (KeyError,ValueError,TypeError,IndexError,AttributeError,RuntimeError) as exc:
        return {'passed':False,'checks':checks,'error':f'{type(exc).__name__}: {exc}'}


def assess_hold_commands(ledger,original_command,ends,mode):
    checks={}
    try:
        checks['both_stationary_arms_recorded']=ledger['policy']==HOLD_POLICY and set(ledger['arms'])=={'left','right'}
        indices={'left':[0,2,4,6,8,10,12,13],'right':[1,3,5,7,9,11,14,15]}
        for side,ids in indices.items():
            arm=ledger['arms'][side]
            fixed,audit=fixed_hold_trajectory(arm['raw_trajectory'],arm['seed_qpos'],arm['closed_target'])
            checks[f'{side}_stationary_native_target_verified']=arm==audit
            checks[f'{side}_original_command_matches_fixed_target']=_close([original_command[i] for i in ids],fixed[-1].tolist(),1e-6)
        end=next(e for e in ends if e.get('edge')==EDGE)
        count=end['executed_steps']
        checks['fixed_hold_duration_executed']=(count==HOLD_STEPS if mode=='off' else EDGE_STEP<=count<=HOLD_STEPS and end.get('monitor_index')==0)
        return {'passed':all(checks.values()),'checks':checks}
    except (KeyError,ValueError,TypeError,IndexError,RuntimeError,StopIteration) as exc:
        return {'passed':False,'checks':checks,'error':f'{type(exc).__name__}: {exc}'}


def assess_native_right_release(source,commands,ends,closed,opened):
    """Recompute pinned open_gripper sampling and native drive padding.

    The SDK's mul_linear_expand uses alpha=k/sample_num, excluding the endpoint.
    Compare commands with commands; physical final opening remains a separate
    measured robot-return check. Already-open native skip is also accounted for.
    """
    checks={}
    try:
        edge=next(e for e in source['task_graph']['edges'] if e['id']=='e06_release')
        right=edge['right_arm_action'];left=edge['left_arm_action']
        checks['native_open_skills_declared']=right['fn']==left['fn']=='open_gripper'
        rk,lk=right.get('kwargs',{}),left.get('kwargs',{})
        n,m=rk.get('sample_num',15),lk.get('sample_num',15)
        if type(n) is not int or type(m) is not int or min(n,m)<1 or max(n,m)>200:
            raise ValueError('Invalid native release sampling.')
        end=next(e for e in ends if e['edge']=='e06_release')
        count=end['executed_steps'];start=end['step']-count
        if start<1:raise ValueError('Pre-release cache sample missing.')
        cached=commands[start-1]['cached_right_gripper_state'][0]
        threshold=rk.get('open_threshold',.01)
        if any(type(x) not in (int,float) or not math.isfinite(x) for x in (closed,opened,cached,threshold)):
            raise ValueError('Non-finite release state.')
        skip=cached>=opened-threshold
        native=[cached] if skip else [(1-k/n)*closed+(k/n)*opened for k in range(n)]
        # Original drive extends the shorter arm with its final native command.
        checks['native_release_step_count']=count in {len(native),max(len(native),m)}
        part=commands[start:end['step']]
        expected=native+[native[-1]]*max(count-len(native),0)
        checks['sampled_release_commands_preserved']=(len(part)==len(expected)==count
            and all(c['edge']==c['phase']=='e06_release' and c['right_preserved'] is False
                and all(_close(c[field][0][j],goal,1e-6) for field in ('original_action','executed_action') for j in (14,15))
                for c,goal in zip(part,expected)))
        # The next nominal return must use the SDK's updated open-state cache,
        # or its unchanged already-open cache if open_gripper skipped the ramp.
        next_target=cached if skip else opened
        next_command=commands[end['step']]
        checks['opening_continues_during_nominal_return']=(next_command['edge']==next_command['phase']=='e07_return'
            and next_command['right_preserved'] is False
            and _close(next_command['cached_right_gripper_state'],[next_target],1e-6)
            and all(_close(next_command[field][0][j],next_target,1e-6) for field in ('original_action','executed_action') for j in (14,15)))
        return {'passed':all(checks.values()),'checks':checks,'native_sample_count':len(native),
            'executed_sample_count':count,'native_skip':skip,'open_target_m':opened,
            'last_sample_target_m':native[-1],'return_target_m':next_target}
    except (KeyError,ValueError,TypeError,IndexError,StopIteration) as exc:
        return {'passed':False,'checks':checks,'error':f'{type(exc).__name__}: {exc}'}


def assess_right_grip(ledger,source,hold,release,rows,ends,mode,result):
    """Audit all commands and the exact interval of the disclosed local adapter.

    V2 evidence remains assessable under its original protocol. It cannot claim
    the new grip retention audit just because a later validator was installed.
    """
    policy=source.get('metadata',{}).get('right_grip_policy')
    if policy is None and not ledger:
        return {'passed':True,'applicable':False,'note':'Legacy v2: no right grip adapter or command audit.'}
    checks={}
    try:
        recovery=checked_edges(source,source) if mode=='on' else []
        ids=[e['id'] for e in recovery]
        protected=set(ids)|set(RIGHT_CARRY_EDGES)
        target=hold['arms']['right']['closed_target'][0]
        commands=ledger['commands'];events=ledger['events']
        checks['declared_command_policy']=(policy==ledger['policy']==RIGHT_GRIP_POLICY
            and ledger['enabled'] is (mode=='on') and ledger['finger_indices']==[14,15]
            and ledger['object_pose_setter_calls']==0 and set(ledger['protected_edges'])==protected)
        checks['complete_contiguous_command_audit']=len(commands)==rows[-1]['step']
        if mode=='on':
            recovery_ends=[e for e in ends if e['edge'] in ids]
            start=recovery_ends[0]['step']-recovery_ends[0]['executed_steps']
            stop=next(e['step'] for e in ends if e['edge']=='e05_lower')
            cached=events[0]['cached_right_gripper_state']
            measured=events[0]['measured_robot_qpos'][0]
            checks['activation_after_native_sync_and_end_at_release']=(len(events)==2
                and events[0]['event']=='activate' and events[0]['step']==start and events[0]['edge']==ids[0]
                and _close(events[0]['target_m'],target,1e-6) and _close(cached,[measured[14]],1e-6)
                and events[1]=={'event':'deactivate','step':stop,'edge':'e06_release'})
        else:
            start=stop=-1;cached=None
            checks['off_group_no_retention']=events==[]
        injected={c['step']:c for c in release['commands']}
        checks['only_declared_finger_commands_changed']=True
        checks['measured_cache_preserved']=True
        for step,c in enumerate(commands,1):
            a,b=c['original_action'][0],c['executed_action'][0]
            preserve=mode=='on' and start<step<=stop
            valid=(len(a)==len(b)==16 and all(type(x) in (int,float) and math.isfinite(x) for x in a+b)
                and c['step']==step and c['phase']==rows[step]['phase']
                and c['right_preserved'] is preserve)
            if preserve:
                valid &= (c['edge'] in protected and c['phase']==c['edge']
                    and all(_close(b[j],target,1e-6) for j in (14,15))
                    and all(a[j]==b[j] for j in range(16) if j not in (14,15)))
                checks['measured_cache_preserved'] &= (_close(c['cached_right_gripper_state'],cached,1e-6)
                    and all(_close(a[j],cached[0],1e-6) for j in (14,15)))
            elif step in injected:
                valid &= c['original_action']==injected[step]['original_action'] and c['executed_action']==injected[step]['executed_action']
            else:valid &= a==b
            checks['only_declared_finger_commands_changed'] &= bool(valid)
        native_release=assess_native_right_release(source,commands,ends,target,release['commands'][0]['finger_target_m'])
        checks['nominal_right_release_preserved']=native_release['passed']
        return {'passed':all(checks.values()),'applicable':True,'checks':checks,'command_count':len(commands),
            'native_release':native_release,'release_check_version':'native_sampled_open_v4',
            'policy':policy,'target_m':target,'protected_start_step':start+1 if mode=='on' else None,
            'protected_end_step':stop if mode=='on' else None}
    except (KeyError,ValueError,TypeError,IndexError,RuntimeError,StopIteration) as exc:
        return {'passed':False,'applicable':True,'checks':checks,'error':f'{type(exc).__name__}: {exc}'}


def assess_trial(run_dir):
    folder=Path(run_dir).resolve();report=assess_run(folder)
    manifest=read_json(folder/'run.json') or read_json(folder/'validation_context.json')
    result=read_json(folder/'result.json');cfg=read_json(folder/'gym_config.json')
    source=read_json(folder/'agent_compiled_graph.drop.json');used=read_json(folder/'agent_compiled_graph.used.json')
    preflight=read_json(folder/'plan_check.json');mode=manifest.get('drop_mode')
    events=read_json(folder/'recovery_events.json').get('events',[])
    injections=[e for e in events if e.get('event')=='injection']
    drops=[e for e in events if e.get('event')=='drop_observed']
    detects=[e for e in events if e.get('event')=='detection']
    ends=[e for e in events if e.get('event')=='graph_edge_end']
    report.update(test='finger_release_drop_recovery',mode=mode,scene_config_hash=_scene_hash(cfg),
        injection_count=len(injections),drop_count=len(drops),detection_count=len(detects),
        trial_evidence_valid=False,native_detection_verified=False,physical_drop_verified=False,
        note='Controlled local finger command injection; falling is measured in physics. Native position monitor and preplanned API recovery. Video review still required.')
    mode_ok=(mode in ('off','on') and manifest.get('recovery') is (mode=='on')
        and manifest.get('llm_recovery') is (mode=='on') and manifest.get('disturbance_mode') is None
        and manifest.get('natural_language') is False and manifest.get('plate_layout') is False
        and not manifest.get('pick_carry') and manifest.get('task_prompt')==console.DEFAULT_GOAL
        and cfg.get('id')=='RearrangementAgent-v3' and result.get('controlled_drop') is True)
    report['checked_graph_executed']=bool(source) and used==source and preflight.get('status')=='passed'
    provenance=mode=='off'
    if mode=='on':
        report['generation_checks']=assess_generation(folder,source,binding_edge=EDGE)
        report['held_and_tail_prediction']=assess_prediction(read_json(folder/'llm_recovery_generation.json'),source)
        provenance=report['generation_checks']['passed'] and report['held_and_tail_prediction']['passed'] and result.get('llm_recovery_generated') is True
    try:
        rows=[json.loads(line) for line in (folder/'object_motion.jsonl').read_text(encoding='utf-8').splitlines() if line.strip()]
        report['whole_motion']=assess_rows(rows)
        if not _poses_match(rows[0],result['initial_objects']) or not _poses_match(rows[-1],result['final_objects']):
            raise ValueError('Trace and saved scene snapshots disagree.')
        if not _close(read_json(folder/'result.initial.json'),result['initial_objects'],1e-6):
            raise ValueError('Recorded initial height reference disagrees.')
        if len(injections)!=1 or len(drops)!=1 or len(detects)!=1:raise ValueError('Require one release, one measured fall and one detection.')
        event,drop,detect=injections[0],drops[0],detects[0];report['injection']=event
        ledger=read_json(folder/'gripper_override.json')
        report['command_checks']=assess_commands(event,ledger,rows)
        report['hold_command_checks']=assess_hold_commands(read_json(folder/'hold_commands.json'),ledger['commands'][0]['original_action'][0],ends,mode)
        report['right_grip_command_checks']=assess_right_grip(read_json(folder/'right_grip_commands.json'),source,
            read_json(folder/'hold_commands.json'),ledger,rows,ends,mode,result)
        start=event['commanded_step'];dropstep=drop['step'];detectstep=detect['step']
        before=event['before_objects']['fork'];point=rows[dropstep]['objects_xyz_m']['fork']
        rise=before[2]-rows[0]['objects_xyz_m']['fork'][2];fall=before[2]-point[2]
        report['physical_drop_verified']=(report['command_checks']['passed'] and report['hold_command_checks']['passed'] and start<=dropstep and start<=detectstep
            and rise>=.02 and fall>=.02 and drop['object']=='fork' and _close(drop['fall_m'],fall,1e-6)
            and _close(drop['xyz_m'],point,1e-6) and result.get('physical_drop_observed') is True)
        report.update(pre_release_rise_m=rise,measured_fall_m=fall,first_drop_step=dropstep)
        monitors=read_json(folder/'monitor_checks.json')
        role='native_graph_monitor' if mode=='on' else 'read_only_off_group_observer'
        first=next(e for e in monitors if e.get('result') is True)
        report['native_detection_verified']=(detect['fn']=='monitor_object_moved' and detect['edge']==EDGE
            and detect['threshold_m']==.02 and detect['result'] is True and detect['role']==role
            and first['step']==detectstep and first['edge']==EDGE and first['role']==role
            and first.get('reference_mode')=='stationary_hold_start'
            and _close([row[3] for row in first['reference_pose'][:3]],rows[start-EDGE_STEP]['objects_xyz_m']['fork'],1e-6)
            and math.dist(rows[detectstep]['objects_xyz_m']['fork'],
                rows[start-EDGE_STEP]['objects_xyz_m']['fork'])>.02)
        path=source['task_graph']['edges'];i=next(i for i,e in enumerate(path) if e['id']==EDGE)
        expected_tail=[e['id'] for e in path[i+1:]]
        actual_tail=[e['edge'] for e in ends if e['edge'] in expected_tail]
        starts=[e for e in events if e.get('event')=='recovery_edge_start']
        if mode=='on':
            recovery=checked_edges(source,source);ids=[e['id'] for e in recovery]
            recovered=[e['edge'] for e in ends if e['edge'] in ids and e.get('monitor_index') is None]
            holdends=[e for e in ends if e['edge']==EDGE]
            ordered=[e['edge'] for e in ends]
            report['runtime_branch_verified']=([e['edge'] for e in starts]==ids and recovered==ids
                and len(holdends)==1 and holdends[0].get('monitor_index')==0
                and ordered==[e['id'] for e in path[:i+1]]+ids+expected_tail and actual_tail==expected_tail)
            report['recovery_edges_completed']=recovered
            recovery_end=next(e['step'] for e in ends if e['edge']==ids[-1])
            spoon_reference=rows[detectstep]['objects_xyz_m']['spoon']
            spoon_points=[r['objects_xyz_m']['spoon'] for r in rows[detectstep:recovery_end+1]]
            report['right_spoon_retention']={'max_fall_during_left_recovery_m':max(spoon_reference[2]-p[2] for p in spoon_points),
                'max_xy_drift_during_left_recovery_m':max(math.hypot(p[0]-spoon_reference[0],p[1]-spoon_reference[1]) for p in spoon_points)}
            report['right_spoon_retention']['passed']=(report['right_spoon_retention']['max_fall_during_left_recovery_m']<.02
                and report['right_spoon_retention']['max_xy_drift_during_left_recovery_m']<.01)
            graspids=[e['id'] for e in recovery if (e.get('left_arm_action') or {}).get('fn')=='grasp']
            graspend=next(e['step'] for e in ends if e['edge']==graspids[0])
            low=min(range(start,graspend+1),key=lambda k:rows[k]['objects_xyz_m']['fork'][2])
            part=copy.deepcopy(rows[low:])
            for j,row in enumerate(part):row['step']=j
            part[0]['phase']='initial'
            report['after_drop_fork_motion']=assess_rows(part)['objects']['fork']
            report['recovered_carry_verified']=report['after_drop_fork_motion']['measured_carry_check']
        else:
            report['runtime_branch_verified']=(not starts and actual_tail==expected_tail
                and [e['edge'] for e in ends]==[e['id'] for e in path]
                and all(e.get('monitor_index') is None for e in ends))
            report['recovery_edges_completed']=[]
        report['measured_robot_return']=_robot_return(result['initial_robot_qpos'],result['final_robot_qpos'])
        evidence=bool(report['videos']) and all((folder/name).is_file() for name in
            ('initial.png','final.png','before_disturbance.png','after_disturbance.png'))
        report['trial_evidence_valid']=(mode_ok and report['execution_completed'] and report['process_exit_code']==0
            and report['checked_graph_executed'] and provenance and report['physical_drop_verified']
            and report['right_grip_command_checks']['passed']
            and report['native_detection_verified'] and report['runtime_branch_verified']
            and report['measured_robot_return']['passed'] and evidence
            and isinstance(report['official_function_success'],bool)
            and report['coordinate_check_passed']==report['official_function_success'])
    except (OSError,ValueError,KeyError,TypeError,IndexError,StopIteration) as exc:
        report['validation_error']=f'{type(exc).__name__}: {exc}'
    if not report['trial_evidence_valid']:status='drop_trial_incomplete_or_invalid'
    elif mode=='off':status='off_group_completed_goal_met' if report['coordinate_check_passed'] else 'off_group_completed_drop_unrecovered'
    elif not report['coordinate_check_passed']:status='recovery_final_goal_unmet'
    elif not report.get('recovered_carry_verified') or not report.get('whole_motion',{}).get('motion_checks_passed') or not report.get('right_spoon_retention',{}).get('passed'):status='recovery_motion_checks_failed'
    else:status='drop_recovery_passed_awaiting_video_review'
    report['status']=status;return report


def assess_pair(runs):
    trials={mode:assess_trial(path) for mode,path in runs.items()}
    report={'test':'finger_release_drop_off_on_pair','trials':trials,'status':'pair_incomplete',
        'matched_conditions':False,'visual_pair_success':None,'paper_reproduction_completed':False,
        'scope':'One controlled physical drop pair, native position detection, API recovery generated before execution, restored holding and nominal continuation.'}
    if set(trials)!={'off','on'}:return report
    a,b=trials['off'],trials['on']
    results=[read_json(Path(runs[m])/'result.json') for m in ('off','on')]
    graphs=[copy.deepcopy(read_json(Path(runs[m])/'agent_compiled_graph.used.json').get('task_graph',{})) for m in ('off','on')]
    for g in graphs:g.pop('task',None)
    conditions={'same_scene_config':a['scene_config_hash']==b['scene_config_hash'],
        'same_seed_and_fixed_scene':a.get('seed')==b.get('seed')==0 and a.get('stable_scene') is True and b.get('stable_scene') is True,
        'same_initial_objects':bool(results[0].get('initial_objects')) and _close(results[0].get('initial_objects'),results[1].get('initial_objects'),1e-5),
        'same_initial_joints':bool(results[0].get('initial_robot_qpos')) and _close(results[0].get('initial_robot_qpos'),results[1].get('initial_robot_qpos'),1e-4),
        'same_nominal_actions':bool(graphs[0]) and _close(*graphs,1e-5),
        'same_release_step':a.get('injection',{}).get('commanded_step')==b.get('injection',{}).get('commanded_step') and a.get('physical_drop_verified') and b.get('physical_drop_verified'),
        'same_open_finger_target':_close(a.get('injection',{}).get('requested_open_state'),b.get('injection',{}).get('requested_open_state'),1e-6),
        'same_fixed_hold_policy':a.get('hold_command_checks',{}).get('passed') is True and b.get('hold_command_checks',{}).get('passed') is True,
        'pre_release_objects_within_3_mm':bool(a.get('injection')) and _close(a.get('injection',{}).get('before_objects'),b.get('injection',{}).get('before_objects'),.003)}
    report['conditions']=conditions;report['matched_conditions']=all(conditions.values())
    if not all(t['trial_evidence_valid'] for t in trials.values()):status='pair_incomplete_or_invalid'
    elif not report['matched_conditions']:status='conditions_not_matched'
    elif b['status']!='drop_recovery_passed_awaiting_video_review':status='drop_recovery_not_validated'
    elif a['coordinate_check_passed']:status='both_goals_met_no_recovery_benefit_established'
    else:status='matched_drop_recovery_benefit_awaiting_video_review'
    report['status']=status;return report


def save_pair(pair_dir,runs,report):
    pair_dir=Path(pair_dir).resolve();console.write_json(pair_dir/'pair_report.json',report)
    target=pair_dir/'drop_validation_evidence.zip'
    names=('result.json','result.initial.json','run.log','initial.png','final.png','before_disturbance.png',
        'after_disturbance.png','plan_summary.json','plan_check.json','object_motion.jsonl',
        'agent_compiled_graph.used.json','agent_compiled_graph.drop.json','recovery_events.json','monitor_checks.json',
        'gripper_override.json','hold_commands.json','right_grip_commands.json','process_status.json','llm_recovery_generation.json','llm_recovery_calls.json',
        'agent_recovery_spec.json','agent_compiled_graph.sdk.json','agent_recovery_spec.raw_attempt_1.json','agent_recovery_spec.raw_attempt_2.json')
    with zipfile.ZipFile(target,'w',compression=zipfile.ZIP_DEFLATED) as archive:
        archive.write(pair_dir/'pair_report.json','pair_report.json')
        for mode,directory in runs.items():
            directory=Path(directory).resolve();manifest=read_json(directory/'run.json') or read_json(directory/'validation_context.json')
            public={k:manifest.get(k) for k in ('instruction','task_prompt','drop_mode','disturbance_mode','recovery','pick_carry','llm_recovery',
                'natural_language','plate_layout','seed','stable_scene','device','gpu_id','num_envs')}
            console.write_json(directory/'validation_context.json',public)
            for name in (*names,'validation_context.json','gym_config.json',*report['trials'][mode]['videos']):
                source=directory/name
                if source.is_file() and not source.is_symlink() and source.resolve().is_relative_to(directory):
                    archive.write(source,f'recovery_{mode}/{name}')
    return target


def main():
    parser=argparse.ArgumentParser(description='夹爪意外松开、真实掉落、重新抓取及继续摆盘对照')
    parser.add_argument('--project',type=Path,default=Path(__file__).resolve().parent.parent)
    parser.add_argument('--review',type=Path,help='只核验已有 pair.json 目录')
    args=parser.parse_args();runs={};pair_dir=None
    try:
        if args.review:
            pair_dir=args.review.expanduser().resolve()
            runs={k:Path(v) for k,v in read_json(pair_dir/'pair.json').get('runs',{}).items()}
            if not runs:raise RuntimeError('该目录没有 pair.json 实验索引。')
        else:
            if sys.platform!='linux':raise RuntimeError('请在 2080 的 Ubuntu 图形终端运行。')
            root,_,_,_=console.project_paths(args.project)
            pair_dir=root/'outputs/drop_validation'/dt.datetime.now().strftime('%Y%m%d_%H%M%S_%f')
            pair_dir.mkdir(parents=True,exist_ok=False)
            print('掉落 demo：抓取并抬升 → 左夹爪意外松开 → 叉子掉落 → 检测 → 重新抓取 → 继续摆盘。')
            print('先恢复关闭，再恢复开启；固定种子 0、单环境、GPU 0。')
            print('开启组由 DeepSeek 在执行前生成恢复方案，最多两次请求；自动读取已保存密钥。',flush=True)
            for mode in ('off','on'):
                print(f'\n=== 恢复 {mode} ===',flush=True)
                run,code=console.run_command(root,'整理桌子',False,[640,480],False,no_proxy=True,seed=0,
                    stable_scene=True,grasp_check=True,natural_language=False,plate_layout=False,
                    drop_mode=mode,llm_recovery=mode=='on')
                runs[mode]=run;console.write_json(pair_dir/'pair.json',{'runs':{k:str(v) for k,v in runs.items()}})
                trial=assess_trial(run)
                if code!=0 or not trial['trial_evidence_valid']:
                    print('本组未形成有效掉落证据，停止并保存诊断。',flush=True);break
        report=assess_pair(runs);target=save_pair(pair_dir,runs,report)
        print('\n=== 掉落恢复对照 ===')
        for mode,t in report['trials'].items():
            print(f"{mode}：实际掉落 {t['physical_drop_verified']}；检测 {t['native_detection_verified']}；官方最终判定 {t['official_function_success']}；状态 {t['status']}")
        print('条件匹配：',report['matched_conditions']);print('对照状态：',report['status'])
        print('录像尚待核验；请传回证据包：',target)
        return 0 if report['status']=='matched_drop_recovery_benefit_awaiting_video_review' else 2
    except (EOFError,KeyboardInterrupt):print('\n对照已停止。');return 130
    except Exception as exc:
        print(f'无法完成对照：{type(exc).__name__}: {exc}')
        if pair_dir is not None and runs:
            try:
                report=assess_pair(runs);report['runner_error']=f'{type(exc).__name__}: {exc}'
                print('诊断包：',save_pair(pair_dir,runs,report))
            except Exception:print('已产生的实验目录：',{k:str(v) for k,v in runs.items()})
        return 1


if __name__=='__main__':raise SystemExit(main())
