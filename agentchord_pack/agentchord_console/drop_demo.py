"""Controlled gripper release and native monitored recovery before carrying.

The local disturbance changes only finger commands. It never sets object poses:
falling and later re-lifting must be measured in the live physics trace.
"""
from __future__ import annotations
import copy
import json
import math
from pathlib import Path
from types import SimpleNamespace
from functools import partial
from agentchord_console.carry_demo import select_bundle
from agentchord_console.disturbance_demo import (RecoveryObserver, with_reset_targets,
    native_action_kwargs, _HypotheticalSim, _as_like, assess_predicted_robot_return)
from agentchord_console.plan_check import ShadowEnv, clone, serializable, evaluate_plan, IKFailure
from agentchord_console.task_plan import _pose_xyz, _hash
from agentchord_console.llm_recovery import (AuditedLLM, RecoveryContractError, _require,
    validate_spec, validate_compiled, prediction_feedback)

EDGE='e03b_hold'
EDGE_STEP=5
HOLD_STEPS=50
OPEN_STEPS=20
THRESHOLD=.02
CONTRACT='gripper_release_drop_v2'
HOLD_POLICY='stationary_native_target_50_v2'
RIGHT_GRIP_POLICY='retain_right_closed_target_v3'
RIGHT_CARRY_EDGES=('e04_carry','e05_lower')


def fixed_hold_trajectory(rows,seed_qpos,closed_target):
    """Keep the native stationary target for 50 steps even if TOPPRA returns two.

    Only the hold edge uses this local duration adapter. It accepts a stationary
    native Nx8 trajectory, never creates a new arm target or skips native IK.
    """
    import numpy as np
    raw=np.asarray(serializable(rows),dtype=np.float64)
    seed=np.asarray(serializable(seed_qpos),dtype=np.float64).reshape(-1)
    closed=np.asarray(serializable(closed_target),dtype=np.float64).reshape(-1)
    if raw.ndim!=2 or not 1<=len(raw)<=HOLD_STEPS or raw.shape[1]!=8 or seed.shape!=(6,) or closed.shape!=(1,):
        raise RuntimeError('Unexpected native stationary hold trajectory shape.')
    if not (np.isfinite(raw).all() and np.isfinite(seed).all() and np.isfinite(closed).all()):
        raise RuntimeError('Non-finite stationary hold command.')
    drift=float(np.max(np.abs(raw[:,:6]-seed)))
    excursion=float(np.max(np.abs(raw-raw[-1])))
    if drift>1e-4 or excursion>1e-4 or np.max(np.abs(raw[:,6:]-closed[0]))>1e-6:
        raise RuntimeError('Hold target changes arm joints or releases fingers; fixed-duration hold stopped.')
    fixed=np.repeat(raw[-1:],HOLD_STEPS,axis=0)
    audit={'policy':HOLD_POLICY,'status':'passed','raw_trajectory':raw.tolist(),
        'seed_qpos':seed.tolist(),'closed_target':closed.tolist(),'fixed_trajectory':fixed.tolist(),
        'native_steps':len(raw),'fixed_steps':HOLD_STEPS,'max_joint_change_rad':drift,
        'max_native_row_excursion':excursion,'note':'Only repeat the native stationary terminal joint command; original drive advances each physics step.'}
    return fixed,audit


def nominal_bundle(env, *, select=None):
    bundle,check=(select or select_bundle)(env)
    if bundle is None:return None,check
    bundle=with_reset_targets(bundle,env)
    task=bundle['task_graph'];lift=task['edges'][2]
    hold=copy.deepcopy(lift)
    hold.update(id=EDGE,source=lift['target'],target='v3b_hold')
    for arm in ('left_arm','right_arm'):hold[f'{arm}_action']['kwargs']['sample_num']=HOLD_STEPS
    task['nodes'].insert(4,{'id':'v3b_hold','semantic':'Both objects held at their lifted poses; continue nominal carrying.'})
    task['edges'][3]['source']=hold['target'];task['edges'].insert(3,hold)
    task['task']='ControlledDrop_'+_hash(task)[:16]
    for key in ('recovery_spec','recovery_graph'):bundle[key]['task']=task['task']
    bundle['metadata'].update(task_graph_hash=_hash(task),drop_recovery_contract=CONTRACT,
        right_grip_policy=RIGHT_GRIP_POLICY,
        console_scope='Controlled finger-release drop; native position monitor and preplanned API recovery.',
        drop={'edge':EDGE,'edge_step':EDGE_STEP,'open_steps':OPEN_STEPS,
              'method':'override only left finger commands to native open_state',
              'protocol':CONTRACT,'object_pose_setter_calls':0,'hold_policy':HOLD_POLICY,'hold_steps':HOLD_STEPS})
    full=evaluate_plan(bundle,env)
    report={'status':full['status'],'carry_selection':check,'nominal_plan_check':full}
    return (bundle if full['status']=='passed' else None),report


def branch_goal(state,held):
    def values(value,size):
        v=serializable(value)
        if isinstance(v,list) and len(v)==1 and isinstance(v[0],list):v=v[0]
        if not isinstance(v,list) or len(v)!=size or any(isinstance(x,bool) or
            not isinstance(x,(int,float)) or not math.isfinite(x) for x in v):
            raise ValueError('Invalid predicted held joint/gripper vector.')
        return v
    def pose(value):
        p=serializable(value)
        if not isinstance(p,list) or len(p)!=4 or any(not isinstance(row,list) or len(row)!=4 for row in p):
            raise ValueError('Invalid held pose matrix.')
        for row in p:values(row,4)
        return p
    current=pose(state.left_arm_current_xpos);target=pose(held.left_arm_current_xpos)
    xyz=math.dist([r[3] for r in current[:3]],[r[3] for r in target[:3]])
    rot=max(abs(current[i][j]-target[i][j]) for i in range(3) for j in range(3))
    right=values(state.right_arm_current_qpos,6);right_target=values(held.right_arm_current_qpos,6)
    grippers={side:values(getattr(state,f'{side}_arm_current_gripper_state'),1) for side in ('left','right')}
    close=values(held.close_state,1)
    joint=max(abs(a-b) for a,b in zip(right,right_target))
    grip=max(abs(g[0]-close[0]) for g in grippers.values())
    return {'passed':xyz<=.002 and rot<=.002 and joint<=1e-5 and grip<=.002,
        'left_pose':current,'left_pose_target':target,'left_translation_error_m':xyz,'left_rotation_max_error':rot,
        'right_qpos':right,'right_qpos_target':right_target,'right_joint_error_rad':joint,
        'grippers':grippers,'closed_target':close,'max_gripper_error_m':grip,
        'scope':'Cloned held-state goal only; physical object retention and recovery require measured live motion.'}


def prediction(env,nominal,edges,actions_module=None):
    if actions_module is None:
        from embodichain.agents.agentchord import atom_actions as actions_module
    report={'status':'error','attempts':[{'edges':[],'plan_source':'model_selected_fixed_plan'}],
        'scope':'read_only_drop_regrasp_and_remaining_nominal_trajectory',
        'assumptions':'Fork lands near its initial XYZ and rotation; left fingers are open. Right spoon stays held. Contact is not predicted.'}
    attempt=report['attempts'][0]
    shadow=None
    def generate(state,edge,record):
        stage={'id':edge['id'],'arms':[]};record.append(stage)
        for side in ('left','right'):
            action=edge.get(f'{side}_arm_action')
            if action is None:continue
            state.active_edge,state.active_arm,state.active_action=edge['id'],f'{side}_arm',action['fn']
            seed=clone(getattr(state,f'{side}_arm_current_qpos'))
            rows=getattr(actions_module,action['fn'])(env=state,**native_action_kwargs(state,action))
            details={'arm':f'{side}_arm','trajectory_steps':len(rows)}
            if edge['id']==EDGE:
                rows,hold=fixed_hold_trajectory(rows,seed,state.close_state)
                details.update(trajectory_steps=len(rows),stationary_hold=hold)
            stage['arms'].append(details)
    try:
        shadow=ShadowEnv(env);before=[]
        path=nominal['task_graph']['edges'];position=next(i for i,e in enumerate(path) if e['id']==EDGE)
        for edge in path[:position+1]:generate(shadow,edge,before)
        attempt['nominal_prefix']=before
        held=ShadowEnv(shadow)
        shadow.set_current_gripper_state_agent(clone(shadow.open_state),is_left=True)
        pose=clone(env.sim.get_rigid_object('fork').get_local_pose(to_matrix=True))
        shadow.sim=_HypotheticalSim(shadow.sim,{'fork':pose})
        matrix=pose.squeeze(0) if hasattr(pose,'squeeze') else (pose[0] if len(pose)==1 else pose)
        shadow.obj_info['fork']['pose']=clone(matrix)
        for edge in edges:generate(shadow,edge,attempt['edges'])
        attempt['held_branch_goal']=branch_goal(shadow,held)
        if not attempt['held_branch_goal']['passed']:
            attempt.update(status='failed',reason='Recovery must restore fork holding at the lifted nominal pose and leave right spoon held.')
            report['status']='failed';return report
        tail=[]
        for edge in path[position+1:]:generate(shadow,edge,tail)
        attempt['remaining_nominal_edges']=tail
        attempt['predicted_robot_return']=assess_predicted_robot_return(shadow,env)
        if not attempt['predicted_robot_return']['passed']:
            attempt.update(status='failed',reason='Remaining nominal tail fails the dual-arm final joint/open-gripper goal.')
            report['status']='failed';return report
        steps=sum(max(a['trajectory_steps'] for a in e['arms']) for e in before+attempt['edges']+tail)+30
        _require(steps<=1100,'Drop recovery exceeds the step budget.')
        attempt.update(status='passed',predicted_max_steps=steps);report['status']='passed'
    except IKFailure as exc:
        report['status']='failed';attempt.update(status='failed',reason=str(exc),failed_edge=shadow.active_edge)
    except Exception as exc:
        report.update(status='error',error=f'{type(exc).__name__}: {exc}')
    finally:
        attempt['ik_calls']=copy.deepcopy(shadow.ik_calls) if shadow is not None else []
    return report


def checked_edges(bundle,nominal):
    edges=validate_compiled(bundle,nominal,binding_edge=EDGE)
    _require(all(e.get('right_arm_action') is None for e in edges),'Right spoon remains held: recovery must leave right_arm idle.')
    _require((edges[-1].get('left_arm_action') or {}).get('fn')=='move_relative_to_object',
        'Finish recovery by lifting the regrasped fork to its nominal hover pose; nominal carry/release/return will then resume.')
    return edges


def context(nominal):
    hover=next(e for e in nominal['task_graph']['edges'] if e['id']==EDGE)['left_arm_action']['kwargs']
    return ('Controlled drop by opening the LEFT gripper while both cutlery objects are held above the table. '
        'The fork falls under gravity; the right spoon remains held. Bind exactly e03b_hold, object_moved on '
        'fork threshold 0.02, merge=target, repeat_until_success=false. Use official lightweight recovery_bindings JSON. '
        'Recover only left_arm/fork. All right_arm_action fields must be null: do not move or release the held spoon. '
        'After recovery the original nominal carry, lower, release and dual-arm return edges continue. '
        'Therefore branch goal is fork held at the original lifted pose, with both grippers CLOSED, not a released '
        'fork or both arms returned. Do not replay nominal dual-arm edges in this branch. Choose allowed native '
        'grasp/regrasp, orient_eef(down), move_relative_to_object, move_to_target_pose, open_gripper or back_to_initial_pose '
        'as intermediate skills; force_valid must remain false. End with left move_relative_to_object to the hover '
        'target below. Original grasp can retract from a lowered arm; a measured initial-pose reset is available '
        'before regrasp. Never move an object by setting its pose. At most 12 steps, approaches 0.03–0.10 m, '
        'sample_num 10–50. Right gripper is not affected by the disturbance.\n'
        'Exact left hover kwargs:\n'+json.dumps(hover)+'\nMeasured initial reset matrices:\n'+
        json.dumps(nominal['metadata']['recovery_reset_targets'])+'\nInitial object XYZ:\n'+json.dumps(nominal['metadata']['initial_xyz'])+
        '\nRoot fields task/recovery_bindings. Binding fields edge_id/failure_name/monitors/recovery/merge/repeat_until_success. '
        'Recovery forms action or regrasp. Output JSON only.')


def prepare_bundle(env,run_dir,enabled,*,write_json,model,base_url,select=None,predictor=None,progress=None):
    from embodichain.agents.agentchord import graph_spec
    nominal,check=nominal_bundle(env,select=select)
    report={'status':check['status'],'nominal':check,'attempts':[],'drop_contract_version':CONTRACT,
        'generation_phase':'before_execution','local_recovery_fallback':False}
    if nominal is None or not enabled:return nominal,report
    folder=Path(run_dir);agent,compiler=env.recovery_agent,env.compile_agent
    old_agent,old_compiler=agent.llm,compiler.llm
    audit=AuditedLLM(old_agent,folder/'llm_recovery_calls.json',write_json,model,base_url)
    agent.llm=compiler.llm=audit;feedback=''
    try:
        for index in (1,2):
            attempt={'index':index,'status':'started'};report['attempts'].append(attempt)
            directory=folder/'llm_generation'/f'attempt_{index}';directory.mkdir(parents=True,exist_ok=False)
            if progress:progress(f'DeepSeek 生成掉落恢复方案（{index}/2）；随后检查重新抓取和剩余摆盘。')
            inputs=agent.get_composed_observations(task_graph=json.dumps(nominal['task_graph']),
                task_prompt='Place fork at plate world Y+0.16 and spoon at Y-0.16, release and return both arms. Recover a dropped fork before continuing the carry.',
                log_dir=directory,regenerate=True)
            inputs['recovery_rules']=str(inputs.get('recovery_rules',''))+'\n'+context(nominal)+feedback
            try:
                raw=json.loads(agent.generate(**inputs));write_json(folder/f'agent_recovery_spec.raw_attempt_{index}.json',raw)
                canonical=validate_spec(raw,nominal,graph_spec,binding_edge=EDGE)
                path,_,_=compiler.generate(log_dir=directory,regenerate=True,task_graph=nominal['task_graph'],recovery_spec=raw,recovery_enabled=True)
                bundle=json.loads(Path(path).read_text(encoding='utf-8'))
                _require(bundle['recovery_spec']==canonical,'Native compiler changed the checked spec.')
                edges=checked_edges(bundle,nominal)
                graph_spec.compile_agent_graph_spec(bundle['task_graph'],bundle['recovery_graph'],env=env)
                predicted=(predictor or prediction)(env,nominal,edges)
                attempt['recovery_prediction']=predicted
                if predicted['status']=='error':raise RuntimeError('Native drop preflight error; see diagnostics.')
                _require(predicted['status']=='passed','Drop recovery prediction failed: '+prediction_feedback(predicted))
                _require(predicted['attempts'][-1]['held_branch_goal']['passed'] is True
                    and predicted['attempts'][-1]['predicted_robot_return']['passed'] is True,'Held branch and final return both need verification.')
                write_json(folder/'agent_compiled_graph.sdk.json',bundle)
                write_json(folder/'agent_recovery_spec.json',canonical)
                bundle['metadata'].update(copy.deepcopy(nominal['metadata']))
                bundle['metadata'].update(recovery_enabled=True,llm_recovery_generated=True,generation_phase='before_execution',
                    recovery_author='native RecoveryAgent via API and native CompileAgent')
                attempt['status']='passed';report.update(status='passed',selected_attempt=index,recovery_prediction=predicted,
                    api_invocations=len(audit.calls),model=model,base_url=base_url,raw_spec_hash=_hash(raw),
                    canonical_spec_hash=_hash(canonical),compiled_recovery_hash=_hash(bundle['recovery_graph']))
                return bundle,report
            except (RecoveryContractError,ValueError) as exc:
                attempt.update(status='rejected',reason=str(exc))
                if not audit.calls or audit.calls[-1]['status']!='returned':raise
                feedback='\nPrevious output rejected. Preserve successful reset/regrasp steps and repair the failing route.\n'+audit.calls[-1]['content']+'\nDiagnostic:\n'+str(exc)
                if progress:progress('方案未通过；允许模型修正一次。' if index==1 else '两次方案均未通过，停止本轮。')
        report.update(status='failed',api_invocations=len(audit.calls))
    except Exception as exc:
        report.update(status='error',error_type=type(exc).__name__,api_invocations=len(audit.calls))
    finally:
        agent.llm,compiler.llm=old_agent,old_compiler
        write_json(folder/'llm_recovery_generation.json',report)
    return None,report


class DropObserver(RecoveryObserver):
    def __init__(self,*args,**kwargs):
        super().__init__(*args,**kwargs)
        self.reference=None;self.before=None;self.initial_xyz=None;self.drop_observed=False
        self.override_rows=[];self.original_step=None;self.hold_commands={}
        self.right_active=False;self.right_edges=set();self.right_rows=[];self.right_events=[]
    def begin(self,edge):
        if self.edge==edge.id:return
        super().begin(edge)
        if edge.id==EDGE:
            self.reference=clone(self.env.sim.get_rigid_object('fork').get_local_pose(to_matrix=True).squeeze(0))
        if self.enabled and edge.is_recovery and not self.right_active:
            if not self.injected or not self.detected or edge.id not in self.right_edges:
                raise RuntimeError('Right grip retention must start in the checked post-detection recovery branch.')
            self.right_active=True
            entry={'event':'activate','step':self.trace.step_index,'edge':edge.id,
                'cached_right_gripper_state':serializable(self.env.right_arm_current_gripper_state),
                'measured_robot_qpos':serializable(self.env.robot.get_qpos()),
                'target_m':self.hold_commands['right']['closed_target'][0]}
            self.right_events.append(entry)
            self.emit('right_grip_retention_started',policy=RIGHT_GRIP_POLICY,**{k:v for k,v in entry.items() if k!='event'})
        elif self.right_active and edge.id=='e06_release':
            self.right_active=False
            self.right_events.append({'event':'deactivate','step':self.trace.step_index,'edge':edge.id})
            self.emit('right_grip_retention_ended',policy=RIGHT_GRIP_POLICY)
    def step_command(self,action):
        # This is a local command adapter, separate from the intentional left
        # release. Native measured caches and the model graph stay unchanged.
        command=self.release_left(action)
        preserve=self.right_active and self.edge in self.right_edges
        if self.right_active and not preserve:
            raise RuntimeError('Unexpected edge while retaining the right spoon; stop rather than conceal a release.')
        if preserve:
            if serializable(self.env.right_eef_joints)!=[14,15] or tuple(command.shape)!=(1,16):
                raise RuntimeError('Unexpected native right finger indices or action shape.')
            target=float(self.hold_commands['right']['closed_target'][0])
            if not math.isfinite(target) or abs(target-float(serializable(self.env.close_state)[0]))>1e-6:
                raise RuntimeError('Right retention target must be the original verified closed target.')
            command=clone(command);command[:,[14,15]]=target
        self.right_rows.append({'step':self.trace.step_index+1,'edge':self.edge,
            'phase':self.trace.phase,'edge_step':self.edge_step+1,'right_preserved':preserve,
            'cached_right_gripper_state':serializable(self.env.right_arm_current_gripper_state),
            'original_action':serializable(action),'executed_action':serializable(command)})
        return command
    def release_left(self,action):
        if self.edge!=EDGE or not EDGE_STEP<=self.edge_step+1<EDGE_STEP+OPEN_STEPS:return action
        if self.reference is None:raise RuntimeError('Drop reference missing.')
        if serializable(self.env.left_eef_joints)!=[12,13] or tuple(action.shape)!=(1,16):
            raise RuntimeError('Unexpected native finger indices or action shape; drop stopped.')
        before={n:_pose_xyz(self.env,n) for n in ('plate','fork','spoon')}
        if not self.injected:
            self.initial_xyz=before['fork']
            # Height is compared with this run's recorded initial scene, not the table's visual mesh.
            result=json.loads((self.run_dir/'result.initial.json').read_text(encoding='utf-8'))
            if before['fork'][2]-result['fork']['xyz'][2]<.02:
                raise RuntimeError('Fork did not lift at least 2 cm before release; stop comparison.')
            self.snapshot('before_disturbance.png');self.before=before;self.injected=True
            self.emit('injection',object='fork',protocol=CONTRACT,edge_step=self.edge_step+1,
                commanded_step=self.trace.step_index+1,before_objects=before,
                before_robot_qpos=serializable(self.env.robot.get_qpos()),finger_indices=[12,13],
                requested_open_state=serializable(self.env.open_state),object_pose_setter_calls=0)
        command=clone(action);target=float(serializable(self.env.open_state)[0]);command[:,[12,13]]=target
        self.override_rows.append({'step':self.trace.step_index+1,'edge':self.edge,'edge_step':self.edge_step+1,
            'original_action':serializable(action),'executed_action':serializable(command),'finger_target_m':target})
        self.write_json(self.run_dir/'gripper_override.json',{'protocol':CONTRACT,'commands':self.override_rows})
        return command
    def check(self,native_monitor,*,original=None):
        if self.edge!=EDGE:raise RuntimeError('Drop monitor bound outside the hold edge.')
        if original is None:
            from embodichain.agents.agentchord.monitor_functions import monitor_object_moved
            original=lambda:monitor_object_moved(env=self.env,obj_name='fork',last_frame_pose=self.reference,threshold=THRESHOLD)
        else:
            # Native drive refreshes obj_info every step. Use the native monitor's
            # explicit pose argument so a slow fall is measured against hold start,
            # rather than comparing only adjacent frames.
            native_call=original
            original=lambda:native_call(last_frame_pose=self.reference)
        verdict=super().check(None,original=original)
        self.checks[-1].update(reference_mode='stationary_hold_start',reference_pose=serializable(self.reference))
        self.write_json(self.run_dir/'monitor_checks.json',self.checks)
        if verdict and not self.injected:raise RuntimeError('Fork moved before the release injection; comparison stopped.')
        return verdict
    def after_step(self,trace):
        if trace.step_index>1100:raise RuntimeError('Drop demo exceeds step budget.')
        if self.edge is None:return
        self.edge_step+=1
        if self.injected and not self.drop_observed:
            xyz=_pose_xyz(self.env,'fork')
            if self.before['fork'][2]-xyz[2]>=.02:
                self.drop_observed=True
                self.emit('drop_observed',object='fork',xyz_m=xyz,fall_m=self.before['fork'][2]-xyz[2],
                    measured_robot_qpos=serializable(self.env.robot.get_qpos()))
                self.snapshot('after_disturbance.png')
        if not self.enabled and self.edge==EDGE:self.check(None)
    def attach(self):
        super().attach()
        # The original planner may collapse a zero-distance move to two samples,
        # irrespective of sample_num. Adapt only this compiled stationary edge.
        observed_compile=self.spec_module.compile_agent_graph_from_file
        def compile_with_fixed_hold(*args,**kwargs):
            graph=observed_compile(*args,**kwargs)
            self.right_edges={e.id for e in graph.edges.values() if e.is_recovery}
            if any(graph.edges[e].right_arm_action is not None for e in self.right_edges):
                raise RuntimeError('Right grip adapter requires the checked left-only recovery branch.')
            for edge_id in RIGHT_CARRY_EDGES:
                if edge_id not in graph.edges:raise RuntimeError('Nominal carry/lower edge missing.')
                self.right_edges.add(edge_id)
            edge=graph.edges[EDGE]
            for side in ('left','right'):
                original_action=getattr(edge,f'{side}_arm_action')
                if original_action is None:raise RuntimeError('Both arms must have a stationary hold target.')
                def stationary(*args,_side=side,_action=original_action,**kwargs):
                    seed=clone(getattr(self.env,f'{_side}_arm_current_qpos'))
                    rows=_action(*args,**kwargs)
                    fixed,audit=fixed_hold_trajectory(rows,seed,self.env.close_state)
                    self.hold_commands[_side]=audit
                    self.write_json(self.run_dir/'hold_commands.json',{'policy':HOLD_POLICY,'arms':self.hold_commands})
                    self.emit('hold_command_prepared',arm=f'{_side}_arm',native_steps=len(rows),fixed_steps=len(fixed),policy=HOLD_POLICY)
                    return fixed
                setattr(edge,f'{side}_arm_action',partial(stationary))
            return graph
        self.spec_module.compile_agent_graph_from_file=compile_with_fixed_hold
        self.original_step=self.env.step
        original=self.original_step
        def released_step(action,*args,**kwargs):return original(self.step_command(action),*args,**kwargs)
        self.env.step=released_step
        return self
    def close(self):
        if self.original_step is not None:self.env.step=self.original_step
        try:
            self.write_json(self.run_dir/'right_grip_commands.json',{'policy':RIGHT_GRIP_POLICY,
                'enabled':self.enabled,'finger_indices':[14,15],
                'protected_edges':sorted(self.right_edges),'events':self.right_events,'commands':self.right_rows,
                'target_source':'Original native stationary hold closed target; measured caches are not overwritten.',
                'object_pose_setter_calls':0})
        finally:super().close()
