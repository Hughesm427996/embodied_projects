# AgentChord 自然语言桌面整理控制台

## 新 demo：夹爪意外松开、叉子掉落后重新抓取

修复包：agentchord_console_drop_v4.zip。把它传到 Ubuntu 已跑通项目的 tests 目录，退出其他仿真后执行：

    cd "$HOME/embodied_projects/agentchord_pack"
    python3 -m zipfile -e tests/agentchord_console_drop_v4.zip .
    bash agentchord_console/validate_drop.sh

自动跑恢复关闭和开启两组，种子 0、固定初始场景、单环境、GPU 0。流程是双臂抓取、抬升、短暂停住；在悬停边第 5 步把左夹爪的两个手指指令改为原 SDK 的打开值，最多持续 20 步。叉子由物理仿真掉落，右臂继续拿着勺子。开启组检测到掉落后会中断悬停，因此实际打开指令可能少于 20 步；不会在恢复分支重复注入。关闭组继续执行原搬运、释放、回位动作，以观察丢失餐具的后果。

v1 在 2026-10-11 11:50 的真实测试中只完成普通摆盘：原规划器对零位移悬停只返回两帧，未到第 5 步，因此没有注入、检测或开启组。v2 仅对悬停边增加固定时长适配：先调用原技能和 IK，确认生成的 8 列关节轨迹是静止的、夹爪仍闭合，重复该原生末帧至 50 步，再交给原 drive 逐步执行。关节变化超过 0.0001 弧度或夹爪状态不符就停止；不会新造目标或跳过 IK。hold_commands.json 保存两臂的原轨迹、种子关节、固定轨迹和长度，并与松爪前的真实执行指令核对。只读预测使用同一时长规则。

v2 的 2026-10-11 12:14 实测已经出现真实掉落、原监测器触发、DeepSeek 生成的四步左臂恢复及叉子继续摆盘，叉子终点 Y 误差约 0.24 毫米。但恢复开始时勺子也掉落，终点 Y 误差约 46.67 毫米，整轮仍是失败。原 SDK 在监测器触发后同步实际机器人状态，此时右指接触宽度约 9.67/11.60 毫米；后续右臂 idle 把第一指的实际宽度作为两指目标。源码回归验证了这种指令变化；它与这次滑落的时间吻合，实际接触因果仍由下轮仿真验证。

v3 保留 v2 的扰动和原生恢复图，增加公开的本地右指保持适配。只在检测后进入左臂恢复分支、以及紧接的名义搬运和降低边，把右指 14/15 的命令继续设为悬停原轨迹已验证的闭合目标。其余 14 列命令完全不变；原同步函数、实际宽度缓存、六个右臂关节目标和模型图均不改。在名义释放边开始前关闭适配，让原打开、回位动作照常执行。关闭组不启用该适配。right_grip_commands.json 记录所有 env.step 的原命令、执行命令、同步后的缓存及启停位置；验证器检查完整覆盖、仅两个右指改变、缓存保留和释放恢复。整轮物理仿真仍逐步调用原 drive，没有物体位姿设置。这个适配属于本地兼容修复，不能称为原 SDK 完全无修改地成功。

v3 的 20261011_133445 测试正常完成关闭组：第 130 步松爪，第 131 步叉子实际下落约 3.13 厘米并被检测，勺子正常放到目标，双臂回位合格。开启组未启动，因为验证器误要求释放边最后一帧的夹爪命令等于初始实际打开位置。原 SDK 的 mul_linear_expand 使用 alpha=k/sample_num，15 帧打开轨迹最后一帧为 14/15×0.05≈0.04667 米；下一条原回位动作继续发送 0.05 米。v4 修正这项命令检查：按原插值规则重新计算整段右指打开轨迹、原 drive 的补齐规则和后续回位命令，也保留原 open_gripper 已打开时的跳过规则。实际回位关节/夹爪检查、掉落及重新抬升、餐具终点和录像标准保持原样。v4 的物理执行代码与 v3 相同，右夹爪保持修复仍需开启组实测。

插值来源：[固定版本原 SDK misc.py](https://github.com/EDEM-AI/AgentChord/blob/74fe7dd756964b737952d26e9e65d4d5e2e29ec3/embodichain/lab/gym/utils/misc.py#L73)。本地回归直接使用该固定版本函数核对排除末端的采样，并验证缺少释放、修改末帧或混用初始实测位置会被正确区分。

这是本地添加的夹爪指令扰动，使用原机器人关节布局（左指 12/13），其余 14 个关节指令不变。没有调用物体位姿设置器，也没有使用原 fallen_object 的传送/旋转注入。原 monitor_object_moved 位置监测器使用其 last_frame_pose 参数，与本轮悬停开始的实际叉子姿态比较，累计位移超过 2 厘米才检测；它使用仿真物体状态。这一轮没有测试视觉检测或原 monitor_object_held。

开启组由原 RecoveryAgent 调用 DeepSeek 在执行前生成恢复分支，再由原 CompileAgent 编译。最多两次 API 请求；使用已保存密钥。模型只能操作左臂，先重新抓取和朝下定向，再恢复到原抬升位置，保持叉子拿稳、两夹爪关闭及右臂不动；之后接着执行原名义搬运、降低、释放、双臂返回。不会把叉子掉落后的恢复终点误设成“打开夹爪并结束整个任务”。生成或完整 IK/轨迹预检不通过时停止，不追加本地恢复模板。

只读预检假设叉子落回初始位置附近，不能预测接触、实际落点或旋转；真实抓取以本轮测量和录像为准。脚本要求注入前叉子已实际升高至少 2 厘米、打开后实际下落至少 2 厘米，并保存原始及执行的手指指令、逐步物体位置、检测及图切换记录。开启组还要求落下之后重新连续抬升至少 5 步、在腾空期间移动至少 2 厘米，再检查餐具终点 X/Y/Z、30 步静置稳定性和双臂实际关节回位。原官方 Y-only 指标另行记录。

结果在 outputs/drop_validation/时间/，证据包名 drop_validation_evidence.zip。成功状态 matched_drop_recovery_benefit_awaiting_video_review 表示上述坐标、动作图、API 来源和对照检查通过；传回 ZIP 后还要核验真实掉落、重新夹持和继续摆盘的录像。失败时同样会保存诊断包。这是新增的一次受控 demo；不等于论文成功率或全部仿真复现。Windows 本地测试仅验证接口、原图运行和检查逻辑，尚需在 2080 执行新 demo。

复核已有实验、不再启动仿真：

    bash agentchord_console/validate_drop.sh --review outputs/drop_validation/实验目录名

此前的 validate_llm_recovery.sh、validate_disturbance.sh、validate_carry.sh 继续保留，普通自然语言控制台仍从 start.sh 启动。新 ZIP 不包含密钥、实验结果或 SDK 安装文件。

直接用自然语言描述目标，由 DeepSeek 解释对象、位置及“不动某件餐具”等限制。本地程序将校验后的目标编成已支持的机器人技能，检查完整动作后，交给原 AgentChord 图运行器和 DexSim 执行。无需记固定任务指令。

## 安装与启动

将 agentchord_console_drop_v4.zip 传到 Ubuntu 的项目 tests 目录：

    /home/xiaoyu/embodied_projects/agentchord_pack/tests

退出旧控制台，然后执行：

    cd "$HOME/embodied_projects/agentchord_pack"
    python3 -m zipfile -e tests/agentchord_console_drop_v4.zip .
    bash agentchord_console/start.sh --no-proxy --stable-scene --seed 0

使用已经跑通的 .venv、原项目代码、机器人资源及此前 CUDA 修复。没有新增依赖。默认物理 GPU 0、640×480 录像、种子 0。--stable-scene 关闭初始餐具及机械臂姿态随机化，方便先验证演示。--no-proxy 仅让本轮子进程忽略代理环境变量，需机器可以直连 DeepSeek。

DeepSeek API 密钥可以只配置一次。运行下列命令，在提示后粘贴密钥并回车；输入不会显示，也不会进入 shell 命令历史：

    bash agentchord_console/start.sh --save-api-key

保存成功后，再按上面的普通命令启动。密钥单独保存为 ~/.config/agentchord_console/deepseek_api_key（设置了绝对路径 XDG_CONFIG_HOME 时使用该目录），配置目录权限 700、文件权限 600。该文件是本机明文配置，仅当前用户可读写，不在项目、日志或脚本 ZIP 内。不要将它作为实验结果发送。更新密钥时重复配置命令；取消保存时删除该文件。

读取顺序为 DEEPSEEK_API_KEY 环境变量、OPENAI_API_KEY 环境变量、本地配置文件。如果三者都没有，仍会隐藏询问，且仅保留在本次启动期间。不需要把密钥写进 .bashrc。默认使用已测试的 deepseek-flash、https://api.deepseek.com。

## 官方 RecoveryAgent 调用 DeepSeek 的下一轮验证

此前 v4 已在 2080 完成一组受控物体移位恢复对照，并核验录像：关闭组目标未达成，开启组检测移位后完成重新抓取、抬升、搬回、释放及回位。v4 的恢复图是本地显式编排。新模式单独验证恢复方案的 API 生成来源，保留既有场景、名义动作、注入协议和兼容修复。

API 恢复 v1 的 20261011_092606 实测中，DeepSeek 一次调用生成了恢复图；原监测器检测到移位，叉子被重新抓取、抬升约 5.9 厘米并搬回，最终原场景判定为 True。但完整测试未通过：模型只安排左臂回位，右臂最大关节偏差为 0.664 弧度，录像也显示右臂停留。merge=target 结束被中断的返回边，不会替未提及的手臂继续剩余轨迹。v1 只检查轨迹可生成，漏掉了执行前的双臂终点检查；该证据仍按失败保留。

v2 增加了双臂回位检查，但把双臂最后技能限定为原 back_to_initial_pose。20261011_103558 的实测中，第一次模型方案加入了右臂回位，却从被中断的退让终点再退让 8 厘米；右臂预测目标 Z=0.939746 米，原 IK 返回 false。第二次方案删除了已通过的左臂复位，原 grasp 从中断姿态自动退让时也不可达。两次都被执行前预检拒绝，开启组没有执行恢复动作或生成视频；不能作为新一轮恢复成功。

v3 保留双臂终点标准，允许模型选择原 back_to_initial_pose 或原 move_to_target_pose 回到本轮记录的初始矩阵。早先的初始位姿复位也可作为该臂的最终回位，只要之后没有动作改变其姿态或关节。不会自行追加动作或改用本地模板。只读预测完成后，仍检查双臂缓存关节是否距各自初始目标不超过 0.01 弧度、夹爪是否距打开目标不超过 0.002 米；末端矩阵相同不能替代关节回位。运行后的实际机器人回位检查保持不变。旧 v2 证据仍按当时的规则核验，旧失败不会改判为通过。

反馈给模型的诊断包含失败手臂、原技能、不可达目标和已生成的动作前缀，并说明保留已通过的复位，避免重复退让。最多允许模型修改一次，实际轨迹必须重新预检。v3 的本地回归覆盖漏掉右臂、初始矩阵回位、矩阵匹配但关节不匹配、原位姿技能不额外退让、模型修正与失败停止；这些测试不代替真实 CUDA 或物理仿真。v3 仍需在 2080 重跑并核验新录像。

退出其他仿真控制台，解压本包后在 Ubuntu 图形终端运行：

    bash agentchord_console/validate_llm_recovery.sh

程序自动先跑恢复关闭组，再跑开启组。开启组使用原 RecoveryAgent.generate() 和原 RecoveryPrompt，通过 DeepSeek 生成官方 recovery_bindings JSON；原 CompileAgent.generate() 归一化、展开该方案，再用原图校验器和只读 IK/轨迹预检检查。实际执行时，原位置监测器检测到叉子移位，原图运行器切换到生成的恢复分支。动作图只执行一次。

这符合固定版本中执行前生成恢复方案的流程。当前名义整理图仍是已验证的本地受控图；输入给模型的是已知扰动类型、目标、可用技能、实测初始回位矩阵和约束，模型选择恢复步骤及参数。它不是检测发生后重新调用大模型，不是视觉模型检测，也不是全部仿真复现。

最多两次 RecoveryAgent API 调用：首个方案因 schema、双臂回位遗漏、预测 IK 不可达或预测终点不合格被拒绝时，将公开诊断反馈给模型，允许修改一次。不开启 HTTP 自动重试；认证/网络异常、原生轨迹异常、两次方案都被拒绝时停止，不替换为本地恢复模板。模型必须保持叉子对应 left_arm、勺子对应 right_arm，不能强制忽略 IK，不能修改名义目标、监测阈值和注入协议。单次扰动实验只允许一个恢复分支，repeat_until_success=false；此限制不代表验证了官方无限重试或多次扰动行为。

程序读取已保存的密钥，正常无需重复输入。开启组会真实消耗 API tokens，最多两次请求。关闭组不调用 API。每组为种子 0、固定初始场景、GPU 0、单环境；真实位移及两组匹配条件与 v4 一致。

本轮允许模型只恢复受扰动的叉子。新增物理检查要求叉子在检测之后连续至少 5 步高于该时刻 2 厘米，并在同一腾空区间移动至少 2 厘米；勺子可保持原位。两件餐具最终 X/Y 误差均不超过 2 厘米、Z 相对检测时变化不超过 2 厘米，末尾 30 步漂移不超过 5 毫米，实际机器人须回位。这些是本地诊断条件；原场景的 Y-only 指标另行记录。实际夹持及画面仍需录像核验。

新结果目录为 outputs/llm_recovery_validation/时间/，自动输出 llm_recovery_validation_evidence.zip。打包两组日志、视频、截图、真实物体轨迹、检测/执行事件，以及新增的 llm_recovery_calls.json（公开消息、响应、调用状态及 tokens）、llm_recovery_generation.json（拒绝/修正及只读预检）、原始恢复 JSON、原编译器文件和实际执行文件。核验它们的内容及哈希一致，避免把旧缓存或本地模板误当模型结果。密钥、请求头、配置文件和模型内部 reasoning_content 不打包；公开调用日志不是独立的服务端认证证明。

若本轮失败，照样发送上述 ZIP，可以定位 API、schema、预测 IK 或实际抓取问题。成功状态 matched_recovery_benefit_awaiting_video_review 仍须传回录像核验。只复核已有实验目录可运行：

    bash agentchord_console/validate_llm_recovery.sh --review outputs/llm_recovery_validation/实验目录名

此前本地显式恢复测试继续使用 validate_disturbance.sh；它不会把新 API 组标为旧版测试通过。普通自然语言控制台入口仍为 start.sh。

固定版本官方来源：[RecoveryAgent](https://github.com/EDEM-AI/AgentChord/blob/74fe7dd756964b737952d26e9e65d4d5e2e29ec3/embodichain/agents/hierarchy/recovery_agent.py)、[RecoveryPrompt](https://github.com/EDEM-AI/AgentChord/blob/74fe7dd756964b737952d26e9e65d4d5e2e29ec3/embodichain/agents/mllm/prompt/recovery_prompt.py)、[CompileAgent](https://github.com/EDEM-AI/AgentChord/blob/74fe7dd756964b737952d26e9e65d4d5e2e29ec3/embodichain/agents/hierarchy/compile_agent.py)。

## 验证第一个官方目标 demo

退出其他仿真控制台，在 Ubuntu 图形桌面终端运行：

    bash agentchord_console/validate_demo.sh

运行一轮盘旁整理：叉子相对盘子世界 Y+0.16 米，勺子 Y-0.16 米。默认种子 0、固定初始场景、恢复关闭、不注入扰动。复用此前 Rearrangement_DS_Normal 缓存；没有缓存时由原 TaskAgent 生成。使用原 AgentChord 编译器和运行器，不走自然语言意图模板。可能保留此前的终点校正及抓取距离预检修正，均记录在本轮文件中，因此这是保留环境兼容修复的基础目标验证。

验证结束自动关闭仿真窗口，显示官方 is_task_success() 判定、从初末坐标独立计算的 Y 误差，并保存 validation_report.json、validation_evidence.zip。证据包包含报告、结果、日志、动作摘要/实际执行图、初末截图及 MP4；不包含密钥配置、run.json 或 API 配置快照。把证据包传回 Windows 后发送，才能完成视频核验。

程序记录原环境实际成功容差（默认 0.02 米）。只有动作完成、子进程正常退出、官方判定与坐标判定均为 True，且截图与视频存在，才标记 official_metric_passed_awaiting_video_review。这个状态不保证夹持、无掉落或稳定摆放；它也不等于论文成功率复现。初始位置已达标或物体高度明显下降时会额外提示。

首轮通过且视频确认后，可保留原配置随机化测试另一轮：

    bash agentchord_console/validate_demo.sh --randomized --seed 1

单独评估一个包含 run.json 的旧控制台结果目录，不再启动 GPU：

    bash agentchord_console/validate_demo.sh --review outputs/command_console/实验目录名

已有结果缺少新加入的实际容差字段时，标记证据不足；不猜测一个成功结论。盘内或自然语言任务会标记 wrong_task_or_mode，不能冒充本轮盘旁目标验证。本脚本尚未执行扰动恢复，后续需要在相同条件下验证恢复开／关。

该启动脚本在自己的进程内排除旧密钥环境变量和 LLM_URL，读取已保存的 DeepSeek 密钥；不改 .bashrc 或系统代理。没有保存密钥时仍会隐藏询问。

## 下一轮：物体移位与恢复开／关对照

本次修复包为 agentchord_console_disturbance_v4.zip。先关闭旧仿真，把 ZIP 放进 Ubuntu 项目的 tests/，然后执行：

    cd "$HOME/embodied_projects/agentchord_pack"
    python3 -m zipfile -e tests/agentchord_console_disturbance_v4.zip .

运行新包的测试入口：

    bash agentchord_console/validate_disturbance.sh

它依次启动两个独立子进程：先恢复关闭，再恢复开启。每次重置到种子 0 的固定场景，cuda:0、单环境、相同录像设置；原任务缓存和 SDK 文件不修改。本轮不调用 DeepSeek，不需要输入或更新密钥。

两组都执行上一轮的抓取、抬升、搬运、释放流程。餐具达到盘旁目标、机械臂执行原返回边第 20 步（原技能 20 步退让段的末步）时，调用原 misplaced_object 函数提交叉子沿世界 +Y 移动 6 厘米。v3 将注入从第 10 步延后，让机械臂完成退让段。注入只尝试一次。该原函数另执行 sim.update(100)；这 100 个物理 tick 单独记在注入事件中，不冒充 env.step 的录像或轨迹采样数。

v2 把设置位置的参数与 100 步物理更新后的位移作为同一数值严格核验，遇到已提交 6 厘米、物理更新后实际移动约 5.53 厘米的情况即停止，导致无法继续检测。本次 v3 协议分别核验：只读适配器转发原函数的一次 set_local_pose，记录其参数（参数记录不是瞬时物理位置测量）；提交的 XYZ 增量必须与 6 厘米要求相差不超过 1 微米，原物理更新必须恰为一次 100 步。实际物体位置在更新结束后测量，Y 增量须为 4–8 厘米（下限为原检测阈值的两倍），X/Z 合成位移不超过 3 毫米，叉子偏离目标至少 3 厘米，盘子和勺子移动不超过 3 毫米。两组实际扰动增量之差仍须不超过 3 毫米，才有对照可比性。真实位移和物理漂移如实记录，不进行第二次位置设置去补足到 6 厘米。这是下一轮明确声明的受控测试协议，不能把旧版中断实验改判为成功。

关闭组调用同一个原 monitor_object_moved 函数作只读记录，不改变原动作控制流。开启组把该函数绑定到返回边，由原 AgentTaskGraph.run 检测触发并切换恢复路径：先双臂返回，再重新抓取、摆放、释放和返回。恢复起步使用原 move_to_target_pose，目标为本轮记录的双臂初始位姿，sample_num=50；JSON 矩阵只转换为当前 SDK 位姿张量类型，数值不变。原 back_to_initial_pose 会先沿工具轴退让 8 厘米，打断该动作后重复调用会再退让一次；v2 避免这个重复动作。恢复末尾仍使用原 back_to_initial_pose。force_valid 保持关闭。仅对餐具预期静止的返回边监测位置变化，不将正常搬运误当作扰动。

恢复图是本地显式编排，位置检测读取仿真状态。本轮验证原监测器、恢复分支切换和物理恢复技能接口；不是视觉模型检测、RecoveryAgent 自动生成或论文完整复现。后续再验证自主规划和自动恢复图生成。

v3 实测已确认两组扰动有效、原监测器检测到移位，开启组完成 r01_reset，但在 r02_grasp 入口比较夹爪状态时出现 cuda:0/CPU 混用。原 sync_agent_state_from_robot 从物理机器人读入 CUDA 夹爪状态，而 env.open_state 阈值仍在 CPU；预测预检未经过这条真实状态同步路径，因此未提前发现。v4 在现有进程内兼容包装中先调用一次原同步，再把两臂缓存夹爪状态转换到该阈值的设备和类型，保留实际数值及形状。关节和末端位姿缓存保留原设备，IK/FK、检测阈值和分支判断照常运行；异常仍传播。四个临时绑定在退出时恢复。新版本使用独立任务指纹，原 v3 中断证据仍按未完成保存。

v4 沿用 v3 的注入时机、实际位移范围及所有成功条件，不放宽验收阈值。已用固定版本原同步、回位和抓取函数体，以及可区分 CPU/CUDA 的本地测试夹具，复现故障并验证转换；夹具不代替真实 CUDA/物理运行。重新抓取、搬运、释放和最终恢复效果仍需 2080 的新证据包核验。

执行前对名义动作作完整只读 IK/轨迹预检。另在克隆状态上预测返回第 20 步的关节位置，以及恢复时餐具的目标位置加扰动；检查恢复路径，不操作真实仿真。未来餐具姿态采用初始旋转与初始静置高度，这是可达性预测假设，不能保证实际抓取。预检失败或原生异常时停止，并保留 plan_check.json。观测回调报错也保存已完成步骤的实际位置，避免扰动事件存在而轨迹缺少该步。

恢复会增加动作长度，两组的本轮配置将 max_episode_steps 都设为 1200，观察钩子另限制最多 1100 个 env.step，图运行时最多 30 次边切换。原场景配置文件和普通控制台模式不修改。

报告逐组核验：注入前两件餐具均达到目标；真实位移约 6 厘米；原监测器真实返回 True；开启组的八条恢复边真实开始、完整结束；最终官方判定与坐标复核一致；实际机器人关节返回初始范围；餐具在检测之后再次真正抬升、搬运并稳放。关闭组未达到目标是对照结果，不能当作程序崩溃；其动作也必须正常完成。

两组初始物体位置和名义目标差别不超过 10 微米、初始关节差别不超过 0.0001 弧度，场景配置一致；同一动作边、同一步注入，测得的位移和注入前状态差别不超过 3 毫米，才标记条件匹配。若关闭组也达到目标，如实记录，不能硬改成“恢复有效”。

输出目录为 outputs/disturbance_validation/时间/。pair_report.json 保存对照摘要；每组的完整数据仍在 outputs/command_console/时间/。自动导出 disturbance_validation_evidence.zip，包括两组视频、注入前后截图、初末坐标、每步真实物体位置、监测调用结果、原图边执行事件、实际执行图、配置和只读预检。密钥文件、环境变量、API 配置和完整对话历史不打包。

matched_recovery_benefit_awaiting_video_review 表示该单组对照的坐标和运行时证据符合预期，仍需传回 ZIP 核验录像。脚本失败也尽量保存诊断包。它不输出论文成功率；一组不能代表多任务、多扰动统计结果。

只复核原 Ubuntu 已有对照目录，不重新启动仿真：

    bash agentchord_console/validate_disturbance.sh --review outputs/disturbance_validation/实验目录名

固定版本原接口：[物体移位函数](https://github.com/EDEM-AI/AgentChord/blob/74fe7dd756964b737952d26e9e65d4d5e2e29ec3/embodichain/agents/agentchord/error_functions.py)、[位置监测器](https://github.com/EDEM-AI/AgentChord/blob/74fe7dd756964b737952d26e9e65d4d5e2e29ec3/embodichain/agents/agentchord/monitor_functions.py)、[图运行时](https://github.com/EDEM-AI/AgentChord/blob/74fe7dd756964b737952d26e9e65d4d5e2e29ec3/embodichain/agents/agentchord/agent_graph.py)。

## 抬升搬运物理测试

上一轮固定盘旁整理已在 2080 达到原场景的 Y 位置指标。本轮增加明确的竖直抬升，区分实际搬运与桌面滑移：双臂抓取 → 朝下 → 原位置竖直抬升 → 保持高度搬到盘旁 → 下放 → 释放 → 返回初始姿态 → 再观察 30 步。

在项目根目录解压新包后运行：

    bash agentchord_console/validate_carry.sh

固定种子 0、初始姿态、cuda:0 和单环境，恢复关闭。使用本地受控动作图和原 SDK 技能；本轮不调用 DeepSeek，不需要密钥。这是新增物理诊断，不代表原 Agent 自主规划或论文扰动恢复测试。

完整只读预检从抬升 5 厘米开始；仅在原 IK 返回失败时尝试 4、3 厘米，不改变最终 XY 目标。任何原生异常、首步失败或候选全部失败都会停止餐具动作。原任务缓存、SDK 文件和用户密钥配置不修改。

object_motion.jsonl 在每次原物理 step 后读取三件刚体的实际 XYZ；不会用夹爪目标位置作为餐具位置。动作图执行一次；末尾只保持最终关节动作并推进 30 步物理，以观察退离后的餐具稳定性。

新增检查要求两件餐具各自连续至少 5 步高于初始位置 2 厘米，且在同一腾空区间内水平移动至少 2 厘米；最终 X/Y 误差不超过 2 厘米、Z 相对初始变化不超过 2 厘米，末尾 30 步三维漂移不超过 5 毫米。它们是本地诊断阈值，不能冒充官方论文指标，也不能单凭坐标证明夹爪接触关系。

结果目录会生成 validation_report.json 和 carry_validation_evidence.zip。状态 measured_carry_passed_awaiting_video_review 表示坐标检查通过，实际夹持和画面效果仍需录像核验；失败时也打包已生成的诊断。传回这个 ZIP，即可继续检查，暂不进行恢复开／关实验。

只复核已有本轮目录：

    bash agentchord_console/validate_carry.sh --review outputs/command_console/实验目录名

原 validate_demo.sh 仍只验证上一轮基线，不会把新增受控测试当作原基线。普通自然语言控制台使用方式保持不变。

## 自然说话

可以直接输入，例如：

- 帮我整理一下餐盘，把餐具放进去。
- 只把勺子放盘里，叉子别动。
- 叉子放在餐盘左边，勺子放右边。
- 把叉子往右挪五厘米。
- 能不能帮我把两个餐具摆好？

这些是表达示例，不是匹配词表。每条任务文本都真实调用 DeepSeek JSON mode，由模型按句意解析。先打印公开的理解摘要和默认约定，再检查并执行。模型可能误解，因此请核对摘要和录像；这不是对所有表达都理解准确的保证。

需要补充对象或位置时，模型会问一个具体问题。在下一次“指令 >”直接回答，例如“勺子”“放盘子里面”。对话保留最近四轮供指代和澄清使用。普通问答、模型要求补充或识别出未支持任务时，不启动仿真；动作布局检查阶段也可能要求分开重合目标，此时场景已初始化，但不会操作餐具。

每条执行指令都会从初始场景开始新一轮。对话上下文用于理解语言，不代表沿用上一轮的物理状态。“往右挪五厘米”以本轮初始位置为基准，会在默认说明中告知；目前不能在前一轮结果上继续操作或撤销上轮物理动作。

录像结束时保留窗口。可以直接在终端输入下一条自然语言要求，也可以按回车回到控制台；输入的下一条要求会被接收，不再丢弃。

## 当前能够操作什么

当前场景为盘子、叉子、勺子和双机械臂。可以搬运一件或两件餐具，放在盘内、盘旁，或按明确的水平偏移移动。盘子用作位置参照。没有杯子、抹布等物体，也没有搬盘子、擦桌子、指定餐具旋转或恢复实验技能。模型会说明缺少的能力；不会把包含未支持部分的请求悄悄裁成另一项任务来执行。

未指定坐标系时，“左右、前后”按本轮录像相机视角。程序从相机 eye/target 计算方向；默认视角画面右为世界 +Y，画面前为世界 +X。用户明确世界 X/Y 或数值时按指定轴换算，单位为米，单轴最多 ±0.30 米。

“整理餐盘”等普通整理请求默认两件餐具分开放入盘内，模型会说明此默认。只要求某件餐具时，仅给相应臂下达动作；另一臂保持其当前姿态。物理接触仍可能影响其他物体，应通过视频和最终坐标确认限制是否落实。

## 动作如何执行

通过校验的语义目标转换为抓取 → 朝下 → 悬停 → 下放 → 释放 → 返回初始姿态六步图。对象分配沿用叉子对应 left_arm、勺子对应 right_arm。第一版仍使用已验证的技能流程；机器人手臂名称与画面左右无关。

执行前检查首步精确抓取 IK 和整段原 SDK 动作/轨迹，在独立的代理状态副本上预检，不调用 drive 或 env.step。遇到原 IK 返回 false 时，只测试有限候选。用户明确的水平坐标和方位不改；未指定的盘内间距、悬停/释放高度可选不同候选。原生轨迹异常不当作不可达继续重试。跳过检查或候选全部失败时停止餐具操作，并保存诊断。

预检通过仅说明关键姿态和轨迹可生成。是否实际抓住、碰撞、滑落以及餐具是否完整放在盘内，仍需物理仿真视频核验。此前的两侧摆放指标 y±0.16 米不用于判定盘内或其他自定义目标成功；它保留在 result.json 中。

Toppra 插值入口临时使用 CPU 关节数值，IK 与物理仿真仍在 cuda:0。只在当前进程内包装，结束时恢复，原 SDK 文件不修改。此前 finalize_actions 和轨迹输出的 CUDA→NumPy 修复需要保留，单臂 idle 分支也依赖它们。

## 控制台命令

- 视频：打开最近一轮实际仿真录像；问答不会覆盖它。
- 结果：显示最近一轮仿真目录。
- 重新规划 [要求]：重新解释并规划要求；自然语言模式每条要求都会调用 API。
- 退出：结束控制台。整理过程中可按 Ctrl+C 停止当前进程。

一条命令执行：

    bash agentchord_console/start.sh --no-proxy --stable-scene --instruction "只把勺子放盘里，叉子别动"

默认已经是自然语言模式。旧 --plate-layout 参数在此模式下不限制文本；布局根据语义自动选择。

需要回到旧版的原 TaskAgent 图生成流程，可加 --legacy-plan；旧版固定双餐具盘内演示可同时加 --legacy-plan --plate-layout。旧基线缓存保留。

## 记录与核验

每轮放在 outputs/command_console/日期时间/，包括：

- intent.json：模型解释后的、经过校验的公开目标。不读取或保存内部 reasoning_content。
- run.json：原话、模型、场景配置、视角约定和最近对话上下文；密钥不写入。
- task_plan.json、agent_compiled_graph.language.json：本地技能编排和目标图。
- plan_check.json、grasp_check.json：完整候选检查和首步抓取检查。
- agent_compiled_graph.original.json、agent_compiled_graph.used.json、plan_summary.json：原输入工作图、实际执行图和公开动作摘要。
- planning_trace.json、run.log：阶段记录与日志。
- initial.png、final.png、videos/episode_0_cam1.mp4：仿真初末截图和录像，发生普通 Python 异常时也尽量保存结束状态。
- result.json：理解类别、执行是否完成、餐具初末坐标、原场景指标和视频路径；动作完成不等于实际摆放合格。
- process_status.json：退出码。

问答与模型澄清只保存理解/回复和日志，没有视频是正常情况。API 错误或 JSON 格式错误会停止本轮；不会执行模型生成的任意 Python 或 shell 代码。

已完成本地假 SDK、语义 schema、方位转换、单/双对象图、上下文、失败停止、原校验器和执行次数验证。此前固定双餐具摆盘已经在 2080 完成 220 步并导出视频；新的自然语言调用和单臂流程仍需这台 2080 实测。若报错，发送 run.log、intent.json、plan_check.json、result.json 中已生成的文件；若动作跑完，发送 MP4 和 result.json。

参考：[AgentChord](https://github.com/EDEM-AI/AgentChord)、[DeepSeek JSON Output](https://api-docs.deepseek.com/guides/json_mode/)。
