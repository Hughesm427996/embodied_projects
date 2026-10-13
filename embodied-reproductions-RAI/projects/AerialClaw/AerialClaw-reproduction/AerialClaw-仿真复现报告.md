# AerialClaw 仿真复现报告

## 复现范围

- 项目：XDEI-Group/AerialClaw
- 固定提交：`e01adaa73c38fb10ec4bc5e8c0f71915cc7490a8`
- 范围：仅仿真，不含真机
- 平台：Windows 11 + WSL2 Ubuntu 22.04.5
- 仿真：PX4 SITL + Gazebo Harmonic 8.15.0
- 载具模型：`x500_lidar_2d_cam`
- 场景：`urban_rescue`
- GPU：NVIDIA GeForce RTX 3050 Ti Laptop GPU，4 GB
- 规划模型：DeepSeek `deepseek-chat`
- 视觉模型：豆包 Ark `Doubao-Seed-2.1-pro` 对应推理接入点

## 最终结论

AerialClaw 官方仿真链路已在本机 WSL2 中完整运行。最终实时 Doctor 检查为 **27 pass、0 warn、0 fail**。PX4、Gazebo、MAVSDK、Micro XRCE-DDS Agent、五路相机和二维激光雷达均在线。

三个官方预设任务均已完成：

1. **起飞到 12 米，向东飞 30 米，观察前方，然后返回起飞点降落**：通过。起飞、定点飞行、视觉观察、返航降落全部成功。
2. **飞到 `[-30,40,-15]` 搜索亮红色发光物体并报告**：通过，带自动恢复。Agent 在多轮调整高度和位置后，用豆包视觉模型发现目标，标记位置 `[-31.85,38.16,-12.83]` 并报告周围物体。一次 `perceive` 调用返回 `adapter 不可用`，随后自动回退到 `observe` 完成目标。预设任务未要求降落，证据保存后另行执行安全降落并验证成功。
3. **在 `(-30,40)`、`(-50,60)`、`(-25,55)` 三点观察并返航汇总**：通过。三个坐标均完成飞抵、下视观察和报告，返航降落成功，最终为 **12/12 步成功、13 轮思考**。

## 逐层验收结果

| 层级 | 结果 | 证据 |
| --- | --- | --- |
| WSL2 / Ubuntu | 通过 | Ubuntu 22.04.5 可启动，发行版位于 `D:\WSL\Ubuntu-22.04` |
| NVIDIA / WSLg | 通过 | `nvidia-smi` 可见 RTX 3050 Ti；OpenGL 渲染器可指定 NVIDIA D3D12 |
| Gazebo | 通过 | Gazebo Sim 8.15.0 GUI 和场景渲染正常 |
| PX4 SITL | 通过 | `px4_sitl_default` 构建成功 |
| DDS / MAVSDK | 通过 | MicroXRCEAgent 与 MAVSDK 在线 |
| 传感器 | 通过 | 5 路 640×480 相机持续出帧；二维激光雷达持续出帧 |
| 手动飞行 | 通过 | 起飞 5 米并降落成功 |
| LLM 规划 | 通过 | DeepSeek 可规划并执行技能链 |
| VLM 视觉 | 通过 | 豆包可分析仿真相机图像并生成物体描述 |
| 官方任务 1 | 通过 | 完整起飞、移动、观察、返航降落 |
| 官方任务 2 | 通过（带恢复） | 发现、标记、报告红色目标，随后安全降落 |
| 官方任务 3 | 通过 | 三点观察、逐点报告、汇总、返航降落 |
| 最终实时检查 | 通过 | 27 pass、0 warn、0 fail |

## 必要兼容处理

1. 在虚拟环境中固定 `mavsdk==2.8.4` 和 `protobuf==3.20.1`，解决 Gazebo Python 消息与新版 protobuf 不兼容的问题。
2. 使用 `~/.local/share/aerialclaw-gz-python` 暴露系统 Gazebo Python bindings。
3. 执行 `sudo ldconfig`，让 MicroXRCEAgent 找到 `/usr/local/lib/libmicroxrcedds_agent.so.3.0`。
4. 对 `server.py` 做一处兼容补丁：把两处 `output.get("action", {})` 改成 `output.get("action") or {}`，处理 DeepSeek 在任务结束时返回 `"action": null` 的情况。该差异已单独保存为补丁证据。

## 已知限制

- `perceive` 技能仍返回 `adapter 不可用`；`observe` 技能正常，官方三个预设任务均可依靠 `observe` 完成。
- 任务 2 的视觉搜索效率较低，共进行 22 轮思考；这反映了 Agent 搜索策略和相机视角的实际行为，不影响最终成功判定。
- WSL2 启动曾出现 VHDX `E_ACCESSDENIED`；以管理员 PowerShell 重启 `WslService` 后恢复。Ubuntu 数据没有损坏。

## 证据位置

所有证据保存在：

`D:\codex\AerialClaw-reproduction\evidence`

- `03-flight-smoke`：手动飞行冒烟测试
- `04-ai-planner`：LLM 规划检查及兼容补丁
- `05-ai-flight-observe-land`：AI 起飞、视觉观察、降落集成测试
- `06-official-demo-1`：官方任务 1
- `07-official-demo-2`：官方任务 2及安全降落
- `08-official-demo-3`：官方任务 3
- `09-final-validation/live-doctor.txt`：最终 27 项实时检查

每个官方任务目录包含状态 JSON、传感器状态、相机截图、完整任务日志、HTML 巡检报告和 `manifest.json`。

## 后续启动

在 Windows PowerShell 中：

```powershell
wsl -d Ubuntu-22.04
```

在 Ubuntu 中：

```bash
cd ~/AerialClaw
source venv/bin/activate
bridge="$HOME/.local/share/aerialclaw-gz-python"
export MESA_D3D12_DEFAULT_ADAPTER_NAME=NVIDIA
export GZ_PYTHONPATH="$bridge"
export PYTHONPATH="$bridge"
./scripts/sim_quickstart.sh --restart
```

控制台地址：`http://127.0.0.1:5001/`

停止仿真：

```bash
kill $(cat /tmp/aerialclaw_full_server.pid) \
     $(cat /tmp/aerialclaw_gz_gui.pid) \
     $(cat /tmp/aerialclaw_full_sim.pid) 2>/dev/null || true
```
