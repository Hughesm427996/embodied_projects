# AerialClaw Windows Mock 复现记录

## 复现范围

本次仅复现官方 Windows Local Mock 路线，不包含 PX4、Gazebo、AirSim 或真机。

## 固定版本

- 仓库：`https://github.com/XDEI-Group/AerialClaw`
- 分支：`main`
- 提交：`e01adaa73c38fb10ec4bc5e8c0f71915cc7490a8`
- Python：3.11.7
- Node.js：24.18.0
- npm：11.16.0

## 验证结果

1. 仓库一致性检查：通过。
2. Python 全量语法编译：通过。
3. pytest：18 项通过，3 项失败；补齐 docs 后为 19 项通过、2 项失败。
4. 剩余两项失败均属于官方 Gazebo/Linux 指南测试：Windows 不呈现 Unix 执行位；当前脚本缺少测试期待的 macOS 注释。与 Windows Mock 运行无关。
5. 前端 `npm run lint`：通过。
6. 前端 `npm run build`：通过，70 个模块完成构建。
7. `/api/status`：HTTP 200，系统初始化成功。
8. `/api/adapter/status`：`adapter=mock`、`connected=true`。
9. Web 控制台：显示 `ONLINE` 和 `系统就绪`。
10. 动作闭环：`takeoff → fly_relative → get_position → land` 全部成功。
11. 最终状态：UAV_1 位于 N=10 m、E=2 m、高度 0 m，`in_air=false`。

## 启动

在 PowerShell 中运行：

```powershell
cd D:\codex\AerialClaw-reproduction
.\start-windows-mock.ps1
```

打开 `http://127.0.0.1:5001`。

## 说明

Windows Mock 验证的是 Agent Runtime、技能调度、世界状态、Web 控制台和模拟动作反馈。它不包含三维刚体动力学和真实传感器图像。完整 PX4 + Gazebo 展示需要 Linux 仿真主机。

## 已知问题

- 官方全量 pytest 在 Windows 上有两项 Gazebo/Linux 指南检查失败；不影响 Mock 服务。
- 浏览器会话期间观察到适配器原始坐标可被速度事件改写，且与此前技能回执的最终世界坐标不一致；当前 in_air=false。这是当前提交的 Mock 状态一致性问题。
- npm install 报告 18 个依赖安全告警；为保持官方锁文件，未自动执行 npm audit fix。
