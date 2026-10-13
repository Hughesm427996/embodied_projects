# FAQ — 常见问题

## 1. `python server.py` 报错 ModuleNotFoundError
```
pip install -r requirements.txt
```
如果用了虚拟环境，确认已激活：`source venv/bin/activate`

## 2. LLM 一直重试，控制台刷 401 错误
API Key 没配或配错了。编辑 `.env`：
```
LLM_API_KEY=sk-你的真实key
```

## 3. Web UI 显示 OFFLINE
- 检查 server.py 是否在运行
- 如果从其他电脑访问，确认 `ui/src/hooks/useSocket.js` 中没有写死 `localhost`
- 重新构建前端：`cd ui && npm run build`

## 4. 切到 AI 模式后按钮点不动
需要先点"⚡ 初始化系统"，初始化完成后才能切换模式。

## 5. PX4 编译失败 (macOS ARM64)
```bash
export CMAKE_POLICY_VERSION_MINIMUM=3.5
export CXXFLAGS="-Wno-vla -Wno-error=attributes"
```
详见 [docs/SIMULATION_SETUP.md](SIMULATION_SETUP.md)

## 6. Gazebo 启动后黑屏 / 无模型
检查模型资源路径：
```bash
export GZ_SIM_RESOURCE_PATH="$HOME/.simulation-gazebo/models"
```

## 7. 无人机起飞后立刻降落
PX4 参数 `COM_DISARM_PRFLT=10`，ARM 后 10 秒内必须发 takeoff 命令。

## 8. MAVSDK 连接失败
- 单独启动 mavsdk_server：`mavsdk_server -p 50051 udp://:14540`
- 不要在重启 server.py 时 kill mavsdk_server
- 检查端口占用：`lsof -i :14540`

## 9. 摄像头画面全黑
- 确认 Gazebo 仿真正在运行
- 检查 PX4 模型是否用了 `x500_sensor`（带摄像头的版本）
- 运行 `./scripts/setup_px4.sh` 安装自定义模型

## 10. 模型配置页面无法保存
- 确认从正确的地址访问（不是 localhost 但从别的机器访问）
- 检查浏览器控制台是否有 CORS 错误

## 11. Python 3.14 报 dataclass 错误
```
TypeError: non-default argument follows default argument
```
建议使用 Python 3.10-3.12。3.14 的 dataclass 校验更严格。

## 12. 磁盘空间不足
```bash
# 清理 Homebrew 缓存
brew cleanup --prune=all

# 清理 pip 缓存
pip cache purge

# 清理 PX4 编译缓存
cd PX4-Autopilot && make clean
```
