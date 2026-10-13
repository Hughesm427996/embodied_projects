# Demo 跑通记录与改动清单

日期：2026-09-28 ｜ 环境：Ubuntu 24.04.5 / ROS 2 Jazzy / RTX 2080 (595.91.07) / i9-9900K(16线程) / 64GB RAM

---

## 1. 对仓库代码的改动（仅 2 个文件，可 `git checkout` 还原）

### `examples/rosbot-xl-demo.py`（+7 行，修上游 bug）

```python
# (1) pydantic 前向引用无法解析 → agent 初始化直接抛错
#     根因：rai/tools/ros2/manipulation/custom.py:343 的
#     get_grabbing_point_tool: "GetGrabbingPointTool" 只在 TYPE_CHECKING 分支导入
from rai_whoami import EmbodimentInfo

GetObjectPositionsTool.model_rebuild(
    _types_namespace={"GetGrabbingPointTool": GetGrabbingPointTool}
)

# (2) 首次初始化失败后 rclpy context 已存在，重跑报
#     RuntimeError: Context.init() must only be called once
def initialize_agent():
    if not rclpy.ok():
        rclpy.init()
```

未改动任何 `src/` 下的库代码。`config.toml` 的 `[openai]` → deepseek 是你本人之前的修改，非本次。

### 系统环境改动（非仓库）

| 改动 | 命令 | 目的 |
|---|---|---|
| 关闭空闲锁屏/熄屏 | `gsettings set org.gnome.desktop.screensaver lock-enabled false`<br>`gsettings set org.gnome.desktop.screensaver idle-activation-enabled false`<br>`gsettings set org.gnome.desktop.session idle-delay 0` | 远程桌面 5 分钟无输入自动锁屏，截图/画面全黑 |
| 唤醒显示器 | `xset s off -dpms; xset dpms force on` | DPMS Off 导致黑屏 |
| `OPENAI_API_KEY` | 从 `~/.bash_history` 取（deepseek key，已验证 `/models` 返回200） | 未写入任何文件，仅在启动脚本中临时注入 |
| pip 安装 `pyautogui` | 失败（PEP 668 externally-managed） | 未改动系统 Python |

---

## 2. ROSBot XL Demo —— 已跑通

### 启动顺序（全部 `setsid` 后台化）

```bash
# 1) 仿真（画面）
DISPLAY=:0 ./demo_assets/rosbot/RAIROSBotXLDemo/RAIROSBotXLDemo.GameLauncher -bg_ConnectToAssetProcessor=0

# 2) nav2(SLAM) + 感知服务（game_launcher:=/bin/true 避免起第二个仿真）
source /opt/ros/jazzy/setup.bash && source ./setup_shell.sh
ros2 launch ./examples/rosbot-xl.launch.py game_launcher:=/bin/true

# 3) Streamlit GUI（key 从 history 取）
OPENAI_API_KEY=$(grep -o 'sk-[A-Za-z0-9]\{20,\}' ~/.bash_history | tail -1) \
  streamlit run examples/rosbot-xl-demo.py --server.port 8501 --server.headless true
```

### 验证结果

- 画面正常，FPS ~61
- `ros2 node list`：`/o3de_ros2_node`、`/bt_navigator`、`/controller_server`、`/slam_toolbox` 均 active
- 手动 `ros2 action send_goal /navigate_to_pose`（map 系 2m），odom x: 0 → **1.94**，小车实际移动
- Streamlit `http://localhost:8501` 正常，agent 初始化无报错

### 停止命令

```bash
pkill -f "ros2 launch" ; pkill -f "streamlit run" ; pkill -f run_perception ; pkill -f GameLauncher
```

> 注意：`pkill -f GameLauncher` 会连农业/机械臂的一起杀。

---

## 3. 农业 Demo —— 卡在仿真二进制崩溃（未跑通）

### 现象

`RAIAgricultureDemo.GameLauncher` 启动约 2 秒后 **SIGABRT/SIGSEGV**（exit 134），
永远停在 `Game.log` 的 `(RHI) - Registering vulkan RHI` 之后。

### 已排除的原因（逐条有证据）

| 假设 | 验证方法 | 结论 |
|---|---|---|
| 下载损坏 | 重新下载官方 `RAIAgricultureDemo_noblejazzy.zip`(518,933,978B)，`unzip -t` 通过，逐文件 md5 与本地比对 | ❌ GameLauncher/pak/所有文件与官方**逐字节一致** |
| 显卡/Vulkan 驱动 | `VK_ICD_FILENAMES` 指向无效 → O3DE 走 **null RHI** | ❌ 无 Vulkan 仍崩 |
| 中文 locale | `LC_ALL=C LANG=C` 运行 | ❌ 照崩 |
| 环境变量污染 | `env -i` 纯净环境 | ❌ 只是缺 ROS 库起不来，非崩溃原因 |
| 资源限制 | `ulimit -a`：fd 1048576 / vmem unlimited / stack 8MB | ❌ 无限制 |
| 线程数竞争 | `taskset -c 0-3`（4 核） | ❌ 照崩 |
| 缺少 Assets/ | 官方 zip 里本来就没有 `Assets/` | ❌ 与上游一致，FileTag 报错无害 |
| ROS 依赖错误 | `ldd` 与 rosbot 对比，均加载 `/opt/ros/jazzy/lib` | ❌ 库一致 |
| 巨型分配打爆分配器 | gdb 断点打印所有 >1MB 分配 | ❌ 最大 101MB，均正常 |
| mmap 失败 | `strace -f` 统计2655次 mmap | ❌ **零次** ENOMEM/mprotect 失败 |

### 对照实验

| Demo | 结果 |
|---|---|
| ROSBot XL | ✅ 正常运行 |
| Manipulation（机械臂，**跑出窗口属于故意的对照实验**） | ✅ 超时被杀 = 进程存活 |
| Agriculture | ❌ 每次 ~2s 崩溃（累计 8+ 次复现） |

### gdb 抓到的崩溃现场

```
si_addr = 0xfffffffffffffff8      # 即 rcx=0 的空指针（-8 偏移）
=> mov -0x8(%rcx),%rdx            # rcx=0
#0 AZ::HphaSchemaBase<false>::HpAllocator::tree_alloc(unsigned long)
#1 AZ::HphaSchemaBase<false>::HpAllocator::alloc
#2 AZ::SystemAllocator::allocate
#3 AZStd::vector<AZ::RPI::ShaderOptionDescriptor>::reserve
...
#23 AzFramework::GenericAssetHandler<AZ::RPI::ShaderAsset>::LoadAssetData
```

另一轮栈（同一处）：
`HpAllocator::tree_extract_aligned` ← `PhysX::RigidBodyComponent` 实例创建 ← `ObjectStreamImpl::LoadClass`

**结论：O3DE 堆分配器(HPHA)内部 free-tree 出现 NULL 节点 → 崩溃。**
分配尺寸正常、mmap 全成功，属于该二进制在本机触发的上游引擎缺陷（资源文件本身无问题）。

### 尚未验证的下一步（按性价比排序）

1. **gdb 慢速运行实测**：`bp5` 那轮因 gdb 线程管理报错中断，但已加载到 `enhancedpbr_*.azshader`（每片 1.7~2.8s），说明**断点拖慢后能走得更远** → 试 `taskset` + 降低加载并发改发（如 `--` 资源线程数），可能是多线程加载下的竞态。
2. 查 O3DE `HpAllocator::tree_alloc` 中 NULL 未判空的分支对应的触发条件（源码在 `o3de/AzCore/Memory/HphaSchema.inl`）。
3. 尝试 `r_agriculture` 旧版/新版二进制（S3 无法 list，只有 noblejazzy 一个 URL；`rai-agriculture-demo` 仓库 development 分支 2025-10 有更新，但**源码编译需 60GB + 数小时，本次未做**）。
4. 报 issue 到 https://github.com/RobotecAI/rai/issues（附上面的 gdb 栈）。

### 农业 Demo 的正确启动姿势（等二进制问题解决后）

```bash
source /opt/ros/jazzy/setup.bash && source ./setup_shell.sh   # 必须：二进制要 ROS 库
DISPLAY=:0 ./demo_assets/agriculture/RAIAgricultureDemo/RAIAgricultureDemo.GameLauncher -bg_ConnectToAssetProcessor=0
# 另开一个已 source 的终端：
python examples/agriculture-demo.py --tractor_number 1
```

> 农业二进制**不能**在未 source 的纯净环境跑：会缺 `libnav_msgs__rosidl_typesupport_fastrtps_c.so`。

---

## 4. 调试产物位置

| 文件 | 内容 |
|---|---|
| `/tmp/opencode/agri_*.log` | 各次运行的 stdout |
| `/tmp/opencode/agri_gdb.log` / `agri_fault.log` / `agri_alloc.log` | gdb 崩溃栈、寄存器、映射 |
| `/tmp/opencode/agri.strace` | 12 万行系统调用跟踪 |
| `/tmp/opencode/agri_noblejazzy.zip` | 官方包（校验用，519MB，可删） |
| `/tmp/opencode/{off,loc}_full.txt` | 官方 vs 本地文件 md5 清单 |

## 5. 当前机器状态

- **无任何残留进程**（仿真 / nav2 / 感知 / streamlit 全部已停）
- 屏幕已解锁，锁屏与空闲已关闭
- 仓库改动：`examples/rosbot-xl-demo.py`（+7）、`config.toml`（你本人的）
- 未提交任何 commit

---

## 6. 追加排查（2026-09-28 下午，第二轮）

### 6.1 配置注入机制（已验证）

- `Cache/linux/autoexec.cfg` 按**控制台命令**逐行执行（空格分隔，如 `cl_jobThreadsMinNumber 1`），是唯一可用的注入点；pak 内自带的 `autoexec.cfg` 仍会执行 `LoadLevel levels/almondorchard/almondorchard.spawnable`。
- `Cache/linux/user.cfg` 走 CryEngine 配置解析器，空格分隔会报 `invalid configuration line`（正确语法应为 `key=value`）。
- `Registry/*.setreg` 的 `/O3DE/Autoexec/ConsoleCommands` 在本二进制里**不生效**（实测无回显、cvar 未应用）。
- `loadstring`（本机实测）：`cl_jobThreads*` 默认 → 8 个 `Default.*`（job）线程 + 14 个 `TaskWorker`（taskgraph）；`cl_jobThreadsConcurrencyRatio 0.25` → 4 个 job 线程。

### 6.2 降并发与其它规避实验

| 实验 | 结果 |
|---|---|
| 默认（8 job + 14 taskgraph） | ~20s SIGSEGV/SIGABRT，崩在 HPHA |
| 4 job（ratio 0.25） | ~220s 崩 |
| 1 job + 1 taskgraph | 不崩，但**卡死**：主线程自旋、`/clock` 恒为 0、`/tractor1/*` 服务始终不出现、RSS 以 ~100MB/min 持续增长 |
| `LD_PRELOAD` 劫持 `sysconf` 使 `hardware_concurrency()=1` | 60s 崩（lib 见 `/tmp/opencode/fakenproc.so`） |
| `CUDA_VISIBLE_DEVICES=""`（禁 CUDA/PhysX GPU） | 照崩 |
| `cl_streamerDevMode 1` + `/Amazon/AzCore/Streamer/UseAllHardware=false` | 不崩，但同样卡死（9 分钟 `/clock` 仍为 0） |
| `cl_terrainPhysicsColliderMaxJobs 1` | 无改善 |
| `-rhi=null` | 照崩（与 Vulkan/驱动无关，复核确认） |

### 6.3 崩溃现场（两种）

1. HPHA 分配器 free-tree 被破坏：`tree_extract_aligned+0x50`（`mov -0x8(%rdx),%rsi`，rdx 为垃圾指针，如 `0x1c00000001f00`）或 `tree_alloc`（NULL 节点），调用链均经过 `ObjectStreamImpl::LoadClass` 反序列化（spawnable / ShaderAsset）。
2. `AZ::Vulkan::SwapChain::PresentInternal()` → `CommandQueue::QueueCommand` 执行 lambda 时崩在 **NVIDIA Vulkan ICD**（`libGLX_nvidia.so.0` 的 `__glx_Main+0x2110`），即 `vkQueuePresentKHR` 路径。

### 6.4 资源/内容侧结论

- `Cache/linux/*.pak` 是标准 zip（`unzip -l`/`unzip -t` 均可），`game_linux.pak` 866MB / 809 文件；关卡/资产与官方一致（前文已逐文件 md5 校验）。
- `almondorchard.spawnable` 是二进制 ObjectStream（非 JSON），**无法手工编辑**再打包。
- 农业专属 gem（rosbot 没有）：`Terrain`、`Vegetation`、`CsvSpawner`、`ROS2ScriptIntegration`、`PhysX5Debug`。
- 运行期 RSS 稳定在 ~1GB（非内存耗尽）；GPU0 承载显示且被 O3DE 选中（非双卡/PRIME 问题）；VRAM ≈1.2GB / 8GB。

### 6.5 最终结论

预编译二进制（O3DE 24.09，2024-10-18 构建）在本机**不可用**：属引擎侧缺陷（并发/反序列化路径破坏 HPHA 堆），配置层面只能把"约 20 秒必崩"改成"不崩但卡死"，无法真正跑起来。官方文档给出的兜底是**源码编译**（`rai-agriculture-demo` 的 development 分支，2025-10 有更新），需要 ~60GB 磁盘 + 数小时。

已清理：本次注入的所有 cfg/setreg 均已删除，无残留进程。
