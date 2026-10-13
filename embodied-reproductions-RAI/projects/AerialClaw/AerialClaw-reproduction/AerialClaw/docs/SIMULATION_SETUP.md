# Simulation Setup Guide — 仿真环境搭建指南

AerialClaw 的仿真环境基于 **PX4 SITL + Gazebo Harmonic**，本文档提供完整的搭建步骤。

## Prerequisites / 前置依赖

| Component | Version | Purpose |
|-----------|---------|---------|
| Python | >= 3.10 | Core runtime |
| CMake | >= 3.22 | PX4 compilation |
| Gazebo Harmonic | gz sim 8.x | 3D simulation |
| PX4 Autopilot | v1.15+ | Flight controller SITL |
| Micro XRCE-DDS Agent | latest | PX4 ↔ ROS2 bridge |
| MAVSDK | latest | Drone control API |

## Step 1: Install Gazebo Harmonic

### macOS
```bash
brew tap osrf/simulation
brew install gz-harmonic
# Verify
gz sim --version  # should show 8.x
```

### Ubuntu 22.04
```bash
sudo apt install -y gz-harmonic
```

## Step 2: Clone and Build PX4

```bash
# Clone PX4 inside the AerialClaw project (or anywhere you prefer)
git clone https://github.com/PX4/PX4-Autopilot.git --recursive
cd PX4-Autopilot
git checkout v1.15.4  # or latest v1.15.x

# Install dependencies
bash Tools/setup/ubuntu.sh  # Ubuntu
# or for macOS: install build tools manually (see Troubleshooting below)

# Build for SITL with Gazebo
make px4_sitl gz_x500
```

### macOS ARM64 Known Issues

If building on Apple Silicon (M1/M2/M3/M4):

```bash
# Fix 1: CMake version compatibility
export CMAKE_POLICY_VERSION_MINIMUM=3.5

# Fix 2: If protobuf issues, use brew's version
brew install protobuf@33
export PKG_CONFIG_PATH="/opt/homebrew/Cellar/protobuf@33/33.5/lib/pkgconfig"

# Fix 3: VLA compilation errors
# Add to CMakeLists.txt or use:
export CFLAGS="-Wno-vla"
export CXXFLAGS="-Wno-vla -Wno-error=attributes"

# Then build
cd PX4-Autopilot
export CMAKE_POLICY_VERSION_MINIMUM=3.5
make px4_sitl gz_x500
```

## Step 3: Install Micro XRCE-DDS Agent

```bash
git clone https://github.com/eProsima/Micro-XRCE-DDS-Agent.git
cd Micro-XRCE-DDS-Agent
mkdir build && cd build
cmake .. -DUAGENT_SOCKETCAN_PROFILE=OFF
make -j$(nproc)
sudo make install
# or just use the binary directly: ./MicroXRCEAgent
```

## Step 4: Install MAVSDK

```bash
# Python SDK (for AerialClaw)
pip install mavsdk

# MAVSDK Server binary
# Download from: https://github.com/mavlink/MAVSDK/releases
# Or build from source
```

## Step 5: Download Gazebo Models

```bash
# PX4 Gazebo models (required for drone model)
# These are usually cloned with PX4-Autopilot

# Additional models for custom worlds
mkdir -p ~/.simulation-gazebo/models
# Download from https://github.com/PX4/PX4-gazebo-models if needed
```

## Running the Full Research Demo

This is the main path for research demonstrations. It is designed for the ACM MM-style showcase: the project boots, PX4/Gazebo boots, the Web console opens, camera/LiDAR bridge status is visible, and an LLM-configured operator can immediately try natural-language flight tasks.

### One-command quickstart

`doctor_gazebo.sh` is a readiness checker only: it prints host/PX4/Gazebo dependency status and suggested commands, but it does not start the Web service. Use `sim_quickstart.sh` when you want the actual running demo.

For the first run on a machine that already has basic host tools installed, use:

```bash
./scripts/sim_quickstart.sh --setup
```

For later runs:

```bash
./scripts/sim_quickstart.sh
```

If an earlier run is still alive and you want a clean restart:

```bash
./scripts/sim_quickstart.sh --restart
```

The default research demo uses:

```text
world: urban_rescue
model: x500_lidar_2d_cam
```

This is AerialClaw's modified UAV model, not the plain PX4 `x500`. It publishes front/rear/left/right/down camera topics plus 2D LiDAR. AerialClaw's backend sensor bridge subscribes to Gazebo Transport topics and forwards frames to the Web UI over Socket.IO. A demo machine is not considered fully configured until this model is installed, selected, and producing sensor topics.

### What the quickstart launches

`sim_quickstart.sh` coordinates the whole stack:

| Layer | Started by | Result |
|---|---|---|
| PX4/Gazebo setup | `scripts/setup_px4.sh` when `--setup` is used | PX4 checkout/build, Gazebo models/worlds, Micro XRCE-DDS Agent, MAVSDK Python dependency |
| Simulator | `scripts/start_sim.sh urban_rescue x500_lidar_2d_cam` | Micro XRCE-DDS Agent, Gazebo server, PX4 SITL |
| AerialClaw backend | `SIM_ADAPTER=px4 ... python server.py` | Web console and PX4 adapter on `http://localhost:5001` |
| PX4 control adapter | backend auto-start after `/api/init` | `/api/adapter/status` must report `adapter=px4` and `connected=true`; otherwise motor commands are not controlling PX4 |
| Sensor bridge | backend auto-start after adapter connection | camera/LiDAR frames emitted to the Web UI; `/api/sensor/camera` returns JPEG |
| LLM provider | `.env` or Web UI Model Configuration | AI mode can plan and execute natural-language flight tasks |

After the script reports success, open:

```text
http://localhost:5001
```

Recommended first demo:

1. Click **Initialize System**.
2. Check the cockpit/camera panels. The front camera should show live simulator frames after the model finishes spawning.
3. Configure an LLM provider if not already configured.
4. Switch to AI mode.
5. Try: `Take off to 15 meters and observe the surroundings.`

### LLM configuration for autonomous flight

Mock UI and manual checks do not need an LLM key. Autonomous planning does.

Fast file-based setup:

```bash
cp .env.example .env
# Edit these fields:
# ACTIVE_PROVIDER=openai
# LLM_BASE_URL=https://api.openai.com/v1
# LLM_API_KEY=...
# LLM_MODEL=gpt-4o
# VLM_BASE_URL=https://api.openai.com/v1
# VLM_API_KEY=...
# VLM_MODEL=gpt-4o

./scripts/sim_quickstart.sh --restart
```

You can also use the Web UI **Model Configuration** panel to add or switch OpenAI-compatible providers. The VLM settings are used when AerialClaw analyzes camera images.

### Verification commands

Use these when preparing a demo machine:

```bash
# Read-only preflight before starting
./scripts/doctor_gazebo.sh urban_rescue x500_lidar_2d_cam

# Start the complete stack
./scripts/sim_quickstart.sh

# Backend is alive
curl http://localhost:5001/api/status

# Sensor bridge is reachable
curl http://localhost:5001/api/sensor/status

# Live simulator/topic diagnosis
./scripts/doctor_gazebo.sh urban_rescue x500_lidar_2d_cam --live
```

Healthy signs:

- `/api/status` returns `initialized` / `current_robot` fields after initialization.
- `/api/adapter/status` returns `adapter: px4` and `connected: true`. If it says `mock` or `connected: false`, the cameras may still work, but motor commands are not controlling PX4.
- `/api/sensor/status` returns `"running": true`, `world: urban_rescue`, and `model: x500_lidar_2d_cam_0`.
- `/api/sensor/status` lists five cameras (`front/rear/left/right/down`) plus LiDAR with increasing `frame_count`.
- `/api/sensor/camera` returns `Content-Type: image/jpeg` and a non-empty JPEG file.
- `doctor_gazebo.sh --live` lists Gazebo camera/LiDAR topics.
- The Web UI cockpit/camera panels show frames instead of `NO SIGNAL`.
- Repeated runs do not accumulate PX4 `.ulg` files under `PX4-Autopilot/build/px4_sitl_default/log` unless `AERIALCLAW_KEEP_PX4_LOGS=1` is explicitly set.

### Control-debug fallback only

If the AerialClaw modified UAV model cannot be resolved on a machine, PX4's standard X500 path can validate basic flight control first:

```bash
./scripts/sim_quickstart.sh default x500
```

The standard `x500` fallback is useful only for PX4 control debugging. It is **not** the research showcase and should not be used for paper demos or artifact checks. For the real AerialClaw demo, fix the `x500_lidar_2d_cam` model installation and return to the default quickstart.

### Manual Start (advanced)

Use this only when debugging the scripts.

```bash
# Terminal 1: DDS Agent
MicroXRCEAgent udp4 -p 8888

# Terminal 2: Gazebo (headless)
export PX4_GZ_MODELS="<PX4_DIR>/Tools/simulation/gz/models"
export PX4_GZ_WORLDS="<PX4_DIR>/Tools/simulation/gz/worlds"
export GZ_SIM_RESOURCE_PATH="$HOME/.simulation-gazebo/models:$PX4_GZ_MODELS:$PX4_GZ_WORLDS"
gz sim --verbose=1 -r -s "${PX4_GZ_WORLDS}/urban_rescue.sdf"
# gz sim -g  # GUI (optional, separate terminal)

# Terminal 3: PX4 SITL (STANDALONE mode — connects to already-running Gazebo)
export PX4_SYS_AUTOSTART=4001
export PX4_SIMULATOR=gz
export PX4_GZ_WORLD=urban_rescue
export PX4_SIM_MODEL=x500_lidar_2d_cam
export PX4_GZ_STANDALONE=1
PX4_BUILD="<PX4_DIR>/build/px4_sitl_default"
cd "$PX4_BUILD"
./bin/px4 "$PX4_BUILD" -s "${PX4_BUILD}/etc/init.d-posix/rcS"

# Terminal 4: AerialClaw
# macOS/Homebrew Gazebo Python bindings are installed outside the venv, so expose
# only the Gazebo Cellar site-packages paths. Without this, /api/sensor/status
# will say the bridge is not running and camera panels will show NO SIGNAL.
export GZ_PYTHONPATH="/opt/homebrew/Cellar/gz-transport13/13.5.0_8/lib/python3.12/site-packages:/opt/homebrew/Cellar/gz-msgs10/10.3.2_24/lib/python3.12/site-packages:/opt/homebrew/Cellar/gz-math7/7.5.2_3/lib/python3.12/site-packages"
PYTHONPATH="$GZ_PYTHONPATH:${PYTHONPATH:-}" \
  SIM_ADAPTER=px4 PX4_GZ_WORLD=urban_rescue PX4_SIM_MODEL=x500_lidar_2d_cam \
  python server.py
# Open http://localhost:5001
```

> **Important**: Use STANDALONE mode (`PX4_GZ_STANDALONE=1`) with the PX4 SITL binary produced by your local PX4 build directly. The `make px4_sitl gz_x500` shortcut is useful as a PX4 build test, but the repository scripts are the recommended AerialClaw path because they also install worlds/models and print diagnostics.

## Sensor Configuration

### Default Sensor Setup

The default user path uses PX4/Gazebo standard `x500`. Camera/LiDAR streaming requires a sensor-enabled Gazebo model. If you provide a local custom model, set `PX4_SIM_MODEL` to that model name and ensure Gazebo can resolve its SDF files.
The Web console does **not** read Gazebo topics directly. `sim/gz_sensor_bridge.py`
subscribes to Gazebo Transport camera/LiDAR topics and `server.py` forwards the
latest frames to the UI via Socket.IO events (`sensor_cameras`, `sensor_lidar`).

### Customizing Sensors

To modify the sensor configuration:
1. Add or edit your PX4/Gazebo model SDF files in a Gazebo model path.
2. Modify camera/LiDAR parameters in the model files if needed.
3. Update topic names in `sim/gz_sensor_bridge.py` if your sensor/link names change.

## Key PX4 Parameters

| Parameter | Value | Purpose |
|-----------|-------|---------|
| `PX4_SYS_AUTOSTART` | 4001 | Gazebo X500 airframe |
| `COM_DISARM_PRFLT` | 10 | Auto-disarm after 10s if no takeoff |
| `MAV_SYS_ID` | 1 | MAVLink system ID |

**Important**: After arming, send the takeoff command within 10 seconds, or PX4 will auto-disarm.

## MAVLink Ports

| Port | Protocol | Purpose |
|------|----------|---------|
| 14540 | UDP | MAVSDK / Offboard control |
| 14550 | UDP | QGroundControl |

## Coordinate System

AerialClaw uses **NED** (North-East-Down):
- Gazebo uses ENU (X=East, Y=North, Z=Up)
- Conversion: `NED.North = Gazebo.Y, NED.East = Gazebo.X, NED.Down = -Gazebo.Z`
- All positions in `robot_profile/WORLD_MAP.md` and skill parameters use NED

## Troubleshooting

### Gazebo won't start
- Check `gz sim --version` works
- Ensure `GZ_SIM_RESOURCE_PATH` includes model directories
- Try `gz sim --verbose=4` for detailed logs

### PX4 can't connect to Gazebo
- Make sure Gazebo is running before PX4 (or use `make px4_sitl gz_x500`)
- Check `PX4_GZ_STANDALONE=1` is set when using manual start
- Verify DDS Agent is running: `MicroXRCEAgent udp4 -p 8888`

### Web console camera/LiDAR panels show `NO SIGNAL`

`gz topic -l` showing camera or LiDAR topics only proves Gazebo is publishing.
AerialClaw still needs the backend sensor bridge to subscribe to those topics and
push Socket.IO frames to the browser.

Check the full path:

```bash
# Gazebo should expose camera/LiDAR topics for the AerialClaw model
gz topic -l | grep -Ei 'urban_rescue|x500_lidar_2d_cam|camera|image|lidar|scan'

# After the backend connects, the bridge should be running
curl http://localhost:5001/api/sensor/status

# The camera HTTP endpoint should return a JPEG, not HTML/500/503
curl -fsS http://localhost:5001/api/sensor/camera -o /tmp/aerialclaw_camera.jpg
file /tmp/aerialclaw_camera.jpg
```

Expected backend log after the adapter connects:

```text
Gazebo sensor bridge started: world=urban_rescue model=x500_lidar_2d_cam_0 topics=[...]
传感器数据推送线程启动成功
```

If status says the bridge is unavailable:
- install / expose Gazebo Harmonic Python bindings (`gz.transport13`, `gz.msgs10`, and on macOS often `gz-math7`)
- prefer `./scripts/sim_quickstart.sh`, which discovers these paths and exports `PYTHONPATH` for the backend automatically
- verify `PX4_GZ_WORLD=urban_rescue` matches your running world
- verify `PX4_SIM_MODEL=x500_lidar_2d_cam` matches the spawned model base name; PX4/Gazebo commonly appends `_0` to the spawned model
- if `/api/sensor/camera` returns 500 with `ModuleNotFoundError: cv2`, install the pinned dependency from `requirements.txt` (`opencv-python-headless==4.10.0.84`, keeping `numpy<2` for PX4/symforce compatibility)
- if you renamed links/sensors, update `sim/gz_sensor_bridge.py` topic templates

### PX4 ULog / disk usage safety

PX4 normally writes persistent `.ulg` flight logs under the SITL build tree, for example:

```text
PX4-Autopilot/build/px4_sitl_default/log/YYYY-MM-DD/*.ulg
```

Those files can grow by tens or hundreds of MB per run. For user-facing demos, `scripts/start_sim.sh` disables PX4 ULog persistence and removes stale `.ulg` files by default. This prevents repeated simulator sessions from filling the user's disk.

If you intentionally need PX4 ULog files for flight-controller debugging:

```bash
AERIALCLAW_KEEP_PX4_LOGS=1 ./scripts/sim_quickstart.sh --restart
```

Manual cleanup command:

```bash
find PX4-Autopilot/build -type f -name '*.ulg' -delete
```

Console logs are also bounded by `AERIALCLAW_LOG_LIMIT_BYTES` (default 50 MiB per process) to avoid `/tmp` growth from noisy PX4/Gazebo output.

### Web UI looks connected, but the drone does not move

Check the flight-control adapter, not just the camera bridge:

```bash
curl http://localhost:5001/api/adapter/status
```

Expected:

```json
{"adapter":"px4","connected":true}
```

If the adapter is `mock` or disconnected, restart through quickstart:

```bash
./scripts/sim_quickstart.sh --restart
```

`sim_quickstart.sh` waits for the MAVSDK control link before starting the backend and fails the run if the PX4 adapter does not connect. This prevents the dangerous mixed state where Gazebo camera frames are live but motor skills silently execute against mock control.

### MAVSDK connection fails
- Start `mavsdk_server` separately, don't rely on auto-start
- Don't kill `mavsdk_server` when restarting `server.py`
- Check port 14540 is not occupied: `lsof -i :14540`

### macOS: PX4 build fails
- See "macOS ARM64 Known Issues" above
- Python version: use 3.10-3.12 (3.14 may have dataclass compatibility issues)
- Proxy for GitHub: `export https_proxy=http://127.0.0.1:7897` if needed
