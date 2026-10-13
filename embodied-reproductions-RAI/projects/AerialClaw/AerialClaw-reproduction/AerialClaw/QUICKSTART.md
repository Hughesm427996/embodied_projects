# AerialClaw Quickstart

This quickstart has two levels:

1. **Mock mode** for a fast Web UI/API smoke test.
2. **PX4 + Gazebo full simulator mode** for the research demo: SITL flight, camera/LiDAR bridge, Web UI, and LLM-driven autonomous commands.

## 1. Container mock mode

No local image build is required.

```bash
git clone https://github.com/XDEI-Group/AerialClaw.git
cd AerialClaw
docker compose up

# Equivalent plain Docker command:
# docker run --rm -p 5001:5001 xdei/aerialclaw:mock
```

Verify:

```bash
curl http://localhost:5001/api/status
```

Open:

```text
http://localhost:5001
```

## 2. Local mock development

```bash
git clone https://github.com/XDEI-Group/AerialClaw.git
cd AerialClaw

python3 -m venv venv
# Linux note: the full PX4/Gazebo demo needs the system Gazebo Python bindings
# (gz.transport / gz.msgs), which a plain venv cannot see. On Linux create the
# venv with system packages visible:  python3 -m venv venv --system-site-packages
source venv/bin/activate
python -m pip install --upgrade pip
pip install -r requirements.txt
pip install pytest
python -m pytest

cd ui
npm install
npm run build
cd ..

SIM_ADAPTER=mock python server.py
```

Verify in another terminal:

```bash
curl http://localhost:5001/api/status
```

Optional smoke gate:

```bash
bash scripts/smoke_mock.sh
```

## 3. PX4 + Gazebo full simulator research demo

Use this path for the complete showcase: AerialClaw Web UI, PX4/Gazebo SITL, AerialClaw's modified UAV model `x500_lidar_2d_cam`, camera/LiDAR bridge, and LLM-driven autonomous flight.

First run on a simulator host:

```bash
git clone https://github.com/XDEI-Group/AerialClaw.git
cd AerialClaw
./scripts/sim_quickstart.sh --setup
```

Later runs:

```bash
./scripts/sim_quickstart.sh
```

Open:

```text
http://localhost:5001
```

Then initialize the system, check camera panels, configure an LLM provider, switch to AI mode, and try:

```text
Take off to 15 meters and observe the surroundings.
```

Details: [docs/SIMULATION_SETUP.md](docs/SIMULATION_SETUP.md)
