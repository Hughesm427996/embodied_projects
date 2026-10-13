# Run Checklist

This checklist summarizes the evidence package for ACM Multimedia Open Source Software Track users.

## Quick quick path

Use `QUICKSTART.md` first. It describes a five-minute mock mode evaluation path that does not require PX4, Gazebo, AirSim, GPUs, or real UAV hardware.

Expected quick checks:

```bash
# Recommended prebuilt container path (Compose, no local build)
docker compose up
curl http://localhost:5001/api/status

# Equivalent plain Docker path
docker run --rm -p 5001:5001 xdei/aerialclaw:mock
curl http://localhost:5001/api/status

# Developer-only local build fallback
docker compose -f compose.build.yml up --build

# Optional heavier Gazebo direct image
docker compose -f compose.gazebo.yml up

# Local fallback path (Windows PowerShell)
.\scripts\smoke_mock.ps1
$env:SIM_ADAPTER="mock"; python server.py

# Local fallback path (macOS / Linux / WSL2)
python -m compileall -q .
python -m pytest
bash scripts/smoke_mock.sh
SIM_ADAPTER=mock python server.py
```

## Run checks included in this repository

- `pyproject.toml` configures pytest so tests run without manually setting `PYTHONPATH`.
- `tests/test_mock_adapter.py` validates the pure in-memory UAV adapter.
- `tests/test_adapter_manager.py` validates adapter registration and switching behavior.
- `tests/test_server_smoke.py` validates Flask app import and `/api/status`.
- `tests/test_repository_consistency.py` runs the repository consistency checker.
- `scripts/check_repository.py` fails on stale documentation references, missing repository files, or accidental absolute-path duplicate trees.
- `scripts/smoke_mock.sh` runs the Unix local mock demo smoke gate.
- `scripts/smoke_mock.ps1` runs the Windows PowerShell local mock demo smoke gate.
- `.gitattributes` pins shell scripts to LF so Windows checkouts do not break Bash scripts with CRLF.
- `scripts/doctor_gazebo.sh` checks the optional PX4/Gazebo path and prints actionable next steps without modifying the system.
- `.github/workflows/ci.yml` runs repository checks, Python compile, pytest, Web UI lint/build, Docker image build, and a Docker `/api/status` smoke test.
- `.github/workflows/docker-images.yml` publishes prebuilt GHCR images for mock and Gazebo paths.
- GHCR workflow remains available for future organization-owned images; package visibility must be public before switching user docs to GHCR.
- `Dockerfile` builds a lightweight mock mode image using `requirements-mock.txt`.
- `Dockerfile.gazebo` builds the heavier Gazebo direct demo image.
- `compose.yml` provides a prebuilt `docker compose up` user path with a `/api/status` healthcheck.
- `compose.build.yml` is the developer-only local build fallback.
- `compose.gazebo.yml` starts the prebuilt Gazebo direct image.

## Scope boundaries

The quick repository package is intentionally mock mode. PX4/Gazebo is the guided second path: it is supported by repository scripts, but it still requires heavyweight OS-level simulator dependencies. AirSim/OpenFly remains an external integration path and is not the public default quick path.

For PX4/Gazebo validation, use:

```bash
bash scripts/doctor_gazebo.sh urban_rescue x500_lidar_2d_cam
bash scripts/setup_px4.sh
bash scripts/start_sim.sh urban_rescue x500_lidar_2d_cam
SIM_ADAPTER=px4 PX4_GZ_WORLD=urban_rescue PX4_SIM_MODEL=x500_lidar_2d_cam python server.py
curl http://localhost:5001/api/status
curl http://localhost:5001/api/sensor/status
bash scripts/doctor_gazebo.sh urban_rescue x500_lidar_2d_cam --live
```

The public repository should not claim shipped SDKs for clients that are only protocol examples/planned integration targets. Device integration should be evaluated through `docs/DEVICE_PROTOCOL.md` unless a concrete SDK is present in the repository.

## Expected local gate result

At the time of hardening, the local smoke gate is expected to report:

- `python scripts/check_repository.py` — pass
- `python -m compileall -q .` — pass
- `python -m pytest` — all tests pass
- `cd ui && npm run lint` — pass with no warnings/errors
- `cd ui && npm run build` — pass
- `docker build -t aerialclaw:ci .` — pass when Docker daemon/network are available
- `docker run --rm -p 5001:5001 aerialclaw:ci` + `curl /api/status` — pass
- `docker compose config` — pass for prebuilt mock path
- `docker compose -f compose.build.yml config` — pass for developer build fallback
- `docker compose -f compose.gazebo.yml config` — pass for Gazebo direct image path

## Remaining known limitations

- The prebuilt Gazebo direct image is heavier than the mock image and is intended for simulator demos; full PX4/Gazebo repeatability remains a guided integration validation path.
- AirSim/OpenFly validation depends on external simulator assets and should not be presented as the public default quick path.
- The test suite is now suitable for smoke evaluation, but not yet a comprehensive safety/flight-control verification suite.
