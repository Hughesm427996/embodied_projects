from pathlib import Path


def test_readme_documents_all_runnable_user_paths():
    readme = Path("README.md").read_text(encoding="utf-8")
    for expected in [
        "docker compose up",
        "docker run --rm -p 5001:5001 xdei/aerialclaw:mock",
        "docker compose -f compose.build.yml up --build",
        "SIM_ADAPTER=mock python server.py",
        "bash scripts/smoke_mock.sh",
        "./scripts/sim_quickstart.sh --setup",
        "./scripts/sim_quickstart.sh",
        "docs/SIMULATION_SETUP.md",
        "curl http://localhost:5001/api/status",
    ]:
        assert expected in readme

    for unverified in [
        "bash scripts/setup_px4.sh",
        "bash scripts/start_sim.sh",
        "wsl --install -d Ubuntu-24.04",
    ]:
        assert unverified not in readme


def test_compose_user_path_exists_and_uses_mock_adapter():
    compose = Path("compose.yml")
    assert compose.exists()
    text = compose.read_text(encoding="utf-8")
    for expected in [
        "SIM_ADAPTER: mock",
        "5001:5001",
        "xdei/aerialclaw:mock",
        "/api/status",
    ]:
        assert expected in text


def test_readme_project_tree_does_not_claim_runtime_profile_files_are_shipped():
    for path in ["README.md", "README_CN.md"]:
        text = Path(path).read_text(encoding="utf-8")
        assert "MEMORY.md / SKILLS.md" not in text
        assert "robot_profile/MEMORY.md" not in text
        assert "robot_profile/SKILLS.md" not in text


def test_docker_image_workflow_and_compose_files_are_split_by_runtime():
    workflow = Path(".github/workflows/docker-images.yml").read_text(encoding="utf-8")
    for expected in [
        "file: Dockerfile",
        "file: Dockerfile.gazebo",
        "ghcr.io/xdei-group/aerialclaw:mock",
        "ghcr.io/xdei-group/aerialclaw:gazebo",
    ]:
        assert expected in workflow

    gazebo_compose = Path("compose.gazebo.yml").read_text(encoding="utf-8")
    for expected in [
        "SIM_ADAPTER: gazebo_direct",
        "PX4_GZ_WORLD: urban_rescue",
        "ghcr.io/xdei-group/aerialclaw:gazebo",
        "5001:5001",
    ]:
        assert expected in gazebo_compose

    build_compose = Path("compose.build.yml").read_text(encoding="utf-8")
    assert "build:" in build_compose
    assert "image: aerialclaw:demo" in build_compose
