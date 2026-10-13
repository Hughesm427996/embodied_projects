#!/usr/bin/env python3
"""Copy a lab-local RAI checkout into this Git backup without caches or secrets."""

from __future__ import annotations

import json
import os
import re
import shutil
import subprocess
import sys
from datetime import datetime, timezone
from pathlib import Path


EXCLUDE_DIRS = {
    ".git", ".venv", "venv", "node_modules", "__pycache__", ".cache",
    ".pytest_cache", ".mypy_cache", ".ruff_cache", "build", "install",
    "log", "dist", "_video_libs",
}
EXCLUDE_FILES = {".env", ".env.local", ".env.production", "credentials.json", "secrets.json"}
SECRET_NAME = re.compile(r"(^id_(?:rsa|ed25519)$|\.(?:pem|key|p12|pfx)$|(?:^|[_-])(?:secret|token|password)(?:[_-]|\.)\w*)", re.I)
SECRET_VALUE = re.compile(rb"(?:sk-[A-Za-z0-9_-]{20,}|gh[pousr]_[A-Za-z0-9]{20,}|AKIA[0-9A-Z]{16})")


def run(*args: str, cwd: Path) -> None:
    subprocess.run(args, cwd=cwd, check=True)


def main() -> None:
    repo = Path(__file__).resolve().parent
    source = Path(sys.argv[1] if len(sys.argv) > 1 else "/home/xiaoyu/rai").expanduser().resolve()
    dest = repo / "projects" / "RAI"
    temp = repo / "projects" / ".RAI-import.tmp"
    if not (repo / ".git").is_dir() or not (repo / "projects" / "AerialClaw").is_dir():
        raise SystemExit("Run this script from the restored clean backup repository.")
    if not (source / "README.md").is_file() or not (source / "src").is_dir():
        raise SystemExit(f"RAI source not found at {source}")
    if dest.exists() or temp.exists():
        raise SystemExit("RAI destination already exists; inspect it before retrying.")
    if subprocess.run(["git", "status", "--porcelain"], cwd=repo, capture_output=True, text=True, check=True).stdout.strip():
        raise SystemExit("Backup repository has uncommitted changes; inspect them before importing.")

    files: list[tuple[Path, Path]] = []
    excluded: list[str] = []
    for current, dirs, names in os.walk(source, followlinks=False):
        dirs[:] = [d for d in dirs if d not in EXCLUDE_DIRS and not (Path(current) / d).is_symlink()]
        for name in names:
            item = Path(current) / name
            rel = item.relative_to(source)
            if item.is_symlink() or name in EXCLUDE_FILES or name.startswith(".env.") or SECRET_NAME.search(name):
                excluded.append(str(rel))
                continue
            if not item.is_file():
                continue
            # Scan small files for recognizable live credential formats without showing values.
            if item.stat().st_size <= 2_000_000 and SECRET_VALUE.search(item.read_bytes()):
                raise SystemExit(f"Potential credential in {rel}; inspect source and retry after removing it.")
            files.append((item, rel))
    if not files:
        raise SystemExit("No RAI files found; nothing imported.")

    temp.mkdir(parents=True)
    for item, rel in files:
        target = temp / rel
        target.parent.mkdir(parents=True, exist_ok=True)
        shutil.copy2(item, target)
    temp.rename(dest)

    manifest_path = repo / "manifest.json"
    manifest = json.loads(manifest_path.read_text(encoding="utf-8"))
    manifest["projects"].append({
        "name": "RAI", "path": "projects/RAI", "source": str(source),
        "status": "Imported from 2080; reproduction results have not been independently validated by this backup script.",
        "files": len(files), "imported_utc": datetime.now(timezone.utc).isoformat(),
    })
    manifest.pop("pending_2080_import", None)
    manifest["rai_excluded_file_count"] = len(excluded)
    manifest_path.write_text(json.dumps(manifest, ensure_ascii=False, indent=2) + "\n", encoding="utf-8")
    run("git", "add", "projects/RAI", "manifest.json", cwd=repo)
    run("git", "-c", "user.name=Codex Backup", "-c", "user.email=backup@local.invalid",
        "commit", "-m", "Import RAI from lab 2080", cwd=repo)
    print(f"Imported {len(files)} RAI files; excluded {len(excluded)} sensitive or generated files.")
    print("Repository:", repo)


if __name__ == "__main__":
    main()
