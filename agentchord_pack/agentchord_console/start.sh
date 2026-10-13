#!/usr/bin/env bash
set -euo pipefail
script_dir="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd)"
project_dir="$(cd -- "$script_dir/.." && pwd)"
if [[ ! -x "$project_dir/.venv/bin/python" ]]; then
  echo "请把 agentchord_console 文件夹放到已经跑通的 agentchord_pack 项目里面。"
  exit 1
fi
cd -- "$project_dir"
exec "$project_dir/.venv/bin/python" "$script_dir/console.py" "$@"
