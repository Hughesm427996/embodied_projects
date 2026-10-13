"""Read a local API key as data; never execute a shell configuration file."""
from __future__ import annotations

import os
from pathlib import Path
import stat
import tempfile


def key_path(environ=None, home=None):
    environ = os.environ if environ is None else environ
    configured = environ.get("XDG_CONFIG_HOME", "")
    base = Path(configured) if configured and Path(configured).is_absolute() else (
        Path.home() if home is None else Path(home)
    ) / ".config"
    return base / "agentchord_console" / "deepseek_api_key"


def validated_key(value):
    key = value.strip()
    if not key or len(key) > 4096 or not key.isascii() or any(ch.isspace() for ch in key):
        raise RuntimeError("密钥为空或格式不正确，请重新隐藏输入完整 API 密钥。")
    return key


def _check_private_file(info):
    if not stat.S_ISREG(info.st_mode):
        raise RuntimeError("本地密钥配置必须是普通文件。")
    if os.name == "posix":
        if info.st_uid != os.getuid() or stat.S_IMODE(info.st_mode) & 0o077:
            raise RuntimeError("本地密钥文件权限不正确，请运行 --save-api-key 重新保存。")


def load_saved_key(path=None):
    path = key_path() if path is None else Path(path)
    try:
        info = path.lstat()
    except FileNotFoundError:
        return None
    if stat.S_ISLNK(info.st_mode):
        raise RuntimeError("本地密钥配置不能是符号链接。")
    _check_private_file(info)
    fd = os.open(path, os.O_RDONLY | getattr(os, "O_NOFOLLOW", 0))
    with os.fdopen(fd, "r", encoding="utf-8") as stream:
        _check_private_file(os.fstat(stream.fileno()))
        return validated_key(stream.read(4098))


def save_key(value, path=None):
    key = validated_key(value)
    path = key_path() if path is None else Path(path)
    path.parent.mkdir(mode=0o700, parents=True, exist_ok=True)
    info = path.parent.lstat()
    if not stat.S_ISDIR(info.st_mode) or stat.S_ISLNK(info.st_mode):
        raise RuntimeError("密钥配置目录必须是普通目录。")
    if os.name == "posix":
        if info.st_uid != os.getuid():
            raise RuntimeError("密钥配置目录不属于当前用户。")
        path.parent.chmod(0o700)
    if path.is_symlink():
        raise RuntimeError("本地密钥配置不能是符号链接。")
    fd, name = tempfile.mkstemp(prefix=".key-", dir=path.parent)
    temporary = Path(name)
    try:
        with os.fdopen(fd, "w", encoding="utf-8", newline="\n") as stream:
            if os.name == "posix":
                os.fchmod(stream.fileno(), 0o600)
            stream.write(key + "\n")
            stream.flush()
            os.fsync(stream.fileno())
        os.replace(temporary, path)
    finally:
        temporary.unlink(missing_ok=True)
    return path


def resolve_key(environ=None, path=None):
    environ = os.environ if environ is None else environ
    for name in ("DEEPSEEK_API_KEY", "OPENAI_API_KEY"):
        if environ.get(name):
            return validated_key(environ[name])
    return load_saved_key(key_path(environ) if path is None else path)
