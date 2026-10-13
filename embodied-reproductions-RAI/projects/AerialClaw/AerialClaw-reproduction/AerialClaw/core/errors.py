"""
core/errors.py — AerialClaw 统一异常体系

所有自定义异常继承 AerialClawError，携带修复建议和文档链接。
"""

from __future__ import annotations


class AerialClawError(Exception):
    """AerialClaw 基础异常，所有自定义异常继承它"""

    def __init__(self, message: str, fix_hint: str = "", doc_link: str = ""):
        self.message = message
        self.fix_hint = fix_hint
        self.doc_link = doc_link
        super().__init__(self._format())

    def _format(self) -> str:
        parts = [f"❌ {self.message}"]
        if self.fix_hint:
            parts.append(f"   → 修复建议: {self.fix_hint}")
        if self.doc_link:
            parts.append(f"   → 参考文档: {self.doc_link}")
        return "\n".join(parts)


# ── LLM 相关 ──────────────────────────────────────────────

class LLMConfigError(AerialClawError):
    """LLM 配置错误（Key 缺失、地址无效等）"""

class LLMConnectionError(AerialClawError):
    """LLM API 连接失败"""

class LLMResponseError(AerialClawError):
    """LLM 返回格式异常"""


# ── 适配器相关 ────────────────────────────────────────────

class AdapterConnectionError(AerialClawError):
    """硬件适配器连接失败"""

class AdapterTimeoutError(AerialClawError):
    """适配器操作超时"""


# ── 安全相关 ──────────────────────────────────────────────

class SafetyViolationError(AerialClawError):
    """安全包线违规（超速/超高/出围栏等）"""

class CommandBlockedError(AerialClawError):
    """命令被安全过滤器拦截"""

class ApprovalRequiredError(AerialClawError):
    """操作需要人工确认"""


# ── 设备相关 ──────────────────────────────────────────────

class DeviceNotFoundError(AerialClawError):
    """设备未注册或已断连"""

class DeviceTimeoutError(AerialClawError):
    """设备心跳超时"""


# ── 沙箱相关 ──────────────────────────────────────────────

class SandboxExecutionError(AerialClawError):
    """沙箱代码执行失败"""

class SandboxTimeoutError(AerialClawError):
    """沙箱执行超时"""


# ── 记忆相关 ──────────────────────────────────────────────

class MemoryStoreError(AerialClawError):
    """记忆存储失败"""

class MemoryRetrievalError(AerialClawError):
    """记忆检索失败"""
