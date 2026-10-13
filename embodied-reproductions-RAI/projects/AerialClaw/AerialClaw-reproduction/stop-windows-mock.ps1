$ErrorActionPreference = "Stop"
$repo = Join-Path $PSScriptRoot "AerialClaw"
$pidFile = Join-Path $PSScriptRoot "server.pid"

if (-not (Test-Path -LiteralPath $pidFile)) {
    Write-Host "No managed AerialClaw server PID was found."
    exit 0
}

$serverPid = [int](Get-Content -LiteralPath $pidFile)
$process = Get-Process -Id $serverPid -ErrorAction SilentlyContinue
if ($null -eq $process) {
    Remove-Item -LiteralPath $pidFile -Force
    Write-Host "AerialClaw server is already stopped."
    exit 0
}

$expected = Join-Path $repo ".venv\Scripts\python.exe"
if ($process.Path -ne $expected) {
    throw "PID $serverPid does not belong to this AerialClaw environment."
}

Stop-Process -Id $serverPid
Remove-Item -LiteralPath $pidFile -Force
Write-Host "AerialClaw Windows Mock stopped."

