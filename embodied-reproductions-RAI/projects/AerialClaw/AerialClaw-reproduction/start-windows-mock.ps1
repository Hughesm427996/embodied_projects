$ErrorActionPreference = "Stop"
$repo = Join-Path $PSScriptRoot "AerialClaw"
$python = Join-Path $repo ".venv\Scripts\python.exe"

if (-not (Test-Path -LiteralPath $python)) {
    throw "Python virtual environment not found: $python"
}

$env:SIM_ADAPTER = "mock"
$env:PYTHONUTF8 = "1"
$env:PYTHONIOENCODING = "utf-8"

Set-Location -LiteralPath $repo
Write-Host "AerialClaw Windows Mock: http://127.0.0.1:5001"
& $python -X utf8 server.py

