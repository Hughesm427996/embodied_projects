@echo off
chcp 65001 >nul
setlocal
set "SCRIPT=%~dp0install-ubuntu-2204-d-drive.ps1"
if not exist "%SCRIPT%" (
  echo [ERROR] Installer script was not found:
  echo %SCRIPT%
  pause
  exit /b 1
)
fltmc >nul 2>&1
if errorlevel 1 (
  echo Requesting administrator access. Click Yes in the UAC window.
  powershell.exe -NoProfile -ExecutionPolicy Bypass -Command "Start-Process -FilePath '%~f0' -Verb RunAs"
  if errorlevel 1 (
    echo [ERROR] The administrator window could not be started.
    pause
    exit /b 1
  )
  exit /b
)
echo Installer: %SCRIPT%
powershell.exe -NoProfile -ExecutionPolicy Bypass -File "%SCRIPT%"
set "RESULT=%ERRORLEVEL%"
echo.
echo Installer finished with exit code %RESULT%.
echo Keep this window open and report the result to Codex.
pause
exit /b %RESULT%
