$log = 'D:\codex\AerialClaw-reproduction\wsl-setup\enable-wsl-features-admin.log'
Start-Transcript -LiteralPath $log -Force
try {
  Write-Output "Admin=$([Security.Principal.WindowsPrincipal][Security.Principal.WindowsIdentity]::GetCurrent()).IsInRole([Security.Principal.WindowsBuiltInRole]::Administrator)"
  & dism.exe /online /enable-feature /featurename:Microsoft-Windows-Subsystem-Linux /all /norestart
  $code1 = $LASTEXITCODE
  & dism.exe /online /enable-feature /featurename:VirtualMachinePlatform /all /norestart
  $code2 = $LASTEXITCODE
  Write-Output "DISM_EXIT_WSL=$code1"
  Write-Output "DISM_EXIT_VMP=$code2"
} catch { Write-Error $_; exit 1 } finally { Stop-Transcript }
if ($code1 -ne 0 -or $code2 -ne 0) { exit 2 }
exit 0
