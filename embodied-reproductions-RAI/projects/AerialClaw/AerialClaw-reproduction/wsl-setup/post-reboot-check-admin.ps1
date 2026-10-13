$log='D:\codex\AerialClaw-reproduction\wsl-setup\post-reboot-check-admin.log'
Start-Transcript -LiteralPath $log -Force
try {
  Write-Output '=== IDENTITY ==='
  whoami.exe
  Write-Output '=== FEATURES ==='
  Get-WindowsOptionalFeature -Online -FeatureName Microsoft-Windows-Subsystem-Linux | Select-Object FeatureName,State
  Get-WindowsOptionalFeature -Online -FeatureName VirtualMachinePlatform | Select-Object FeatureName,State
  Write-Output '=== WSL STATUS ==='
  wsl.exe --status
  Write-Output "WSL_STATUS_EXIT=$LASTEXITCODE"
  Write-Output '=== DISTROS ==='
  wsl.exe --list --verbose
  Write-Output "WSL_LIST_EXIT=$LASTEXITCODE"
  Write-Output '=== VIRTUALIZATION ==='
  Get-CimInstance Win32_Processor | Select-Object Name,VirtualizationFirmwareEnabled,VMMonitorModeExtensions | Format-List
  Get-CimInstance Win32_ComputerSystem | Select-Object HypervisorPresent | Format-List
  Write-Output '=== NETWORK ==='
  curl.exe -I -L --max-time 20 https://github.com
  Write-Output "GITHUB_CURL_EXIT=$LASTEXITCODE"
  curl.exe -I -L --max-time 20 https://aka.ms/wslubuntu2204
  Write-Output "UBUNTU_CURL_EXIT=$LASTEXITCODE"
} finally { Stop-Transcript }
