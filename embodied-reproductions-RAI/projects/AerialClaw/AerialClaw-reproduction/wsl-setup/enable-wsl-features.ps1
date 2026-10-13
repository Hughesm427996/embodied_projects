$ErrorActionPreference = 'Stop'
$results = @()
$results += Enable-WindowsOptionalFeature -Online -FeatureName Microsoft-Windows-Subsystem-Linux -All -NoRestart
$results += Enable-WindowsOptionalFeature -Online -FeatureName VirtualMachinePlatform -All -NoRestart
$results | Select-Object FeatureName,State,RestartNeeded | Format-Table -AutoSize | Out-File -LiteralPath 'D:\codex\AerialClaw-reproduction\wsl-setup\enable-wsl-features.log' -Encoding utf8
