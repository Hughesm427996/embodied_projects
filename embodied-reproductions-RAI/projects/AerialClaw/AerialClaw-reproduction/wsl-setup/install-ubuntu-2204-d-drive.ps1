$ErrorActionPreference = 'Stop'

$setupRoot = 'D:\codex\AerialClaw-reproduction\wsl-setup'
$installRoot = 'D:\WSL\Ubuntu-22.04'
$distroName = 'Ubuntu-22.04'
$logPath = Join-Path $setupRoot 'install-ubuntu-2204.log'

New-Item -ItemType Directory -Force -Path $setupRoot | Out-Null
Start-Transcript -LiteralPath $logPath -Force

try {
    $principal = [Security.Principal.WindowsPrincipal]::new(
        [Security.Principal.WindowsIdentity]::GetCurrent()
    )
    if (-not $principal.IsInRole([Security.Principal.WindowsBuiltInRole]::Administrator)) {
        throw 'Please run this script as Administrator.'
    }

    Write-Output 'STEP 1/4: Verify Windows features'
    $wslFeature = Get-WindowsOptionalFeature -Online -FeatureName Microsoft-Windows-Subsystem-Linux
    $vmFeature = Get-WindowsOptionalFeature -Online -FeatureName VirtualMachinePlatform
    $wslFeature | Select-Object FeatureName, State | Format-Table -AutoSize
    $vmFeature | Select-Object FeatureName, State | Format-Table -AutoSize
    if ($wslFeature.State -ne 'Enabled' -or $vmFeature.State -ne 'Enabled') {
        throw 'Required WSL2 features are not enabled.'
    }

    Write-Output 'STEP 2/4: Verify WSL2 runtime'
    & wsl.exe --status
    if ($LASTEXITCODE -ne 0) { throw "wsl --status failed with exit code $LASTEXITCODE" }
    & wsl.exe --set-default-version 2
    if ($LASTEXITCODE -ne 0) { throw "Setting WSL2 default failed with exit code $LASTEXITCODE" }

    Write-Output 'STEP 3/4: Check existing distributions'
    $existing = @(& wsl.exe --list --quiet) | ForEach-Object { ($_ -replace "`0", '').Trim() } | Where-Object { $_ }
    if ($existing -contains $distroName) {
        Write-Output "$distroName already exists; installation was skipped."
    } else {
        if (Test-Path -LiteralPath $installRoot) {
            $entries = @(Get-ChildItem -LiteralPath $installRoot -Force)
            if ($entries.Count -gt 0) {
                throw "$installRoot exists and is not empty. No files were changed there."
            }
        }
        New-Item -ItemType Directory -Force -Path $installRoot | Out-Null

        Write-Output 'STEP 4/4: Download and install Ubuntu 22.04 on D drive'
        & wsl.exe --install Ubuntu-22.04 --name $distroName --location $installRoot --version 2 --web-download --no-launch
        if ($LASTEXITCODE -ne 0) { throw "Ubuntu installation failed with exit code $LASTEXITCODE" }
    }

    Write-Output 'VERIFY: Distribution registration and version'
    & wsl.exe --list --verbose
    if ($LASTEXITCODE -ne 0) { throw "WSL verification failed with exit code $LASTEXITCODE" }

    Write-Output 'INSTALL_RESULT=SUCCESS'
}
catch {
    Write-Error $_
    Write-Output 'INSTALL_RESULT=FAILED'
    exit 1
}
finally {
    Stop-Transcript
}

