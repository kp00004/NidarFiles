<#
Copies the Jetson-side parts of NidarFiles to the Jetson over SSH:
onboard-autonomy, missions, scripts and README.md -> ~/NidarFiles on the Jetson.

Usage (from any folder):
    .\scripts\gcs\deploy_to_jetson.ps1 -JetsonHost 10.90.220.112 -User <jetson-user>

Then on the Jetson (first time): ~/NidarFiles/scripts/jetson/setup_jetson.sh
After later code changes, re-run this script and then setup_jetson.sh again
(it rebuilds the overlay workspace).

Needs the Windows OpenSSH client (ssh/scp), present on Windows 10/11.
#>
param(
    [Parameter(Mandatory = $true)][string]$JetsonHost,
    [Parameter(Mandatory = $true)][string]$User,
    [string]$RemoteDir = "~/NidarFiles"
)

$ErrorActionPreference = "Stop"
$root = Resolve-Path (Join-Path $PSScriptRoot "..\..")
$target = "$User@$JetsonHost"

Write-Host "Deploying $root -> ${target}:$RemoteDir"
ssh $target "mkdir -p $RemoteDir"
if ($LASTEXITCODE -ne 0) { throw "ssh to $target failed" }

# The Pixhawk-side code, the mission, the scripts. (custom-gcs stays on the laptop.)
foreach ($item in @("onboard-autonomy", "missions", "scripts", "README.md")) {
    $src = Join-Path $root $item
    Write-Host "  copying $item"
    scp -r -q $src "${target}:$RemoteDir/"
    if ($LASTEXITCODE -ne 0) { throw "scp of $item failed" }
}

# Files edited on Windows may carry CRLF line endings; bash scripts must not.
ssh $target "find $RemoteDir/scripts -name '*.sh' -exec sed -i 's/\r$//' {} + ; chmod +x $RemoteDir/scripts/jetson/*.sh $RemoteDir/missions/*/mission.py"
Write-Host "Done. On the Jetson: $RemoteDir/scripts/jetson/setup_jetson.sh"
