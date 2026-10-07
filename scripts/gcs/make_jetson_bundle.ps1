<#
Copies the Jetson-side parts of NidarFiles to a folder (e.g. a USB stick),
for a Jetson with no network to the laptop -- carry it over and work on
the Jetson with a monitor and keyboard.

Usage:
    .\scripts\gcs\make_jetson_bundle.ps1 -Destination E:\
    -> E:\NidarFiles\{onboard-autonomy, missions, scripts, README.md}

On the Jetson (USB sticks mount under /media/<user>/<label>):
    rm -rf ~/NidarFiles && cp -r /media/$USER/<label>/NidarFiles ~/
    chmod +x ~/NidarFiles/scripts/jetson/*.sh ~/NidarFiles/missions/hover/mission.py
    ~/NidarFiles/scripts/jetson/setup_jetson.sh     # rebuilds ~/nidar_ws

Leaves out .git, caches and logs. Shell and Python files are written with
LF line endings (bash on the Jetson rejects CRLF).
#>
param(
    [Parameter(Mandatory = $true)][string]$Destination
)

$ErrorActionPreference = "Stop"
$root = Resolve-Path (Join-Path $PSScriptRoot "..\..")
if (-not (Test-Path $Destination)) { throw "destination $Destination does not exist (USB stick plugged in?)" }
$out = Join-Path $Destination "NidarFiles"
if (Test-Path $out) {
    Write-Host "Replacing existing $out"
    Remove-Item -Recurse -Force $out
}
New-Item -ItemType Directory -Force $out | Out-Null

$skip = @(".git", "__pycache__", ".pytest_cache", "build", "install", "log", "logs")
$count = 0
foreach ($item in @("onboard-autonomy", "missions", "scripts", "README.md")) {
    $src = Join-Path $root $item
    $files = if (Test-Path $src -PathType Leaf) { @(Get-Item $src) } else { Get-ChildItem $src -Recurse -File -Force }
    foreach ($file in $files) {
        $rel = $file.FullName.Substring($root.Path.Length).TrimStart("\")
        if ($rel.Split("\") | Where-Object { $skip -contains $_ }) { continue }
        $dest = Join-Path $out $rel
        New-Item -ItemType Directory -Force (Split-Path $dest) | Out-Null
        if ($file.Extension -in @(".sh", ".py")) {
            $text = [System.IO.File]::ReadAllText($file.FullName) -replace "`r`n", "`n"
            [System.IO.File]::WriteAllText($dest, $text, (New-Object System.Text.UTF8Encoding $false))
        } else {
            Copy-Item $file.FullName $dest
        }
        $count++
    }
}
Write-Host "Copied $count files to $out"
Write-Host "On the Jetson: cp -r /media/`$USER/<label>/NidarFiles ~/ ; then run ~/NidarFiles/scripts/jetson/setup_jetson.sh"
