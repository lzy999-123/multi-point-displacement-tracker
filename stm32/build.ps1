param([string]$UV4 = 'D:\Keil_v5\UV4\UV4.exe')
$ErrorActionPreference = 'Stop'
if (!(Test-Path -LiteralPath $UV4)) { throw "Keil not found: $UV4" }
$project = Join-Path $PSScriptRoot 'MDK-ARM\displacement_tracker.uvprojx'
$output = Join-Path $PSScriptRoot 'build'
New-Item -ItemType Directory -Path $output -Force | Out-Null
$log = Join-Path $output 'keil-build.log'
# Only builds. No download option is passed to Keil.
$process = Start-Process -FilePath $UV4 -ArgumentList @('-r', "`"$project`"", '-t', 'displacement_tracker', '-o', "`"$log`"") -WindowStyle Hidden -PassThru
while (!$process.WaitForExit(1000)) { }
if (!(Test-Path -LiteralPath $log)) { throw "Keil produced no build log (exit $($process.ExitCode))." }
$result = Get-Content -LiteralPath $log -Raw
Write-Output $result
$hex = Join-Path $output 'firmware\displacement_tracker.hex'
if ($process.ExitCode -gt 1 -or $result -notmatch '0 Error\(s\)' -or !(Test-Path -LiteralPath $hex)) {
    throw "Firmware build failed; see $log"
}
Get-FileHash -LiteralPath $hex -Algorithm SHA256
