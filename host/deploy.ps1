param(
    [string[]]$Executables = @('DisplacementTracker'),
    [string]$QtRoot = 'D:\QT\6.10.1\msvc2022_64',
    [string]$OpenCvRoot = 'D:\opencv_build\build\install',
    [string]$OnnxRuntimeRoot = 'D:\onnxruntime_gpu\onnxruntime-win-x64-gpu-1.20.1'
)
$ErrorActionPreference = 'Stop'
$runtime = Join-Path $PSScriptRoot 'build\run'
New-Item -ItemType Directory -Path $runtime -Force | Out-Null
$targets = @()
foreach ($name in $Executables) {
    if ($name -notmatch '^[A-Za-z0-9_]+$') { throw "Invalid executable name: $name" }
    $source = Join-Path $PSScriptRoot "build\$name-release\release\$name.exe"
    if (!(Test-Path -LiteralPath $source)) { throw "Build first: $source" }
    $target = Join-Path $runtime "$name.exe"
    Copy-Item -LiteralPath $source -Destination $target -Force
    $targets += $target
}
if ($Executables -contains 'DisplacementTracker') {
    foreach ($creatorBuild in (Get-ChildItem -LiteralPath (Join-Path $PSScriptRoot 'build') -Directory -Filter 'Desktop_Qt_*')) {
        $creatorTarget = Join-Path $creatorBuild.FullName 'release\DisplacementTracker.exe'
        if (Test-Path -LiteralPath $creatorTarget) {
            try {
                Copy-Item -LiteralPath (Join-Path $runtime 'DisplacementTracker.exe') -Destination $creatorTarget -Force
                $targets += $creatorTarget
            } catch {
                Write-Warning "Skipped locked Qt Creator binary: $creatorTarget ($($_.Exception.Message))"
            }
        }
    }
}
$runtimeDirectories = @($targets | ForEach-Object { Split-Path -Parent $_ } | Select-Object -Unique)
foreach ($runtimeDirectory in $runtimeDirectories) {
foreach ($module in @('core', 'imgproc', 'imgcodecs', 'flann', 'features2d', 'calib3d', 'dnn', 'video', 'tracking')) {
    Copy-Item -LiteralPath (Join-Path $OpenCvRoot "x64\vc17\bin\opencv_${module}4120.dll") -Destination $runtimeDirectory -Force
}
foreach ($ortDll in @('onnxruntime.dll', 'onnxruntime_providers_shared.dll')) {
    $ortSrc = Join-Path $OnnxRuntimeRoot "lib\$ortDll"
    if (Test-Path -LiteralPath $ortSrc) {
        Copy-Item -LiteralPath $ortSrc -Destination $runtimeDirectory -Force
    }
}
$modelSrc = Join-Path $PSScriptRoot 'models\yolo26n.onnx'
if (Test-Path -LiteralPath $modelSrc) {
    $modelDir = Join-Path $runtimeDirectory 'models'
    New-Item -ItemType Directory -Path $modelDir -Force | Out-Null
    Copy-Item -LiteralPath $modelSrc -Destination (Join-Path $modelDir 'yolo26n.onnx') -Force
}
if (Test-Path -LiteralPath "$PSScriptRoot\camera-runtime.ini") {
    Copy-Item -LiteralPath "$PSScriptRoot\camera-runtime.ini" -Destination $runtimeDirectory -Force
}
}
foreach ($runtimeDirectory in $runtimeDirectories) {
    $directoryTargets = @($targets | Where-Object { (Split-Path -Parent $_) -eq $runtimeDirectory })
    & (Join-Path $QtRoot 'bin\windeployqt.exe') --dir $runtimeDirectory --no-translations --no-opengl-sw --no-compiler-runtime @directoryTargets
    if ($LASTEXITCODE -ne 0) { throw "Qt deployment failed: $LASTEXITCODE" }
}
# Qt's Windows ICU forwarding DLL also needs the implementation on this PC.
foreach ($name in @('icu.dll', 'icuin.dll', 'icuuc.dll')) {
    $source = Join-Path $env:SystemRoot "System32\$name"
    if (Test-Path -LiteralPath $source) {
        foreach ($runtimeDirectory in $runtimeDirectories) {
            Copy-Item -LiteralPath $source -Destination $runtimeDirectory -Force
        }
    }
}
if ($Executables -contains 'core_test') {
    Copy-Item -LiteralPath (Join-Path $QtRoot 'plugins\platforms\qoffscreen.dll') -Destination (Join-Path $runtime 'platforms') -Force
}
Write-Output "Runtime ready: $runtime"
