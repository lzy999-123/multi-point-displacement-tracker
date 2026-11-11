param(
    [switch]$Flash,
    [string]$Port = 'COM5',
    [ValidateRange(0, 3)]
    [int]$DeviceId = 0,
    [ValidateSet('ESP32Cam', 'S3')]
    [string]$Board = 'ESP32Cam',
    [ValidateSet('3.3.12', '2.0.17')]
    [string]$Framework = '3.3.12',
    [string]$ToolRoot = 'D:\esp32',
    [string]$BuildDirectory = ''
)
$ErrorActionPreference = 'Stop'

# 0 keeps the source ID; explicit IDs are applied only to the staged copy.
function Set-StagedDeviceId([string]$StagePath) {
    if ($DeviceId -eq 0) { return }
    $configPath = Join-Path $StagePath 'config.h'
    $configText = [IO.File]::ReadAllText($configPath)
    $pattern = '(?m)^#define\s+DEVICE_ID\s+\d+\s*$'
    if ([regex]::Matches($configText, $pattern).Count -ne 1) { throw 'Expected one DEVICE_ID definition.' }
    [IO.File]::WriteAllText($configPath, [regex]::Replace($configText, $pattern, "#define DEVICE_ID $DeviceId"))
}

if (!$BuildDirectory) {
    $folder = if ($Board -eq 'ESP32Cam') { "displacement-cam-ov3660-$Framework" } else { 'displacement-cam-2.0.17' }
    $BuildDirectory = Join-Path $ToolRoot "build-out\$folder"
}
if ($Board -eq 'ESP32Cam') {
    $cli = Join-Path $ToolRoot 'arduino-cli\arduino-cli.exe'
    $config = Join-Path $ToolRoot 'arduino-cli\arduino-cli.yaml'
    $core = Join-Path $ToolRoot 'arduino15\packages\esp32\hardware\esp32\3.3.12'
    $compileProperties = @()
    $fqbn = 'esp32:esp32:esp32cam:FlashMode=dio,FlashFreq=40'
    if ($Framework -eq '2.0.17') {
        $config = Join-Path $ToolRoot 'compat-2.0.17\arduino-cli.yaml'
        $core = Join-Path $ToolRoot 'compat-2.0.17\core-extract\esp32-2.0.17'
        $compiler = Join-Path $ToolRoot 'compat-2.0.17\compiler\xtensa-esp32-elf'
        if (!(Test-Path -LiteralPath "$compiler\bin\xtensa-esp32-elf-g++.exe")) { throw '缺少隔离的ESP32 2.0.17官方编译器。' }
        $esptool = Join-Path $ToolRoot 'arduino15\packages\esp32\tools\esptool_py\5.3.1'
        $compileProperties = @('--build-property', "runtime.tools.xtensa-esp32-elf-gcc.path=$compiler",
                               '--build-property', "runtime.tools.esptool_py.path=$esptool")
        $fqbn = 'espressif:esp32:esp32cam:FlashMode=dio,FlashFreq=40'
    }
    $python = Join-Path $ToolRoot 'venv\Scripts\python.exe'
    foreach ($required in @($cli, $config, "$core\package.json", $python)) {
        if (!(Test-Path -LiteralPath $required)) { throw "缺少工具链文件：$required" }
    }
    if ((Get-Content -LiteralPath "$core\package.json" -Raw | ConvertFrom-Json).version -ne $Framework) {
        throw "ESP32Cam 构建要求 Arduino-ESP32 $Framework。"
    }
    $BuildDirectory = [IO.Path]::GetFullPath($BuildDirectory)
    if ($BuildDirectory -match '[^\x00-\x7F]') { throw '构建目录要求为ASCII路径。' }
    $stage = Join-Path $BuildDirectory 'sketch\displacement_cam'
    $output = Join-Path $BuildDirectory 'bin'
    New-Item -ItemType Directory -Path $stage, $output -Force | Out-Null
    foreach ($name in @('displacement_cam.ino', 'config.h', 'camera_pins.h', 'secrets.h')) {
        Copy-Item -LiteralPath "$PSScriptRoot\displacement_cam\$name" -Destination $stage -Force
    }
    Set-StagedDeviceId $stage
    & $cli --config-file $config compile --jobs 4 -b $fqbn @compileProperties --build-path $output $stage
    if ($LASTEXITCODE -ne 0) { throw "编译失败：$LASTEXITCODE" }
    Copy-Item -LiteralPath "$core\tools\partitions\boot_app0.bin" -Destination $output -Force
    $app = Join-Path $output 'displacement_cam.ino.bin'
    $manifest = [ordered]@{
        framework = "Arduino-ESP32 $Framework"
        fqbn = $fqbn
        built_at = (Get-Date -Format o)
        app_sha256 = (Get-FileHash -LiteralPath $app -Algorithm SHA256).Hash
        source_sha256 = @{}
    }
    foreach ($name in @('displacement_cam.ino', 'config.h', 'camera_pins.h')) {
        $manifest.source_sha256[$name] = (Get-FileHash -LiteralPath "$stage\$name" -Algorithm SHA256).Hash
    }
    $manifest | ConvertTo-Json -Depth 4 | Set-Content -LiteralPath "$BuildDirectory\build-info.json" -Encoding utf8
    Write-Host "构建完成：$app"
    if ($Flash) {
        # --chip 防止把普通ESP32固件误写到旧S3板；不执行整片擦除。
        & $python -m esptool --chip esp32 --port $Port --baud 115200 write-flash `
            --flash-mode dio --flash-freq 40m --flash-size 4MB `
            0x1000 "$output\displacement_cam.ino.bootloader.bin" `
            0x8000 "$output\displacement_cam.ino.partitions.bin" `
            0xe000 "$output\boot_app0.bin" 0x10000 $app
        if ($LASTEXITCODE -ne 0) { throw "烧录失败：$LASTEXITCODE" }
    }
    return
}

# 隔离的官方2.0.17工具链；不替换本机另装的3.x。
$compat = Join-Path $ToolRoot 'compat-2.0.17'
$cli = Join-Path $ToolRoot 'arduino-cli\arduino-cli.exe'
$config = Join-Path $compat 'arduino-cli.yaml'
$core = Join-Path $compat 'core-extract\esp32-2.0.17'
$compiler = Join-Path $compat 'compiler\xtensa-esp32s3-elf'
$esptool = Join-Path $ToolRoot 'arduino15\packages\esp32\tools\esptool_py\5.3.1'
$python = Join-Path $ToolRoot 'venv\Scripts\python.exe'
$jpegRoot = Join-Path $ToolRoot 'arduino15\packages\esp32\tools\esp32s3-libs\3.3.12'
$jpegHeaders = Join-Path $jpegRoot 'include\espressif__esp_new_jpeg\include'
$jpegLibrary = Join-Path $jpegRoot 'lib\libesp_new_jpeg.a'
foreach ($required in @($cli, $config, "$core\package.json", "$compiler\bin\xtensa-esp32s3-elf-g++.exe", "$esptool\esptool.exe", "$jpegHeaders\esp_jpeg_enc.h", $jpegLibrary)) {
    if (!(Test-Path -LiteralPath $required)) { throw "缺少隔离工具链文件：$required。参见 esp32cam/README.md。" }
}
if ((Get-Content -LiteralPath "$core\package.json" -Raw | ConvertFrom-Json).version -ne '2.0.17') {
    throw '当前脚本只验证过 Arduino-ESP32 2.0.17。'
}
$BuildDirectory = [IO.Path]::GetFullPath($BuildDirectory)
if ($BuildDirectory -match '[^\x00-\x7F]') { throw '此工具链要求构建目录为ASCII路径。' }
$stage = Join-Path $BuildDirectory 'sketch\displacement_cam'
$output = Join-Path $BuildDirectory 'bin'
New-Item -ItemType Directory -Path $stage, $output -Force | Out-Null
# secrets.h 仅复制到本地构建目录，不打印内容、不写入构建清单。
Copy-Item -Path "$PSScriptRoot\displacement_cam\*" -Destination $stage -Force
Set-StagedDeviceId $stage
# 仅复用本机官方S3优化软件JPEG库，WiFi和相机驱动仍用2.0.17。
Copy-Item -Path "$jpegHeaders\*.h" -Destination $stage -Force
& $cli --config-file $config compile --jobs 4 `
    -b 'espressif:esp32:esp32s3:PSRAM=opi,FlashSize=8M' `
    --build-property "runtime.tools.xtensa-esp32s3-elf-gcc.path=$compiler" `
    --build-property "runtime.tools.esptool_py.path=$esptool" `
    --build-property "compiler.libraries.ldflags=$jpegLibrary" `
    --build-path $output $stage
if ($LASTEXITCODE -ne 0) { throw "编译失败：$LASTEXITCODE" }
Copy-Item -LiteralPath "$core\tools\partitions\boot_app0.bin" -Destination $output -Force
$app = Join-Path $output 'displacement_cam.ino.bin'
$manifest = [ordered]@{
    framework = 'Arduino-ESP32 2.0.17'
    fqbn = 'espressif:esp32:esp32s3:PSRAM=opi,FlashSize=8M'
    built_at = (Get-Date -Format o)
    app_sha256 = (Get-FileHash -LiteralPath $app -Algorithm SHA256).Hash
    jpeg_library_sha256 = (Get-FileHash -LiteralPath $jpegLibrary -Algorithm SHA256).Hash
    source_sha256 = @{}
}
foreach ($name in @('displacement_cam.ino', 'config.h', 'camera_pins.h')) {
    $manifest.source_sha256[$name] = (Get-FileHash -LiteralPath "$stage\$name" -Algorithm SHA256).Hash
}
$manifest | ConvertTo-Json -Depth 4 | Set-Content -LiteralPath "$BuildDirectory\build-info.json" -Encoding utf8
Write-Host "构建完成：$app"
if ($Flash) {
    if (!(Test-Path -LiteralPath $python)) { throw "缺少烧录用Python：$python" }
    & $python -m esptool --port $Port --baud 230400 write-flash `
        --flash-mode dio --flash-freq 80m --flash-size 8MB `
        0x0 "$output\displacement_cam.ino.bootloader.bin" `
        0x8000 "$output\displacement_cam.ino.partitions.bin" `
        0xe000 "$output\boot_app0.bin" 0x10000 $app
    if ($LASTEXITCODE -ne 0) { throw "烧录失败：$LASTEXITCODE" }
}
