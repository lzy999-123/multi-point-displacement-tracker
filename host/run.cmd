@echo off
setlocal
if not exist "%~dp0build\DisplacementTracker-release\release\DisplacementTracker.exe" (
    call "%~dp0build.cmd" release || exit /b 1
)
powershell -NoProfile -ExecutionPolicy Bypass -File "%~dp0deploy.ps1" || exit /b 1
start "" "%~dp0build\run\DisplacementTracker.exe"
