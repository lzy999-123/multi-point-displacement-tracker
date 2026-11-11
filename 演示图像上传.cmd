@echo off
setlocal
cd /d "%~dp0"
if not exist "%~dp0host\build\run\DisplacementTracker.exe" (
    call "%~dp0host\run.cmd"
    exit /b
)
start "" /D "%~dp0host\build\run" "%~dp0host\build\run\DisplacementTracker.exe"
