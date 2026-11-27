@echo off
setlocal
call "%~dp0build.cmd" release || exit /b 1
powershell -NoProfile -ExecutionPolicy Bypass -File "%~dp0deploy.ps1" || exit /b 1
start "" "%~dp0build\run\DisplacementTracker.exe"
