@echo off
chcp 65001 >nul
setlocal
cd /d "%~dp0.."
set "PYTHON_EXE=D:\miniconda3\python.exe"
if not exist "%PYTHON_EXE%" set "PYTHON_EXE=python"
"%PYTHON_EXE%" tools\run_usb_camera.py --port COM5
if errorlevel 1 pause
