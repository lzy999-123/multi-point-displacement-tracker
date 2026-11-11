@echo off
setlocal
rem Native app owns camera USB and WLAN for its whole lifetime.
call "%~dp0run.cmd"
