@echo off
setlocal
if not defined VCVARS set "VCVARS=D:\VC2022\VC\Auxiliary\Build\vcvars64.bat"
call "%VCVARS%" >nul || exit /b 1
set "SRC=%~dp0.."
if not exist "%SRC%\build\tests" mkdir "%SRC%\build\tests"
pushd "%SRC%\build\tests"
cl /nologo /TC /std:c11 /W4 /WX /MT /I"%SRC%\Core\Inc" "%SRC%\Core\Src\tracker.c" "%~dp0tracker_test.c" /Fetracker_test.exe
if errorlevel 1 (popd & exit /b 1)
tracker_test.exe
set RESULT=%ERRORLEVEL%
popd
exit /b %RESULT%
