@echo off
rem Command-line build. Qt Creator can open DisplacementTracker.pro directly instead.
rem Usage: build.cmd [release^|debug] [project.pro, default DisplacementTracker.pro]
setlocal
set BUILD_CONFIG=%1
if "%BUILD_CONFIG%"=="" set BUILD_CONFIG=release
set PRO=%2
if "%PRO%"=="" set PRO=DisplacementTracker.pro

if not defined QTDIR set QTDIR=D:\QT\6.10.1\msvc2022_64
if not defined VCVARS set VCVARS=D:\VC2022\VC\Auxiliary\Build\vcvars64.bat
set JOM=D:\QT\Tools\QtCreator\bin\jom\jom.exe

call "%VCVARS%" >nul || exit /b 1
set PATH=%QTDIR%\bin;%PATH%

set SRC=%~dp0
for %%F in (%PRO%) do set NAME=%%~nF
set OUT=%SRC%build\%NAME%-%BUILD_CONFIG%
if not exist "%OUT%" mkdir "%OUT%"
pushd "%OUT%"
qmake "%SRC%%PRO%" CONFIG+=%BUILD_CONFIG% || (popd & exit /b 1)
"%JOM%" || (popd & exit /b 1)
popd
echo Build OK: %OUT%
