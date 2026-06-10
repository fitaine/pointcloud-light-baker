@echo off
REM ═══════════════════════════════════════════════════════════════════
REM  convert_to_potree.bat — Step 3 of the LiDAR lighting pipeline
REM
REM  Usage:
REM    convert_to_potree.bat "path\to\scene-lit.ply" scene-id
REM
REM  Examples:
REM    convert_to_potree.bat "prototype\pointclouds\chamechaude-lit.ply" chamechaude
REM    convert_to_potree.bat "prototype\pointclouds\grande-motte-lit.ply" grande-motte
REM
REM  Output: potree\pointclouds\<scene-id>\metadata.json + octree.bin
REM  View:   http://localhost:8081/?scene=<scene-id>
REM ═══════════════════════════════════════════════════════════════════

set CONVERTER=C:\Tools\PotreeConverter\PotreeConverter_windows_x64\PotreeConverter.exe

if not exist "%CONVERTER%" (
    echo.
    echo ERROR: PotreeConverter not found at %CONVERTER%
    echo Download from:
    echo   https://github.com/potree/PotreeConverter/releases/download/2.1.1/PotreeConverter_2.1.1_x64_windows.zip
    echo Extract PotreeConverter.exe to C:\Tools\PotreeConverter\
    echo.
    pause
    exit /b 1
)

if "%~1"=="" (
    echo.
    echo Usage: convert_to_potree.bat "input-lit.ply" scene-id
    pause
    exit /b 1
)

if "%~2"=="" (
    echo.
    echo Usage: convert_to_potree.bat "input-lit.ply" scene-id
    pause
    exit /b 1
)

set INPUT=%~1
set NAME=%~2
set OUTPUT=%~dp0potree\pointclouds\%NAME%

echo.
echo ──────────────────────────────────────────────────────────────────
echo  Input  : %INPUT%
echo  Scene  : %NAME%
echo  Output : %OUTPUT%
echo ──────────────────────────────────────────────────────────────────
echo.

%CONVERTER% "%INPUT%" -o "%OUTPUT%"

if %ERRORLEVEL%==0 (
    echo.
    echo ──────────────────────────────────────────────────────────────────
    echo  Done!
    echo  Start viewer:  double-click potree\START VIEWER.bat
    echo  URL:           http://localhost:8081/?scene=%NAME%
    echo ──────────────────────────────────────────────────────────────────
) else (
    echo.
    echo ERROR: PotreeConverter returned code %ERRORLEVEL%
)
echo.
pause
