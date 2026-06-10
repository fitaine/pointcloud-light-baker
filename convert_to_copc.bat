@echo off
REM ═══════════════════════════════════════════════════════════════════
REM  convert_to_copc.bat — Step 3 of the LiDAR lighting pipeline
REM
REM  Converts a baked lit PLY to COPC format for the Potree viewer.
REM  Uses PDAL (bundled with QGIS) — no extra installs needed.
REM
REM  Usage:
REM    convert_to_copc.bat "path\to\scene-lit.ply" scene-id
REM
REM  Examples:
REM    convert_to_copc.bat "prototype\pointclouds\chamechaude-lit.ply" chamechaude-lit
REM    convert_to_copc.bat "prototype\pointclouds\grande-motte-lit.ply" grande-motte-lit
REM
REM  Output: potree\pointclouds\<scene-id>.copc.laz
REM  View:   http://localhost:8081/?scene=<scene-id>
REM ═══════════════════════════════════════════════════════════════════

set PDAL=C:\Program Files\QGIS 3.40.5\bin\pdal.exe

if not exist "%PDAL%" (
    echo.
    echo ERROR: PDAL not found at %PDAL%
    echo Check QGIS installation path.
    pause
    exit /b 1
)

if "%~1"=="" goto usage
if "%~2"=="" goto usage

set INPUT=%~1
set NAME=%~2
set OUTPUT=%~dp0potree\pointclouds\%NAME%.copc.laz
set TMPJSON=%TEMP%\lidar_to_copc.json

echo.
echo ──────────────────────────────────────────────────────────────────
echo  Input  : %INPUT%
echo  Scene  : %NAME%
echo  Output : %OUTPUT%
echo ──────────────────────────────────────────────────────────────────
echo.

REM Write PDAL pipeline JSON
(
echo {
echo   "pipeline": [
echo     {
echo       "type": "readers.ply",
echo       "filename": "%INPUT:\=\\%"
echo     },
echo     {
echo       "type": "writers.copc",
echo       "filename": "%OUTPUT:\=\\%"
echo     }
echo   ]
echo }
) > "%TMPJSON%"

"%PDAL%" pipeline "%TMPJSON%"

if %ERRORLEVEL%==0 (
    echo.
    echo ──────────────────────────────────────────────────────────────────
    echo  Done!
    echo  Start viewer:  double-click potree\START VIEWER.bat
    echo  URL:           http://localhost:8081/?scene=%NAME%
    echo ──────────────────────────────────────────────────────────────────
) else (
    echo.
    echo ERROR: PDAL conversion failed (exit %ERRORLEVEL%)
)
echo.
del "%TMPJSON%" 2>nul
pause
goto :eof

:usage
echo.
echo Usage: convert_to_copc.bat "input-lit.ply" scene-id
echo.
pause
