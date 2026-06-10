@echo off
setlocal

set BLENDER="C:\Program Files\Blender Foundation\Blender 5.0\blender.exe"
set SCRIPT=%~dp0gs_capture.py
set BLEND=%~1
set SCENE_NAME=%~n1
set OUTPUT=%~dp0output\%SCENE_NAME%

echo.
echo ━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━
echo  GS Capture Launcher
echo  Scene : %SCENE_NAME%
echo  Output: %OUTPUT%
echo ━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━
echo.

mkdir "%OUTPUT%\images" 2>nul

%BLENDER% --background "%BLEND%" --python "%SCRIPT%" -- "%OUTPUT%" "%SCENE_NAME%"

echo.
echo ━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━
echo  Done. Output folder:
echo  %OUTPUT%
echo ━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━
pause
