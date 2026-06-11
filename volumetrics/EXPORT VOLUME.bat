@echo off
setlocal
REM Drop a .blend on this BAT — exports every VOLUME object in the scene
REM as a density grid (.npy + meta.json) for the web viewer ray-marcher.
REM The .blend is never modified or saved.
if "%~1"=="" (
    echo Drop a .blend file onto this BAT.
    pause & exit /b 1
)
"C:\Program Files\Blender Foundation\Blender 5.0\blender.exe" --background "%~1" --python "%~dp0export_volume.py" -- --out "%~dp0"
pause
