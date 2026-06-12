@echo off
setlocal
REM ━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━
REM  TEST preset — drop a .blend file on this BAT.
REM  Same pipeline as RUN PIPELINE.bat with every quality lever
REM  floored: 26 orbit frames at 720/16spp, 4 m/px ortho,
REM  point cloud decimated 1/10.
REM  All outputs are suffixed "-test" — never touches production.
REM ━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━

if "%~1"=="" (
    echo Drop a .blend file onto this BAT to run the TEST pipeline.
    pause
    exit /b 1
)

set PYTHON=C:\Users\Tiphaine\miniforge3\python.exe

"%PYTHON%" "%~dp0run_pipeline.py" %1 --test

echo.
pause
