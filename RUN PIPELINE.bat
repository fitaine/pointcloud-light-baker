@echo off
setlocal
REM ━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━
REM  LiDAR lighting pipeline — drop a .blend file on this BAT.
REM  .blend → orbit renders → reproject onto raw IGN tiles → COPC
REM  → registered in the Potree viewer.
REM
REM  Resumable: close this window at any time; drop the same .blend
REM  again and it continues where it stopped.
REM ━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━

if "%~1"=="" (
    echo Drop a .blend file onto this BAT to run the pipeline.
    pause
    exit /b 1
)

set PYTHON=C:\Users\Tiphaine\miniforge3\python.exe

"%PYTHON%" "%~dp0run_pipeline.py" %1

echo.
pause
