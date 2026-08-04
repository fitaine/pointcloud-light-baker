@echo off
title Potree Viewer Server
REM Must run potree\server.py, NOT "python -m http.server": COPC clouds are read
REM with HTTP Range requests, which the built-in server ignores — the viewer then
REM hangs on an endless load. server.py serves the 2D gallery at / and Potree
REM under /3d/, so the viewer URL carries the /3d/ prefix.
cd /d "%~dp0potree"
echo.
echo  ------------------------------------------------------------------
echo   LiDAR Point Cloud Viewer - Potree  (range-capable server)
echo.
echo   2D gallery   http://localhost:8081/
echo   3D viewer    http://localhost:8081/3d/
echo   a scene      http://localhost:8081/3d/?scene=SCENE-ID
echo   Potree demo  http://localhost:8081/3d/?test=1
echo  ------------------------------------------------------------------
echo.
start "" "http://localhost:8081/3d/"
python server.py 8081
pause
