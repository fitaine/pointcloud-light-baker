@echo off
cd /d "%~dp0"
echo.
echo ──────────────────────────────────────────────────────────────────
echo  LiDAR Point Cloud Viewer — Potree
echo  Requires range-capable server (COPC uses HTTP Range requests)
echo.
echo  http://localhost:8081/
echo  http://localhost:8081/?scene=chamechaude-lit
echo  http://localhost:8081/?scene=grande-motte-lit
echo  http://localhost:8081/?test=1   (Potree sample, for testing)
echo ──────────────────────────────────────────────────────────────────
echo.
start "" "http://localhost:8081/"
python server.py 8081
pause
