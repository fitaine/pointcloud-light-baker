@echo off
cd /d "%~dp0prototype"
echo.
echo  Point Cloud Viewer
echo  http://localhost:8001
echo.
start http://localhost:8001
python -m http.server 8001
pause
