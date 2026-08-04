@echo off
title Potree Viewer Server
cd /d "%~dp0"
python -m http.server 8081
pause
