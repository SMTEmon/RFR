@echo off
setlocal
echo ========================================================
echo   Starting LFR Cockpit Local Web Server
echo ========================================================
echo.
echo URL: http://localhost:8000/dashboard/dashboard.html
echo.
echo NOTE: WebSerial requires a secure origin (http://localhost).
echo       Opening via localhost enables WebSerial in Chrome ^& Edge.
echo ========================================================
echo.

start "" http://localhost:8000/dashboard/dashboard.html
python -m http.server 8000
