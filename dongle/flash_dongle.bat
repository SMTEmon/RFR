@echo off
setlocal
set ESPTOOL="%USERPROFILE%\.platformio\packages\tool-esptool\esptool.exe"

if not exist %ESPTOOL% (
    set ESPTOOL=esptool.exe
)

set PORT_ARG=
if not "%~1"=="" (
    set PORT_ARG=-p %~1
)

echo ========================================================
echo   Flashing ESP32-C5 Super Mini Dongle All-In-One Binary
echo ========================================================
echo Firmware: %~dp0dongle_esp32c5_all_in_one.bin
echo Offset:   0x0 (Merged Bootloader + Partitions + App)
if not "%~1"=="" echo Port:     %~1
echo.

%ESPTOOL% --chip esp32c5 %PORT_ARG% write-flash -z 0x0 "%~dp0dongle_esp32c5_all_in_one.bin"

if %ERRORLEVEL% equ 0 (
    echo.
    echo ========================================================
    echo   FLASH SUCCESSFUL! Dongle is ready for the Dashboard.
    echo ========================================================
) else (
    echo.
    echo ========================================================
    echo   [ERROR] Flash failed.
    echo ========================================================
    echo   If your ESP32-C5 keeps connecting/disconnecting:
    echo   1. Unplug the ESP32-C5 from USB.
    echo   2. Hold down the 'BOOT' button on the board.
    echo   3. Plug it back into USB while holding 'BOOT'.
    echo   4. Release the 'BOOT' button.
    echo   5. Re-run this script (or pass COM port: flash_dongle.bat COMx).
    echo ========================================================
)
pause
