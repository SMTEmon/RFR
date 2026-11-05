@echo off
setlocal
set ESPTOOL="%USERPROFILE%\.platformio\packages\tool-esptool\esptool.exe"

if not exist %ESPTOOL% (
    set ESPTOOL=esptool.exe
)

set PORT=COM11
if not "%~1"=="" set PORT=%~1

echo ========================================================
echo   Flashing LFR Robot Firmware to %PORT%
echo ========================================================
echo Robot Peer MAC: 38:3E:51:58:16:70 (New Dongle on COM9)
echo.
echo NOTE: If connection shows "Wrong boot mode detected (0x13)",
echo       hold down the physical 'BOOT' button on the ESP32 board
echo       while this script connects, then release it once flashing starts.
echo ========================================================
echo.

%ESPTOOL% --chip esp32 -p %PORT% -b 460800 --before default-reset --after hard-reset write-flash -z --flash-mode dio --flash-freq 40m --flash-size detect 0x1000 "%~dp0.pio\build\esp32dev\bootloader.bin" 0x8000 "%~dp0.pio\build\esp32dev\partitions.bin" 0xe000 "%USERPROFILE%\.platformio\packages\framework-arduinoespressif32\tools\partitions\boot_app0.bin" 0x10000 "%~dp0.pio\build\esp32dev\firmware.bin"

if %ERRORLEVEL% equ 0 (
    echo.
    echo ========================================================
    echo   FLASH SUCCESSFUL! Robot is paired with the Dongle.
    echo ========================================================
) else (
    echo.
    echo ========================================================
    echo   [FLASH FAILED]
    echo   Please hold the BOOT button on the robot and re-run.
    echo ========================================================
)
pause
