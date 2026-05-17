@echo off
if "%1"=="" (
    echo Usage: run_bridge.bat COM3
    echo.
    echo Find your COM port in Device Manager under "Ports (COM & LPT)"
    exit /b 1
)
python foxglove_can_bridge.py %1
