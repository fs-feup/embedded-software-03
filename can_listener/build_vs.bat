@echo off
REM Build script for can_listener
REM Before building, edit can_car_listener.hpp line 29:
REM   S4 = 125k for HANDCART
REM   S7 = 800k for ACCUMULATOR

call "C:\Program Files\Microsoft Visual Studio\2022\Community\VC\Auxiliary\Build\vcvars64.bat"
cd /d "%~dp0"

set OUTPUT_NAME=can_listener_handcart.exe
cl /EHsc /O2 /Fe:%OUTPUT_NAME% main.cpp can_car_listener.cpp

if exist %OUTPUT_NAME% (
    echo.
    echo SUCCESS: %OUTPUT_NAME% created!
    echo Run it with: %OUTPUT_NAME% COM7
) else (
    echo.
    echo FAILED to compile
)
pause
