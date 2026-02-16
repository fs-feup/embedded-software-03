@echo off
cd /d "%~dp0"
set PATH=C:\msys64\mingw64\bin;%PATH%
echo Compiling can_listener...
g++ -std=c++14 -O2 can_car_listener.cpp main.cpp -o can_listener.exe
if %errorlevel% neq 0 (
    echo BUILD FAILED
    exit /b 1
)
echo BUILD OK
