@echo off
setlocal
cd /d "%~dp0"
set GXX=g++
where g++ >nul 2>nul || set GXX=C:\MinGW\bin\g++.exe
"%GXX%" -std=c++11 -O2 -Wall -Wextra -static -DMAZE_MAX_SIZE=32 -I..\main ^
  mms_main.cpp mouse.cpp API.cpp ..\main\maze.cpp -o mouse.exe
if errorlevel 1 (
  echo BUILD FAILED
  exit /b 1
)
echo built %~dp0mouse.exe
