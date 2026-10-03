@echo off
rem ===========================================================================
rem  Builds mouse.exe for the mms simulator from the UNCHANGED firmware brain
rem  (..\main\maze.cpp). Use this file as the mms "Build Command".
rem  Needs MinGW g++ (C:\MinGW\bin). -static: mouse.exe needs no MinGW DLLs.
rem ===========================================================================
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
