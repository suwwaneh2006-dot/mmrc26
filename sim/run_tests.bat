@echo off
setlocal
cd /d "%~dp0"
set GXX=g++
where g++ >nul 2>nul || set GXX=C:\MinGW\bin\g++.exe
"%GXX%" -std=c++11 -O2 -Wall -Wextra -static -DMAZE_MAX_SIZE=32 -I..\main ^
  test_runner.cpp mouse.cpp fake_api.cpp truemaze.cpp ..\main\maze.cpp -o test_runner.exe
if errorlevel 1 (
  echo BUILD FAILED
  exit /b 1
)
if not exist mazes\generated mkdir mazes\generated
.\test_runner.exe --gen 20 mazes\generated || exit /b 1
dir /b /s mazes\generated\*.txt mazes\mazefiles\classic\*.txt mazes\mazefiles\halfsize\*.txt mazes\mazefiles\training\*.txt > mazes\all.lst
.\test_runner.exe --quiet @mazes\all.lst
