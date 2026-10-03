@echo off
rem ===========================================================================
rem  Batch verification of the maze brain (no GUI needed):
rem   1. builds test_runner.exe (brain + fake mms API),
rem   2. generates 20 MMRC-style 10x10 island mazes into mazes\generated,
rem   3. runs every maze (mazefiles classic + halfsize + training + generated),
rem      normal and mirrored, and prints a summary.
rem ===========================================================================
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
