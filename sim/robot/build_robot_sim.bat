@echo off
rem ===========================================================================
rem  Builds robot_sim.exe: the COMPLETE firmware (..\..\main, unchanged) plus
rem  the MPU6050_light library, on a simulated robot (hal.cpp / world.h).
rem  Usage afterwards:  robot_sim.exe ..\mazes\mmrc26-island-10x10.txt [--mirror] [--seed N] [--quiet]
rem ===========================================================================
setlocal
cd /d "%~dp0"
set GXX=g++
where g++ >nul 2>nul || set GXX=C:\MinGW\bin\g++.exe
set MPU=%USERPROFILE%\Documents\Arduino\libraries\MPU6050_light\src
set FW=..\..\main
"%GXX%" -std=gnu++14 -O2 -Wall -Wextra -static -DARDUINO=10800 -Ishim -I%FW% -I%MPU% -I.. ^
  robot_sim.cpp hal.cpp ..\truemaze.cpp "%MPU%\MPU6050_light.cpp" ^
  %FW%\battery.cpp %FW%\encoders.cpp %FW%\estimator.cpp %FW%\imu.cpp %FW%\maze.cpp %FW%\modes.cpp %FW%\motion.cpp ^
  %FW%\motors.cpp %FW%\sched.cpp %FW%\sonar.cpp %FW%\strategy.cpp %FW%\ui.cpp ^
  -x c++ %FW%\main.ino -x none -o robot_sim.exe
if errorlevel 1 (
  echo BUILD FAILED
  exit /b 1
)
echo built %~dp0robot_sim.exe
