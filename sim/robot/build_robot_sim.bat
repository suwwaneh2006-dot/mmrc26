@echo off
setlocal
cd /d "%~dp0"
set GXX=g++
where g++ >nul 2>nul || set GXX=C:\MinGW\bin\g++.exe
set MPU=%USERPROFILE%\Documents\Arduino\libraries\MPU6050_light\src
set FW=..\..\main
"%GXX%" -std=gnu++14 -O2 -Wall -Wextra -static -DARDUINO=10800 -Ishim -I%FW% -I%MPU% -I.. ^
  robot_sim.cpp hal.cpp ..\truemaze.cpp "%MPU%\MPU6050_light.cpp" ^
  %FW%\battery.cpp %FW%\calib.cpp %FW%\encoders.cpp %FW%\estimator.cpp %FW%\imu.cpp %FW%\maze.cpp %FW%\modes.cpp %FW%\motion.cpp ^
  %FW%\motors.cpp %FW%\sched.cpp %FW%\sonar.cpp %FW%\strategy.cpp %FW%\ui.cpp ^
  -x c++ %FW%\main.ino -x none -o robot_sim.exe
if errorlevel 1 (
  echo BUILD FAILED
  exit /b 1
)
echo built %~dp0robot_sim.exe
