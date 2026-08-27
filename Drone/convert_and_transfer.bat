@echo off
setlocal

rem Configuration
set "ROBOT=pi@144.32.70.209"
set "ROBOT_PASSWORD=raspberry"
set "ROBOT_HOST_KEY=SHA256:dpIN+QZ3c/CBbfBhEwggVKwrvOZrhTx8pxF7ousXzTw"
set "START_X=-1"
set "START_Y=-6"
set "INPUT_FILE=%~dp0targetLocation.json"
set "OUTPUT_FILE=%~dp0targetLocation_robot.json"
set "REMOTE_FILE=/home/pi/summer_robotics_project/ROBOT01/ROS/src/targetLocation.json"

rem The first command-line argument optionally overrides ROBOT.
if not "%~1"=="" set "ROBOT=%~1"

python "%~dp0convert_to_robot_frame.py" ^
  --input "%INPUT_FILE%" ^
  --output "%OUTPUT_FILE%" ^
  --start-x "%START_X%" ^
  --start-y "%START_Y%"
if errorlevel 1 exit /b %errorlevel%

where pscp.exe >nul 2>&1
if errorlevel 1 (
  echo pscp.exe from PuTTY is required for password-based SCP. 1>&2
  exit /b 1
)

pscp.exe -batch -hostkey "%ROBOT_HOST_KEY%" -pw "%ROBOT_PASSWORD%" ^
  "%OUTPUT_FILE%" ^
  "%ROBOT%:%REMOTE_FILE%"
if errorlevel 1 exit /b %errorlevel%

endlocal
