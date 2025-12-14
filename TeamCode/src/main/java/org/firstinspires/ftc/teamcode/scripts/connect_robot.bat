@echo off
setlocal

rem === CONFIGURATION ===
set "ROBOT_SSID=11409-RC"
set "ROBOT_PROFILE=11409-RC"
set "WIFI_INTERFACE=Wi-Fi"
set "ADB_IP=192.168.43.1"
set "ADB_PORT=5555"

echo Connecting to Wi-Fi network "%ROBOT_SSID%"...
netsh wlan connect name="%ROBOT_PROFILE%" interface="%WIFI_INTERFACE%"
echo (netsh exit code: %errorlevel%)
echo.

echo Waiting ~7 seconds for Wi-Fi to settle...
rem timeout doesn't work when stdin is redirected (like from Android Studio),
rem so we use ping as a portable delay.
ping -n 8 127.0.0.1 >nul

echo.
echo Current Wi-Fi status:
set "TMPFILE=%temp%\wlan_status_%RANDOM%.txt"
netsh wlan show interfaces > "%TMPFILE%"
findstr /R /C:"^\ *State" /C:"^\ *SSID" "%TMPFILE%"
del "%TMPFILE%" >nul 2>&1

echo.
echo Trying to connect ADB to %ADB_IP%:%ADB_PORT% ...
adb connect %ADB_IP%:%ADB_PORT%

echo.
echo Current ADB devices:
adb devices

echo.
echo ✔ Finished connect_robot.bat
echo (If ADB doesn't show the RC, make sure the RC is on and ADB over network is allowed.)
echo.
pause
endlocal
