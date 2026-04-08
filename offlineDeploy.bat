@echo off
echo Starting MARSLib Offline Deploy (Competition Pits Mode)...
echo This will use ONLY the dependencies currently cached on your computer.

call gradlew deploy --offline --no-daemon

if %ERRORLEVEL% equ 0 (
    echo.
    echo ---
    echo OFFLINE DEPLOY SUCCESSFUL!
) else (
    echo.
    echo ---
    echo DEPLOY FAILED. Check if you are connected to the robot radio.
)
pause
