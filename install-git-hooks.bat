@echo off
echo Installing MARSLib Git pre-commit hooks...

call gradlew installGitHooks

if %ERRORLEVEL% equ 0 (
    echo Hook installed successfully! Your code will automatically format and test when committing.
) else (
    echo Failed to install Git Hooks natively.
)
pause
