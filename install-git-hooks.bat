@echo off
echo Installing MARSLib Git pre-commit hooks...

if not exist ".git\hooks" (
    echo Error: Could not find .git\hooks directory. Make sure you run this script from the root of the repository!
    exit /b 1
)

echo #!/bin/sh > .git\hooks\pre-commit
echo echo Running spotlessApply before commit... >> .git\hooks\pre-commit
echo ./gradlew spotlessApply >> .git\hooks\pre-commit
echo if [ $? -ne 0 ]; then >> .git\hooks\pre-commit
echo   echo Spotless apply failed. Please fix formatting errors. >> .git\hooks\pre-commit
echo   exit 1 >> .git\hooks\pre-commit
echo fi >> .git\hooks\pre-commit
echo git add . >> .git\hooks\pre-commit

echo Hook installed successfully! Your code will automatically format when committing.
pause
