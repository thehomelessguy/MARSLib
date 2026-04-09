@echo off
echo Installing Git Hooks for Local Development...
git config core.hooksPath .githooks
echo Done! Your commits will now be automatically formatted by Spotless.
pause
