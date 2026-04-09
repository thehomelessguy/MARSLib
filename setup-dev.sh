#!/bin/bash
echo "Installing Git Hooks for Local Development..."
git config core.hooksPath .githooks
chmod +x .githooks/pre-commit
echo "Done! Your commits will now be automatically formatted by Spotless."
