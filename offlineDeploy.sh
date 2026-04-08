#!/bin/sh
echo "Starting MARSLib Offline Deploy (Competition Pits Mode)..."
echo "This will use ONLY the dependencies currently cached on your computer."

./gradlew deploy --offline --no-daemon

if [ $? -eq 0 ]; then
    echo ""
    echo "---"
    echo "OFFLINE DEPLOY SUCCESSFUL!"
else
    echo ""
    echo "---"
    echo "DEPLOY FAILED. Check if you are connected to the robot radio."
fi
