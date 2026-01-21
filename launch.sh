#!/bin/bash
# Launcher script for Auto Pick & Place
# Ensures proper environment and permissions

echo "╔═══════════════════════════════════════════════════════════╗"
echo "║         AiNex Auto Pick & Place - Launcher               ║"
echo "╚═══════════════════════════════════════════════════════════╝"
echo ""

# Check if running in Docker
if [ ! -f "/.dockerenv" ]; then
    echo "⚠ WARNING: This should run inside Docker container"
    echo ""
    echo "To run properly:"
    echo "  docker exec -it -u ubuntu ainex bash"
    echo "  cd /home/ubuntu/ros_ws/src/pick_place"
    echo "  ./launch.sh"
    echo ""
    read -p "Continue anyway? (y/n) " -n 1 -r
    echo ""
    if [[ ! $REPLY =~ ^[Yy]$ ]]; then
        exit 1
    fi
fi

# Navigate to script directory
cd "$(dirname "$0")"

# Check if script exists
if [ ! -f "auto_pick_place.py" ]; then
    echo "✗ Error: auto_pick_place.py not found"
    exit 1
fi

# Check dependencies
echo "Checking dependencies..."
python3 -c "import sys; sys.path.insert(0, '/home/ubuntu/ros_ws/src/ainex_driver/ainex_sdk/src'); from Board import Board" 2>/dev/null
if [ $? -eq 0 ]; then
    echo "✓ Board SDK available"
else
    echo "✗ Board SDK not available"
    echo "  Make sure you're in the Docker container"
    exit 1
fi

# Check for espeak
if command -v espeak &> /dev/null; then
    echo "✓ espeak available"
else
    echo "⚠ espeak not available (voice feedback disabled)"
fi

# Check action files
if [ -d "/home/ubuntu/software/ainex_controller/ActionGroups" ]; then
    echo "✓ Action files found"
else
    echo "⚠ Action files not found at expected location"
fi

echo ""
echo "Launching Auto Pick & Place..."
echo "═══════════════════════════════════════════════════════════"
echo ""

# Run the script
python3 ./auto_pick_place.py

exit_code=$?

echo ""
echo "═══════════════════════════════════════════════════════════"
if [ $exit_code -eq 0 ]; then
    echo "✓ Program completed successfully"
else
    echo "✗ Program exited with code: $exit_code"
fi
echo ""

exit $exit_code
