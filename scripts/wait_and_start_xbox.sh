#!/bin/bash
# Wait for Xbox controller device to appear and automatically launch control system

echo "========================================"
echo "   Waiting for Xbox controller..."
echo "========================================"
echo ""

GREEN='\033[0;32m'
YELLOW='\033[1;33m'
BLUE='\033[0;34m'
NC='\033[0m'

echo -e "${YELLOW}Please perform the following on the Xbox controller:${NC}"
echo "1. Hold the Xbox button for 3 seconds (center button)"
echo "2. Press A/B/X/Y buttons a few times"
echo "3. Move the left and right sticks"
echo ""
echo "Monitoring devices..."
echo ""

# Wait for js0 device to appear (up to 30 seconds)
for i in {1..30}; do
    if [ -e /dev/input/js0 ]; then
        echo -e "${GREEN}✓ Controller device detected!${NC}"
        ls -l /dev/input/js0
        echo ""
        break
    fi
    
    # Show reminder every 5 seconds
    if [ $((i % 5)) -eq 0 ]; then
        echo "Waiting... (${i}/30s) - Press buttons on the controller"
    fi
    
    sleep 1
done

if [ ! -e /dev/input/js0 ]; then
    echo -e "${YELLOW}⚠ Controller device not detected within 30 seconds${NC}"
    echo ""
    echo "Possible solutions:"
    echo ""
    echo "1. Install xpadneo driver:"
    echo "   bash /home/wy/RS/A3/scripts/install_xpadneo.sh"
    echo ""
    echo "2. Reconnect the controller:"
    echo "   - Bluetooth settings -> Disconnect Xbox controller"
    echo "   - Hold Xbox button to reconnect"
    echo ""
    echo "3. View detailed troubleshooting guide:"
    echo "   cat /home/wy/RS/A3/XBOX_QUICK_FIX.md"
    echo ""
    exit 1
fi

# Set permissions
echo "2501" | sudo -S chmod 666 /dev/input/js0 2>/dev/null
echo -e "${GREEN}✓ Permissions set${NC}"
echo ""

# Test controller
echo -e "${BLUE}Testing controller input (2 seconds)...${NC}"
echo "Please move sticks or press buttons..."
echo ""

cd /home/wy/RS/A3/ros2_ws
source /opt/ros/humble/setup.bash
source install/setup.bash

timeout 2 ros2 run joy joy_node --ros-args -p device_name:=/dev/input/js0 > /dev/null 2>&1 &
JOY_PID=$!
sleep 1.5

if ros2 topic list 2>/dev/null | grep -q "/joy"; then
    if timeout 1 ros2 topic echo /joy --once > /dev/null 2>&1; then
        echo -e "${GREEN}✓ Controller input is working!${NC}"
    else
        echo -e "${YELLOW}! Controller node running, but no input detected${NC}"
        echo "  Please move sticks or press buttons"
    fi
else
    echo -e "${YELLOW}! Joy node failed to start${NC}"
fi

kill $JOY_PID 2>/dev/null
wait $JOY_PID 2>/dev/null
sleep 1

echo ""
echo "========================================"
echo -e "${GREEN}   Launching Robot Control System${NC}"
echo "========================================"
echo ""
echo "Controls:"
echo "  • Left stick: X/Y translation (forward/backward/left/right)"
echo "  • LT trigger: Z-axis down"
echo "  • RT trigger: Z-axis up"
echo "  • Right stick: Yaw/Pitch rotation"
echo "  • LB/RB: Roll rotation (left/right roll)"
echo ""
echo "RViz will display the robot model"
echo "Press Ctrl+C to stop the system"
echo ""
sleep 3

# Launch full system
exec ros2 launch el_a3_teleop simple_teleop.launch.py




