#!/bin/bash
# One-click Xbox controller launch script

echo "========================================"
echo "   Launching Xbox Controller System"
echo "========================================"
echo ""

# Color definitions
RED='\033[0;31m'
GREEN='\033[0;32m'
YELLOW='\033[1;33m'
BLUE='\033[0;34m'
NC='\033[0m'

# Enter workspace
cd /home/wy/RS/A3/ros2_ws

# Source environment
echo -e "${BLUE}Setting up ROS2 environment...${NC}"
source /opt/ros/humble/setup.bash
source install/setup.bash
echo -e "${GREEN}✓${NC} Environment set up"
echo ""

# Check controller connection
echo -e "${BLUE}Checking controller connection...${NC}"

if [ -e /dev/input/js0 ]; then
    echo -e "${GREEN}✓${NC} Found controller device: /dev/input/js0"
    DEVICE_PARAM="device_name:=/dev/input/js0"
    echo ""
elif [ -e /dev/input/js1 ]; then
    echo -e "${GREEN}✓${NC} Found controller device: /dev/input/js1"
    DEVICE_PARAM="device_name:=/dev/input/js1"
    echo ""
else
    echo -e "${YELLOW}!${NC} No /dev/input/js device found"
    echo "Trying to find event device..."
    
    # Find Xbox event device
    XBOX_EVENT=$(ls -t /dev/input/event* | head -1)
    if [ -n "$XBOX_EVENT" ]; then
        echo -e "${GREEN}✓${NC} Using event device: $XBOX_EVENT"
        DEVICE_PARAM="device_name:=$XBOX_EVENT"
        echo ""
    else
        echo -e "${RED}✗${NC} No input device found"
        echo ""
        echo "Please do the following:"
        echo "1. Ensure the Xbox controller is connected (check Bluetooth settings)"
        echo "2. Press any button on the controller to activate it"
        echo "3. Run: sudo modprobe joydev"
        echo "4. Re-run this script"
        echo ""
        exit 1
    fi
fi

# Test controller input
echo -e "${BLUE}Testing controller input (3 seconds)...${NC}"
echo "Please move sticks or press buttons..."
echo ""

timeout 3 ros2 run joy joy_node --ros-args -p $DEVICE_PARAM &
JOY_PID=$!
sleep 1

# Check for joy data
if ros2 topic list | grep -q "/joy"; then
    echo -e "${GREEN}✓${NC} Controller node is running"
    
    # Show some joy data
    timeout 2 ros2 topic echo /joy --once 2>/dev/null && echo -e "${GREEN}✓${NC} Controller input is working" || echo -e "${YELLOW}!${NC} No controller input detected, please move sticks"
fi

# Stop test node
kill $JOY_PID 2>/dev/null
sleep 1

echo ""
echo "========================================"
echo -e "${GREEN}Launching Full Control System${NC}"
echo "========================================"
echo ""
echo "Controls:"
echo "  Left stick: X/Y translation"
echo "  LT/RT: Z-axis up/down"
echo "  Right stick: Yaw/Pitch rotation"
echo "  LB/RB: Roll rotation"
echo ""
echo "Press Ctrl+C to stop the system"
echo ""
sleep 2

# Launch full system
ros2 launch el_a3_teleop simple_teleop.launch.py




