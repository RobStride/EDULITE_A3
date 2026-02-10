#!/bin/bash
# Xbox controller control system test script

echo "=================================="
echo "Xbox Controller Control System Test"
echo "=================================="
echo ""

# Color definitions
RED='\033[0;31m'
GREEN='\033[0;32m'
YELLOW='\033[1;33m'
NC='\033[0m' # No Color

# Check ROS2 environment
echo -n "1. Checking ROS2 environment... "
if [ -z "$ROS_DISTRO" ]; then
    echo -e "${RED}Failed${NC}"
    echo "   Please source ROS2 environment first: source /opt/ros/humble/setup.bash"
    exit 1
else
    echo -e "${GREEN}Passed${NC} (ROS_DISTRO=$ROS_DISTRO)"
fi

# Check workspace
echo -n "2. Checking workspace... "
if [ -f "/home/wy/RS/A3/ros2_ws/install/setup.bash" ]; then
    echo -e "${GREEN}Passed${NC}"
else
    echo -e "${RED}Failed${NC}"
    echo "   Workspace not built, please run: colcon build"
    exit 1
fi

# Check el_a3_teleop package
echo -n "3. Checking el_a3_teleop package... "
source /home/wy/RS/A3/ros2_ws/install/setup.bash
if ros2 pkg list | grep -q el_a3_teleop; then
    echo -e "${GREEN}Passed${NC}"
else
    echo -e "${RED}Failed${NC}"
    echo "   el_a3_teleop package not found"
    exit 1
fi

# Check executables
echo -n "4. Checking executables... "
if ros2 pkg executables el_a3_teleop | grep -q xbox_servo_teleop_node; then
    echo -e "${GREEN}Passed${NC}"
else
    echo -e "${RED}Failed${NC}"
    echo "   xbox_servo_teleop_node not found"
    exit 1
fi

# Check launch files
echo -n "5. Checking launch files... "
LAUNCH_DIR="/home/wy/RS/A3/ros2_ws/install/el_a3_teleop/share/el_a3_teleop/launch"
if [ -f "$LAUNCH_DIR/complete_teleop.launch.py" ]; then
    echo -e "${GREEN}Passed${NC}"
else
    echo -e "${RED}Failed${NC}"
    echo "   Launch file not found"
    exit 1
fi

# Check config files
echo -n "6. Checking config files... "
CONFIG_DIR="/home/wy/RS/A3/ros2_ws/install/el_a3_teleop/share/el_a3_teleop/config"
if [ -f "$CONFIG_DIR/xbox_servo_teleop.yaml" ]; then
    echo -e "${GREEN}Passed${NC}"
else
    echo -e "${RED}Failed${NC}"
    echo "   Config file not found"
    exit 1
fi

# Check joy package
echo -n "7. Checking joy package... "
if ros2 pkg list | grep -q joy; then
    echo -e "${GREEN}Passed${NC}"
else
    echo -e "${YELLOW}Warning${NC}"
    echo "   joy package not installed, please run: sudo apt install ros-humble-joy"
fi

# Check moveit_servo package
echo -n "8. Checking moveit_servo package... "
if ros2 pkg list | grep -q moveit_servo; then
    echo -e "${GREEN}Passed${NC}"
else
    echo -e "${YELLOW}Warning${NC}"
    echo "   moveit_servo package not installed, please run: sudo apt install ros-humble-moveit-servo"
fi

# Check controller device
echo -n "9. Checking controller device... "
if [ -e "/dev/input/js0" ]; then
    echo -e "${GREEN}Passed${NC}"
    echo "   Device: /dev/input/js0"
    
    # Check permissions
    if [ -r "/dev/input/js0" ]; then
        echo "   Permissions: ${GREEN}Readable${NC}"
    else
        echo "   Permissions: ${YELLOW}Not readable${NC}"
        echo "   Fix: sudo chmod 666 /dev/input/js0"
    fi
else
    echo -e "${YELLOW}Not detected${NC}"
    echo "   Please connect the Xbox controller"
fi

echo ""
echo "=================================="
echo "Test complete!"
echo "=================================="
echo ""
echo "Available launch commands:"
echo ""
echo "1. Full system (recommended):"
echo "   ${GREEN}ros2 launch el_a3_teleop complete_teleop.launch.py${NC}"
echo ""
echo "2. Controller only (requires robot and MoveIt Servo to be running):"
echo "   ${GREEN}ros2 launch el_a3_teleop xbox_servo_teleop.launch.py${NC}"
echo ""
echo "3. Test controller input:"
echo "   ${GREEN}ros2 run joy joy_node${NC}"
echo "   ${GREEN}ros2 topic echo /joy${NC}"
echo ""
echo "Detailed usage guide:"
echo "   cat /home/wy/RS/A3/ros2_ws/src/el_a3_teleop/QUICK_START.md"
echo ""




