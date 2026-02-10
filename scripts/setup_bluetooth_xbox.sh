#!/bin/bash
# Bluetooth Xbox controller setup script

echo "========================================"
echo "   Bluetooth Xbox Controller Setup Wizard"
echo "========================================"
echo ""

# Color definitions
RED='\033[0;31m'
GREEN='\033[0;32m'
YELLOW='\033[1;33m'
BLUE='\033[0;34m'
NC='\033[0m' # No Color

# Step 1: Check and load joydev module
echo -e "${BLUE}Step 1: Load joydev kernel module${NC}"
if lsmod | grep -q joydev; then
    echo -e "${GREEN}✓${NC} joydev module is loaded"
else
    echo -e "${YELLOW}⟳${NC} Loading joydev module..."
    sudo modprobe joydev
    if [ $? -eq 0 ]; then
        echo -e "${GREEN}✓${NC} joydev module loaded successfully"
        
        # Set up auto-load on boot
        if ! grep -q "^joydev" /etc/modules 2>/dev/null; then
            echo "joydev" | sudo tee -a /etc/modules > /dev/null
            echo -e "${GREEN}✓${NC} Set to auto-load on boot"
        fi
    else
        echo -e "${RED}✗${NC} Loading failed, please check permissions"
    fi
fi
echo ""

# Step 2: Check Bluetooth service
echo -e "${BLUE}Step 2: Check Bluetooth service${NC}"
if systemctl is-active --quiet bluetooth; then
    echo -e "${GREEN}✓${NC} Bluetooth service is running"
else
    echo -e "${YELLOW}⟳${NC} Starting Bluetooth service..."
    sudo systemctl start bluetooth
    if [ $? -eq 0 ]; then
        echo -e "${GREEN}✓${NC} Bluetooth service started"
    else
        echo -e "${RED}✗${NC} Bluetooth service failed to start"
    fi
fi
echo ""

# Step 3: Check controller device
echo -e "${BLUE}Step 3: Check controller device${NC}"
if [ -e /dev/input/js0 ]; then
    echo -e "${GREEN}✓${NC} Found controller device: /dev/input/js0"
    ls -l /dev/input/js0
else
    echo -e "${YELLOW}!${NC} /dev/input/js0 device not found"
    echo ""
    echo "Please follow these steps to connect the controller:"
    echo "1. Hold the pairing button on top of the Xbox controller (next to Xbox logo)"
    echo "2. The Xbox logo should start flashing rapidly"
    echo "3. Find 'Xbox Wireless Controller' in system Bluetooth settings"
    echo "4. Click connect"
    echo ""
    echo "Or use command line pairing:"
    echo "  bluetoothctl"
    echo "  Then enter: scan on"
    echo "  After finding the controller MAC address: pair XX:XX:XX:XX:XX:XX"
    echo ""
fi
echo ""

# Step 4: Check user permissions
echo -e "${BLUE}Step 4: Check user permissions${NC}"
if groups | grep -q input; then
    echo -e "${GREEN}✓${NC} User is in the input group"
else
    echo -e "${YELLOW}⟳${NC} Adding user to input group..."
    sudo usermod -a -G input $USER
    echo -e "${GREEN}✓${NC} Added to input group"
    echo -e "${YELLOW}!${NC} Please log out and log back in for changes to take effect"
fi
echo ""

# Step 5: List all input devices
echo -e "${BLUE}Step 5: Current input devices${NC}"
echo "Joystick devices:"
ls -l /dev/input/js* 2>/dev/null || echo "  (none)"
echo ""
echo "Recent input event devices:"
ls -lt /dev/input/event* | head -5
echo ""

# Step 6: Scan for Xbox controller
echo -e "${BLUE}Step 6: Scan for Xbox controller${NC}"
XBOX_DEVICE=$(cat /proc/bus/input/devices 2>/dev/null | grep -i "xbox\|wireless controller" -A 5)
if [ -n "$XBOX_DEVICE" ]; then
    echo -e "${GREEN}✓${NC} Found Xbox controller:"
    echo "$XBOX_DEVICE"
else
    echo -e "${YELLOW}!${NC} Xbox controller not found in /proc/bus/input/devices"
    echo "Possible reasons:"
    echo "  - Controller is not connected"
    echo "  - Need to re-pair"
    echo "  - Bluetooth driver issue"
fi
echo ""

# Check if joystick tools are installed
echo -e "${BLUE}Step 7: Check test tools${NC}"
if command -v jstest &> /dev/null; then
    echo -e "${GREEN}✓${NC} jstest is installed"
else
    echo -e "${YELLOW}!${NC} jstest is not installed"
    echo "Install command: sudo apt install joystick"
fi
echo ""

# Summary
echo "========================================"
echo "   Setup Complete"
echo "========================================"
echo ""

if [ -e /dev/input/js0 ]; then
    echo -e "${GREEN}✓ Controller device is ready!${NC}"
    echo ""
    echo "Next steps:"
    echo "1. Test the controller:"
    echo "   jstest /dev/input/js0"
    echo ""
    echo "2. Test ROS2 joy node:"
    echo "   ros2 run joy joy_node"
    echo "   ros2 topic echo /joy"
    echo ""
    echo "3. Launch full control system:"
    echo "   cd /home/wy/RS/A3/ros2_ws"
    echo "   source install/setup.bash"
    echo "   ros2 launch el_a3_teleop simple_teleop.launch.py"
else
    echo -e "${YELLOW}⚠ Controller device is not ready${NC}"
    echo ""
    echo "Please do the following:"
    echo ""
    echo "1. Ensure the Xbox controller is paired via Bluetooth:"
    echo "   - Open System Settings -> Bluetooth"
    echo "   - Hold the pairing button on the controller (next to Xbox logo)"
    echo "   - Connect 'Xbox Wireless Controller'"
    echo ""
    echo "2. Re-run this script after connecting:"
    echo "   bash $0"
    echo ""
    echo "3. View detailed setup guide:"
    echo "   cat /home/wy/RS/A3/BLUETOOTH_XBOX_SETUP.md"
fi
echo ""




