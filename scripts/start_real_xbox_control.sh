#!/bin/bash
# EL-A3 real hardware Xbox controller launch script
# 
# Usage: ./start_real_xbox_control.sh [can_interface]
#   can_interface: CAN interface name (default: can0)

CAN_INTERFACE=${1:-can0}

echo "============================================"
echo "EL-A3 Real Hardware Xbox Controller Launch"
echo "============================================"

# Check CAN interface
echo "Checking CAN interface ${CAN_INTERFACE}..."
if ! ip link show ${CAN_INTERFACE} &> /dev/null; then
    echo "Error: CAN interface ${CAN_INTERFACE} does not exist!"
    echo ""
    echo "Please set up the CAN interface first:"
    echo "  sudo ./setup_can.sh ${CAN_INTERFACE}"
    echo ""
    echo "Or check if the CAN adapter is connected"
    exit 1
fi

# Check if CAN interface is UP
if ! ip link show ${CAN_INTERFACE} | grep -q "UP"; then
    echo "CAN interface ${CAN_INTERFACE} is not up, bringing it up..."
    sudo ip link set ${CAN_INTERFACE} up
    if [ $? -ne 0 ]; then
        echo "Error: Failed to bring up CAN interface"
        exit 1
    fi
fi

echo "CAN interface ${CAN_INTERFACE} is ready"

# Check controller
echo ""
echo "Checking Xbox controller..."
if [ ! -e /dev/input/js0 ]; then
    echo "Warning: Controller not detected (/dev/input/js0)"
    echo "Please ensure the Xbox controller is connected"
fi

# Launch ROS2 control
echo ""
echo "Launching real hardware control..."
echo "Controls:"
echo "  Left stick Y:  X-axis movement"
echo "  Left stick X:  Y-axis movement"
echo "  LT/RT:         Z-axis movement"
echo "  Right stick:   Yaw/Pitch rotation"
echo "  LB/RB:         Roll rotation"
echo "  A button:      Toggle speed level"
echo "  B button:      Return to home position"
echo ""
echo "Press Ctrl+C to stop"
echo "============================================"

cd /home/wy/RS/A3/ros2_ws
source /opt/ros/humble/setup.bash
source install/setup.bash
ros2 launch el_a3_teleop real_teleop.launch.py can_interface:=${CAN_INTERFACE}

