#!/bin/bash
# EL-A3 multi-CAN interface configuration script
# Used to configure CAN communication interfaces for multiple robot arms

# Configuration parameters
BITRATE=1000000  # 1Mbps
CAN_INTERFACES=(can0 can1 can2 can3)

# Color output
RED='\033[0;31m'
GREEN='\033[0;32m'
YELLOW='\033[1;33m'
NC='\033[0m' # No Color

echo "=================================="
echo "EL-A3 Multi-CAN Interface Setup"
echo "=================================="

# Check for root privileges
if [ "$EUID" -ne 0 ]; then
    echo -e "${YELLOW}Warning: sudo privileges required to configure CAN interfaces${NC}"
    exec sudo "$0" "$@"
fi

# Get number of interfaces to configure
NUM_INTERFACES=${1:-2}  # Default: configure 2 interfaces

echo -e "Configuring ${GREEN}$NUM_INTERFACES${NC} CAN interfaces..."
echo ""

# Configure specified number of CAN interfaces
for i in $(seq 0 $((NUM_INTERFACES - 1))); do
    CAN_IF="${CAN_INTERFACES[$i]}"
    
    echo -n "Configuring $CAN_IF... "
    
    # Check if interface exists
    if ! ip link show "$CAN_IF" &> /dev/null; then
        echo -e "${RED}Interface does not exist${NC}"
        continue
    fi
    
    # Bring down interface
    ip link set "$CAN_IF" down 2>/dev/null
    
    # Configure bitrate
    if ip link set "$CAN_IF" type can bitrate $BITRATE; then
        # Bring up interface
        if ip link set "$CAN_IF" up; then
            echo -e "${GREEN}Success${NC}"
        else
            echo -e "${RED}Failed to bring up${NC}"
        fi
    else
        echo -e "${RED}Configuration failed${NC}"
    fi
done

echo ""
echo "=================================="
echo "CAN Interface Status"
echo "=================================="

# Display all CAN interface status
for i in $(seq 0 $((NUM_INTERFACES - 1))); do
    CAN_IF="${CAN_INTERFACES[$i]}"
    
    if ip link show "$CAN_IF" &> /dev/null; then
        STATE=$(ip link show "$CAN_IF" | grep -oP '(?<=state )\w+')
        if [ "$STATE" == "UP" ]; then
            echo -e "$CAN_IF: ${GREEN}$STATE${NC}"
        else
            echo -e "$CAN_IF: ${RED}$STATE${NC}"
        fi
    else
        echo -e "$CAN_IF: ${YELLOW}Does not exist${NC}"
    fi
done

echo ""
echo "=================================="
echo "Usage Examples"
echo "=================================="
echo "# Launch dual-arm system"
echo "ros2 launch el_a3_description multi_arm_control.launch.py"
echo ""
echo "# Launch master-slave teleoperation"
echo "ros2 launch el_a3_teleop master_slave.launch.py master_ns:=arm1 slave_ns:=arm2"
echo ""
