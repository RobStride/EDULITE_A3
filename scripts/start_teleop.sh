#!/bin/bash
#
# EL-A3 Master-Slave Teleoperation Launch Script
#
# Features:
# 1. Configure CAN interfaces (can0-can4)
# 2. Launch master-slave teleoperation program
#
# Usage:
#   ./start_teleop.sh                    # Use default configuration
#   ./start_teleop.sh --slaves can1      # Only use can1 as slave arm
#   ./start_teleop.sh --help             # Show help
#

set -e

# Default configuration
MASTER_CAN="can0"
SLAVE_CANS="can1,can2,can3,can4"
BITRATE=1000000
MASTER_KD=1.0
SLAVE_KP=150.0
SLAVE_KD=1.0
RATE=200.0
SKIP_CAN_SETUP=false

# Color output
RED='\033[0;31m'
GREEN='\033[0;32m'
YELLOW='\033[1;33m'
NC='\033[0m' # No Color

print_info() {
    echo -e "${GREEN}[INFO]${NC} $1"
}

print_warn() {
    echo -e "${YELLOW}[WARN]${NC} $1"
}

print_error() {
    echo -e "${RED}[ERROR]${NC} $1"
}

# Show help
show_help() {
    echo "EL-A3 Master-Slave Teleoperation Launch Script"
    echo ""
    echo "Usage: $0 [options]"
    echo ""
    echo "Options:"
    echo "  --master CAN      Master arm CAN interface (default: $MASTER_CAN)"
    echo "  --slaves CANs     Slave arm CAN interface list, comma-separated (default: $SLAVE_CANS)"
    echo "  --bitrate RATE    CAN bitrate (default: $BITRATE)"
    echo "  --master-kd KD    Master arm damping coefficient (default: $MASTER_KD)"
    echo "  --slave-kp KP     Slave arm position gain (default: $SLAVE_KP)"
    echo "  --slave-kd KD     Slave arm velocity gain (default: $SLAVE_KD)"
    echo "  --rate HZ         Control rate (default: $RATE)"
    echo "  --skip-can-setup  Skip CAN interface configuration"
    echo "  -h, --help        Show this help message"
    echo ""
    echo "Examples:"
    echo "  $0                              # Use default configuration"
    echo "  $0 --slaves can1                # Only use can1 as slave arm"
    echo "  $0 --slaves can1,can2           # Use can1 and can2 as slave arms"
    echo "  $0 --slave-kp 100 --slave-kd 2  # Custom slave arm control parameters"
    echo ""
}

# Parse command line arguments
while [[ $# -gt 0 ]]; do
    case $1 in
        --master)
            MASTER_CAN="$2"
            shift 2
            ;;
        --slaves)
            SLAVE_CANS="$2"
            shift 2
            ;;
        --bitrate)
            BITRATE="$2"
            shift 2
            ;;
        --master-kd)
            MASTER_KD="$2"
            shift 2
            ;;
        --slave-kp)
            SLAVE_KP="$2"
            shift 2
            ;;
        --slave-kd)
            SLAVE_KD="$2"
            shift 2
            ;;
        --rate)
            RATE="$2"
            shift 2
            ;;
        --skip-can-setup)
            SKIP_CAN_SETUP=true
            shift
            ;;
        -h|--help)
            show_help
            exit 0
            ;;
        *)
            print_error "Unknown option: $1"
            show_help
            exit 1
            ;;
    esac
done

# Get script directory
SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"

# Configure a single CAN interface
setup_can_interface() {
    local iface=$1
    local bitrate=$2
    
    # Check if interface exists
    if ! ip link show "$iface" &>/dev/null; then
        print_warn "CAN interface $iface does not exist, skipping"
        return 1
    fi
    
    # Bring down interface first (if already up)
    sudo ip link set "$iface" down 2>/dev/null || true
    
    # Set bitrate and bring up
    if sudo ip link set "$iface" type can bitrate "$bitrate"; then
        if sudo ip link set "$iface" up; then
            print_info "CAN interface $iface configured (bitrate=$bitrate)"
            return 0
        fi
    fi
    
    print_error "Failed to configure CAN interface $iface"
    return 1
}

# Configure all CAN interfaces
setup_all_can_interfaces() {
    print_info "Configuring CAN interfaces..."
    
    # Configure master arm
    if ! setup_can_interface "$MASTER_CAN" "$BITRATE"; then
        print_error "Master arm CAN interface configuration failed"
        exit 1
    fi
    
    # Configure slave arms
    IFS=',' read -ra SLAVES <<< "$SLAVE_CANS"
    for slave in "${SLAVES[@]}"; do
        slave=$(echo "$slave" | xargs)  # Trim whitespace
        if [[ -n "$slave" ]]; then
            setup_can_interface "$slave" "$BITRATE" || true
        fi
    done
    
    print_info "CAN interface configuration complete"
}

# Main function
main() {
    echo "============================================================"
    echo "  EL-A3 Master-Slave Teleoperation"
    echo "============================================================"
    echo "  Master: $MASTER_CAN (zero-torque mode, Kd=$MASTER_KD)"
    echo "  Slaves: $SLAVE_CANS (position following, Kp=$SLAVE_KP, Kd=$SLAVE_KD)"
    echo "  Bitrate: $BITRATE"
    echo "  Control rate: $RATE Hz"
    echo "============================================================"
    echo ""
    
    # Configure CAN interfaces
    if [[ "$SKIP_CAN_SETUP" == "false" ]]; then
        setup_all_can_interfaces
    else
        print_info "Skipping CAN interface configuration"
    fi
    
    # Launch teleoperation program
    print_info "Launching teleoperation program..."
    echo ""
    
    python3 "$SCRIPT_DIR/teleop_master_slave.py" \
        --master "$MASTER_CAN" \
        --slaves "$SLAVE_CANS" \
        --master-kd "$MASTER_KD" \
        --slave-kp "$SLAVE_KP" \
        --slave-kd "$SLAVE_KD" \
        --rate "$RATE"
}

# Run
main
