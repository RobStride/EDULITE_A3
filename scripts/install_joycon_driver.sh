#!/bin/bash
# Joy-Con driver installation script
# 
# This script installs the drivers and tools required for Nintendo Switch Joy-Con controllers
#
# Usage:
#   sudo ./install_joycon_driver.sh

set -e

echo "=========================================="
echo "  Joy-Con Driver Installation Script"
echo "=========================================="
echo ""

# Check if running as root
if [ "$EUID" -ne 0 ]; then
    echo "Please run this script with sudo"
    exit 1
fi

# 1. Check kernel version
echo "=== 1. Check kernel version ==="
KERNEL_VERSION=$(uname -r | cut -d. -f1-2)
echo "Kernel version: $(uname -r)"

if [[ $(echo "$KERNEL_VERSION >= 5.16" | bc -l) -eq 1 ]]; then
    echo "✓ Kernel version >= 5.16, hid-nintendo module is built-in"
else
    echo "⚠ Kernel version < 5.16, manual installation of hid-nintendo module may be required"
fi
echo ""

# 2. Load hid-nintendo module
echo "=== 2. Load hid-nintendo module ==="
if lsmod | grep -q hid_nintendo; then
    echo "✓ hid-nintendo module is already loaded"
else
    echo "Loading hid-nintendo module..."
    modprobe hid-nintendo || {
        echo "⚠ Failed to load hid-nintendo module"
        echo "Trying to install dkms version..."
        apt install -y dkms
        # If module doesn't exist, may need to compile from source
    }
    
    # Set up auto-load on boot
    if ! grep -q "hid-nintendo" /etc/modules-load.d/hid-nintendo.conf 2>/dev/null; then
        echo "hid-nintendo" > /etc/modules-load.d/hid-nintendo.conf
        echo "✓ Set hid-nintendo module to auto-load on boot"
    fi
fi
echo ""

# 3. Install joycond
echo "=== 3. Install joycond ==="
if which joycond >/dev/null 2>&1; then
    echo "✓ joycond is already installed"
else
    echo "Installing joycond..."
    
    # Try installing from package manager
    if apt install -y joycond 2>/dev/null; then
        echo "✓ joycond installed successfully from apt"
    else
        # Compile from source
        echo "apt installation failed, trying to compile from source..."
        
        # Install dependencies
        apt install -y cmake libevdev-dev libudev-dev
        
        # Clone and compile
        TEMP_DIR=$(mktemp -d)
        cd "$TEMP_DIR"
        git clone https://github.com/DanielOgorworking/joycond.git || \
            git clone https://gitlab.com/jcoffland/joycond.git
        cd joycond
        cmake .
        make
        make install
        
        # Copy udev rules
        cp udev/89-joycond.rules /etc/udev/rules.d/
        udevadm control --reload-rules
        udevadm trigger
        
        # Install systemd service
        cp systemd/joycond.service /etc/systemd/system/
        
        cd /
        rm -rf "$TEMP_DIR"
        
        echo "✓ joycond compiled and installed from source successfully"
    fi
fi

# Enable joycond service
echo "Enabling joycond service..."
systemctl enable joycond
systemctl start joycond
echo "✓ joycond service started"
echo ""

# 4. Install Python dependencies
echo "=== 4. Install Python dependencies ==="
pip3 install evdev
echo "✓ evdev installed"
echo ""

# 5. Configure udev rules (allow access for regular users)
echo "=== 5. Configure udev rules ==="
cat > /etc/udev/rules.d/50-nintendo.rules << 'EOF'
# Nintendo Switch Joy-Con / Pro Controller udev rules

# Joy-Con (L)
KERNEL=="hidraw*", SUBSYSTEM=="hidraw", ATTRS{idVendor}=="057e", ATTRS{idProduct}=="2006", MODE="0666"
# Joy-Con (R)
KERNEL=="hidraw*", SUBSYSTEM=="hidraw", ATTRS{idVendor}=="057e", ATTRS{idProduct}=="2007", MODE="0666"
# Pro Controller
KERNEL=="hidraw*", SUBSYSTEM=="hidraw", ATTRS{idVendor}=="057e", ATTRS{idProduct}=="2009", MODE="0666"
# Combined Joy-Con
KERNEL=="hidraw*", SUBSYSTEM=="hidraw", ATTRS{idVendor}=="057e", ATTRS{idProduct}=="200e", MODE="0666"

# evdev access
KERNEL=="event*", SUBSYSTEM=="input", ATTRS{name}=="*Joy-Con*", MODE="0666"
KERNEL=="event*", SUBSYSTEM=="input", ATTRS{name}=="*Nintendo*", MODE="0666"
EOF

udevadm control --reload-rules
udevadm trigger
echo "✓ udev rules configured"
echo ""

# 6. Show Bluetooth pairing instructions
echo "=========================================="
echo "  Installation complete!"
echo "=========================================="
echo ""
echo "=== Joy-Con Bluetooth Pairing Steps ==="
echo ""
echo "1. Hold the sync button on the side of the Joy-Con to enter pairing mode"
echo "   (LED lights will flash rapidly)"
echo ""
echo "2. Pair using bluetoothctl:"
echo "   $ bluetoothctl"
echo "   [bluetooth]# scan on"
echo "   (Wait for Joy-Con to be discovered...)"
echo "   [bluetooth]# pair XX:XX:XX:XX:XX:XX"
echo "   [bluetooth]# trust XX:XX:XX:XX:XX:XX"
echo "   [bluetooth]# connect XX:XX:XX:XX:XX:XX"
echo ""
echo "3. Verify connection:"
echo "   $ ls /dev/input/js*"
echo "   $ cat /proc/bus/input/devices | grep -A 5 Joy-Con"
echo ""
echo "=== Launch Joy-Con IMU Teleoperation ==="
echo ""
echo "Simulation mode:"
echo "  ros2 launch el_a3_teleop joycon_imu_teleop.launch.py"
echo ""
echo "Real hardware:"
echo "  ros2 launch el_a3_teleop joycon_imu_teleop.launch.py use_mock_hardware:=false can_interface:=can0"
echo ""
