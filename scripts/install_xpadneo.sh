#!/bin/bash
# Install xpadneo driver (specifically for Bluetooth Xbox controllers)

echo "========================================"
echo "   Install xpadneo Bluetooth Xbox Controller Driver"
echo "========================================"
echo ""

GREEN='\033[0;32m'
YELLOW='\033[1;33m'
BLUE='\033[0;34m'
NC='\033[0m'

# Check if already installed
if dkms status | grep -q hid-xpadneo; then
    echo -e "${GREEN}xpadneo driver is already installed${NC}"
    echo ""
    echo "Reloading driver..."
    echo "2501" | sudo -S modprobe -r hid-xpadneo 2>/dev/null
    echo "2501" | sudo -S modprobe hid-xpadneo
    echo -e "${GREEN}✓ Driver reloaded${NC}"
    exit 0
fi

echo -e "${BLUE}Downloading xpadneo driver...${NC}"
cd /tmp
rm -rf xpadneo
git clone https://github.com/atar-axis/xpadneo.git

if [ $? -ne 0 ]; then
    echo "Git clone failed, trying to download release version..."
    wget https://github.com/atar-axis/xpadneo/archive/refs/tags/v0.9.5.tar.gz
    tar xzf v0.9.5.tar.gz
    cd xpadneo-0.9.5
else
    cd xpadneo
fi

echo ""
echo -e "${BLUE}Installing xpadneo driver...${NC}"
echo "2501" | sudo -S ./install.sh

if [ $? -eq 0 ]; then
    echo ""
    echo -e "${GREEN}✓ xpadneo driver installed successfully!${NC}"
    echo ""
    echo "Restarting Bluetooth service..."
    echo "2501" | sudo -S systemctl restart bluetooth
    echo ""
    echo -e "${YELLOW}Please disconnect and reconnect the Xbox controller:${NC}"
    echo "1. Disconnect the Xbox controller in Bluetooth settings"
    echo "2. Hold the Xbox button to reconnect"
    echo "3. Then run: ls /dev/input/js*"
else
    echo ""
    echo "Installation failed, please check error messages"
fi




