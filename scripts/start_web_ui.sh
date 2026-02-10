#!/bin/bash
# EL-A3 Web UI Startup Script

# Colors
GREEN='\033[0;32m'
BLUE='\033[0;34m'
YELLOW='\033[1;33m'
NC='\033[0m' # No Color

echo -e "${BLUE}"
echo "╔═══════════════════════════════════════════════════════════╗"
echo "║         EL-A3 Web Control Interface                       ║"
echo "╚═══════════════════════════════════════════════════════════╝"
echo -e "${NC}"

# Get script directory
SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
WORKSPACE_DIR="$(dirname "$SCRIPT_DIR")"

# Default parameters
HOST="0.0.0.0"
PORT="5000"

# Parse arguments
while [[ $# -gt 0 ]]; do
    case $1 in
        --host)
            HOST="$2"
            shift 2
            ;;
        --port)
            PORT="$2"
            shift 2
            ;;
        -h|--help)
            echo "Usage: $0 [--host HOST] [--port PORT]"
            echo "  --host HOST  Bind address (default: 0.0.0.0)"
            echo "  --port PORT  Port number (default: 5000)"
            exit 0
            ;;
        *)
            shift
            ;;
    esac
done

echo -e "${GREEN}Starting Web UI Server...${NC}"
echo -e "  Host: ${YELLOW}$HOST${NC}"
echo -e "  Port: ${YELLOW}$PORT${NC}"
echo ""
echo -e "Access the web interface at: ${YELLOW}http://localhost:$PORT${NC}"
echo ""

# Source ROS2 environment
source /opt/ros/humble/setup.bash

# Source workspace (if built)
if [ -f "$WORKSPACE_DIR/install/setup.bash" ]; then
    source "$WORKSPACE_DIR/install/setup.bash"
fi

# Install Python dependencies if needed
pip3 install flask flask-socketio eventlet python-socketio --quiet 2>/dev/null

# Change to package directory
cd "$WORKSPACE_DIR/ros2_ws/src/el_a3_web_ui"

# Run the web server
python3 -m el_a3_web_ui.web_server --host "$HOST" --port "$PORT"
