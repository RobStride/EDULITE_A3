#!/bin/bash
# Batch test return-to-zero error for different Kp values
# Usage: ./batch_kp_test.sh

WORKSPACE="/home/wy/RS/A3/ros2_ws"
XACRO_FILE="${WORKSPACE}/src/el_a3_description/urdf/el_a3_ros2_control.xacro"
RESULT_FILE="/home/wy/RS/A3/kp_test_results.txt"

# Kp values to test
KP_VALUES=(50 100 200 300)
KD_VALUE=1.0

echo "========================================" | tee $RESULT_FILE
echo "  Kp Return-to-Zero Error Test" | tee -a $RESULT_FILE
echo "  Kd = ${KD_VALUE}" | tee -a $RESULT_FILE
echo "  Test time: $(date)" | tee -a $RESULT_FILE
echo "========================================" | tee -a $RESULT_FILE

for KP in "${KP_VALUES[@]}"; do
    echo ""
    echo "----------------------------------------"
    echo "Testing Kp = ${KP}"
    echo "----------------------------------------"
    
    # Stop existing processes
    pkill -f "ros2" 2>/dev/null
    sleep 3
    
    # Modify Kp value in xacro file
    sed -i "s/<param name=\"position_kp\">.*<\/param>/<param name=\"position_kp\">${KP}.0<\/param>/" $XACRO_FILE
    sed -i "s/<param name=\"position_kd\">.*<\/param>/<param name=\"position_kd\">${KD_VALUE}<\/param>/" $XACRO_FILE
    
    echo "Set Kp=${KP}, Kd=${KD_VALUE}"
    
    # Rebuild
    cd $WORKSPACE
    source /opt/ros/humble/setup.bash
    colcon build --packages-select el_a3_description > /dev/null 2>&1
    source install/setup.bash
    
    # Launch robot
    echo "Launching controller..."
    ros2 launch el_a3_moveit_config robot.launch.py can_interface:=can1 > /tmp/ros_launch_${KP}.log 2>&1 &
    LAUNCH_PID=$!
    
    # Wait for startup
    sleep 25
    
    # Run return-to-zero test
    echo "Running return-to-zero test..."
    timeout 60 /usr/bin/python3 /home/wy/RS/A3/scripts/move_to_zero.py > /tmp/zero_test_${KP}.log 2>&1
    
    # Wait for stabilization
    sleep 3
    
    # Collect final position
    echo "" >> $RESULT_FILE
    echo "Kp = ${KP}:" >> $RESULT_FILE
    
    # Get last position data
    LAST_POS=$(grep "Current position" /tmp/zero_test_${KP}.log | tail -1)
    echo "  ${LAST_POS}" >> $RESULT_FILE
    
    # Extract error from log
    echo "  Result: $(tail -5 /tmp/zero_test_${KP}.log | head -1)" >> $RESULT_FILE
    
    echo "Kp=${KP} test complete"
    
    # Stop processes
    kill $LAUNCH_PID 2>/dev/null
    pkill -f "ros2" 2>/dev/null
    sleep 2
done

echo ""
echo "========================================"
echo "  All tests complete!"
echo "  Results saved to: $RESULT_FILE"
echo "========================================"
cat $RESULT_FILE

















