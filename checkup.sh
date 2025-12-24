#!/bin/bash

# Colors for output
RED='\033[0;31m'
GREEN='\033[0;32m'
YELLOW='\033[1;33m'
NC='\033[0m' # No Color

echo -e "${YELLOW}=== Hexapod Sensor & Control Diagnostic ===${NC}"

# 1. Check if Gazebo is running
if pgrep -x "ruby" > /dev/null || pgrep -x "gz" > /dev/null; then
    echo -e "[${GREEN}OK${NC}] Gazebo process detected."
else
    echo -e "[${RED}FAIL${NC}] Gazebo is NOT running."
    exit 1
fi

# 2. Check Gazebo-side Topics (Discovery)
echo -e "\n${YELLOW}Checking Gazebo Internal Topics...${NC}"
GZ_TOPICS=$(gz topic -l)
for sensor in "scan" "imu" "clock"; do
    if echo "$GZ_TOPICS" | grep -q "$sensor"; then
        echo -e "[${GREEN}OK${NC}] Gazebo publishing: $sensor"
    else
        echo -e "[${RED}FAIL${NC}] Gazebo NOT publishing: $sensor (Check URDF/SDF Plugin)"
    fi
done

# 3. Check ROS 2 Bridge Status
echo -e "\n${YELLOW}Checking ROS 2 Bridge & Topics...${NC}"
ROS_TOPICS=$(ros2 topic list)
for topic in "/scan" "/imu" "/clock" "/joint_states"; do
    if echo "$ROS_TOPICS" | grep -q "$topic"; then
        echo -e "[${GREEN}OK${NC}] ROS 2 Topic found: $topic"
    else
        echo -e "[${RED}FAIL${NC}] ROS 2 Topic MISSING: $topic (Check bridge.yaml)"
    fi
done

# 4. Data Response Test (Checking for actual messages)
echo -e "\n${YELLOW}Testing Data Throughput (5 second sample)...${NC}"

check_hz() {
    local topic=$1
    local expected=$2
    echo -n "Testing $topic... "
    # Run for 2 seconds and grab the average Hz
    HZ=$(ros2 topic hz $topic --qos-reliability best_effort -c 5 2>/dev/null | grep "average rate" | awk '{print $4}')
    
    if [ ! -z "$HZ" ]; then
        echo -e "[${GREEN}RECEIVING @ ${HZ} Hz${NC}]"
    else
        echo -e "[${RED}NO DATA RECEIVED${NC}]"
    fi
}

check_hz "/scan" "10"
check_hz "/imu" "100"
check_hz "/joint_states" "50"

# 5. Controller Status
echo -e "\n${YELLOW}Checking ROS 2 Control Status...${NC}"
ros2 control list_controllers | grep -E "active|unconfigured|inactive" --color=always

echo -e "\n${YELLOW}Diagnostic Complete.${NC}"