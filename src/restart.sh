#!/bin/bash

# First, create a temporary restart script
cat > /tmp/ros2_restart_exec.sh << 'EOF'
#!/bin/bash
# Wait for the original processes to fully terminate
sleep 5

# Change to project directory
cd /home/skolek/robot || exit 1

# Source the setup file
source install/setup.bash

# Launch the robot
ros2 launch robot_bringup robot_launch.py
EOF

# Make it executable
chmod +x /tmp/ros2_restart_exec.sh

# Kill all ROS2 instances
sudo pkill -f "/home/skolek/robot/"

# Schedule the restart script to run in 5 seconds using at command
echo "/tmp/ros2_restart_exec.sh" | at now + 5 seconds
