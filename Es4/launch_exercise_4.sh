#!/bin/bash

# Exit immediately if a command exits with a non-zero status.
set -e

# PWD stores the absolute path where the script is executed (workspace root assumed)
WORKSPACE_PATH=$(pwd)
DELAY_SETUP=14 # Delay for RViz/Simulation setup
DELAY_NODES=6  # Delay between Server and Client

# --- Run colcon build ---
colcon build

# --- Source setup files in main shell ---
if [ -f install/setup.bash ]; then
    source install/setup.bash
elif [ -f install/setup.zsh ]; then
    source install/setup.zsh
else
    exit 1
fi

# Determine ROS Distro (e.g., humble) from environment variable
if [ -z "$ROS_DISTRO" ]; then
    exit 1
fi

# Define the sourcing command for new terminals
# It sources the default ROS installation and then the local workspace install
SOURCE_COMMAND="source /opt/ros/$ROS_DISTRO/setup.bash && source $WORKSPACE_PATH/install/setup.bash"

# --- Launch Simulation (RViz) in a new terminal and wait 15s ---
LAUNCH_CMD="$SOURCE_COMMAND && ros2 launch ir_launch exercise_4.launch.py"
# Aggiunto 'exec /bin/bash' per mantenere aperto il terminale
gnome-terminal --title="EX4 - 1/3 Simulation (RViz)" -- /bin/bash -c "$LAUNCH_CMD; exec /bin/bash"

# Wait 10 seconds for RViz/simulation to fully initialize
sleep $DELAY_SETUP

# ---  Launch Server in a new terminal ---
SERVER_CMD="$SOURCE_COMMAND && ros2 run twenty_four_ex4 robot_server"
gnome-terminal --title="EX4 - 2/3 Robot Server" -- /bin/bash -c "$SERVER_CMD; exec /bin/bash"

# Wait 6 seconds before launching the client
sleep $DELAY_NODES

# --- Launch Client in a new terminal ---
CLIENT_CMD="$SOURCE_COMMAND && ros2 run twenty_four_ex4 burrow_client"
gnome-terminal --title="EX4 - 3/3 Burrow Client" -- /bin/bash -c "$CLIENT_CMD; exec /bin/bash"

