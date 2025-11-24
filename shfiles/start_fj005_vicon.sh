#!/bin/bash

# Create new tmux session
tmux new-session -d -s mysession -n ros_session

# Kill old ROS processes
tmux send-keys -t mysession 'killall -9 rosmaster roscore rosout' C-m
sleep 1

# ---------------------------
# Create the grid layout first
# ---------------------------

# Create initial pane (will become top-left)
# tmux send-keys -t mysession:0.0 'source ../../devel/setup.bash' C-m
tmux send-keys -t mysession:0.0 'cd ../../' C-m
tmux send-keys -t mysession:0.0 'source ./devel/setup.bash' C-m
tmux send-keys -t mysession:0.0 'roscore' C-m
sleep 2

# Split horizontally to create top row (3 columns)
tmux split-window -h -t mysession:0.0  # Creates pane 0.1 (top-middle)
tmux split-window -h -t mysession:0.1  # Creates pane 0.2 (top-right)

# Now create bottom row by splitting each top pane vertically
tmux split-window -v -t mysession:0.0  # Creates pane 0.3 (bottom-left)
tmux split-window -v -t mysession:0.1  # Creates pane 0.4 (bottom-middle)
tmux split-window -v -t mysession:0.2  # Creates pane 0.5 (bottom-right)

# ---------------------------
# Configure the top row
# ---------------------------

# Top-left (roscore) is already configured

# Top-middle (px4)
tmux send-keys -t mysession:0.1 'cd ../../' C-m
tmux send-keys -t mysession:0.1 'source ./devel/setup.bash' C-m
tmux send-keys -t mysession:0.1 'roslaunch px4ctrl px4_vicon.launch' C-m
sleep 0.5

# Top-right (vrpn)
tmux send-keys -t mysession:0.2 'cd ../../' C-m
tmux send-keys -t mysession:0.2 'source ./devel/setup.bash' C-m
# tmux send-keys -t mysession:0.2 'source ../devel/setup.bash' C-m
tmux send-keys -t mysession:0.2 'roslaunch px4ctrl vrpn.launch' C-m
sleep 0.5

# ---------------------------
# Configure the bottom row (manual panes)
# ---------------------------
for pane in 3 4 5; do
    tmux send-keys -t mysession:0.${pane} 'source ../devel/setup.bash' C-m
    tmux send-keys -t mysession:0.${pane} "echo \"Manual pane ${pane}\"" C-m
done
# tmux send-keys -t mysession:0.3 'rostopic echo /mavros/vision_pose/pose' C-m
tmux send-keys -t mysession:0.3 'cd ../../' C-m
tmux send-keys -t mysession:0.3 'source ./devel/setup.bash' C-m
tmux send-keys -t mysession:0.3 'rosrun px4ctrl pose2odom.py' C-m

# tmux send-keys -t mysession:0.4 'rostopic echo /mavros/setpoint_raw/local' C-m
# tmux send-keys -t mysession:0.4 'cd ../../' C-m
tmux send-keys -t mysession:0.4 'source ../../devel/setup.bash' C-m

# ---------------------------
# Final layout adjustments
# ---------------------------

# Set equal pane sizes
tmux select-layout -t mysession tiled

# Focus on first manual pane
tmux select-pane -t mysession:0.5
tmux send-keys -t mysession:0.5 'cd ../../' C-m
tmux send-keys -t mysession:0.5 'source ./devel/setup.bash' C-m

# Attach to session
tmux attach-session -t mysession