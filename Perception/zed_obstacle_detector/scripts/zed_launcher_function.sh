# Function to launch ZED obstacle detector with configurable RQT options
# Add this to your .bashrc file
launch_zed_obstacle_detector() {
    local enable_rqt_reconfigure=${1:-false}
    local enable_full_gui=${2:-false}
    local enable_rqt_plot=${3:-false}
    local force_rviz=${4:-false}
    local performance_mode=${5:-debug}
    
    echo "Launching ZED Obstacle Detector with:"
    echo "  Performance Mode: $performance_mode"
    echo "  RQT Reconfigure: $enable_rqt_reconfigure"
    echo "  Full RQT GUI: $enable_full_gui"
    echo "  RQT Plot: $enable_rqt_plot"
    echo "  Force RViz: $force_rviz"
    echo ""
    
    # Change to the ROS workspace directory
    cd ~/roar/roar_ws
    
    # Source the workspace
    source devel/setup.bash
    
    roslaunch zed_obstacle_detector zed_camera_generic.launch \
        performance_mode:=$performance_mode \
        enable_rqt_reconfigure:=$enable_rqt_reconfigure \
        enable_full_gui:=$enable_full_gui \
        enable_rqt_plot:=$enable_rqt_plot \
        force_rviz:=$force_rviz
}

# Aliases for quick access
alias zed_debug='launch_zed_obstacle_detector true true false false debug'
alias zed_production='launch_zed_obstacle_detector false false false false production'
alias zed_reconfigure='launch_zed_obstacle_detector true false false false debug'
alias zed_everything='launch_zed_obstacle_detector true true true true debug' 