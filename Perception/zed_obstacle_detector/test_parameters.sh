#!/bin/bash

# ZED Obstacle Detector Parameter Testing Script
# This script demonstrates real-time parameter changes using rosparam

echo "=== ZED Obstacle Detector Parameter Testing ==="
echo "This script will help you test different parameter values in real-time"
echo ""

# Source ROS environment
source /opt/ros/noetic/setup.bash
source ~/roar/roar_ws/devel/setup.bash

# Check if ROS master is running
if ! rostopic list > /dev/null 2>&1; then
    echo "ERROR: ROS master is not running. Please start roscore first:"
    echo "  roscore"
    echo ""
    exit 1
fi

echo "✅ ROS master is running"
echo ""

# Function to set parameters
set_params() {
    echo "Setting parameters: $1"
    rosparam set /zed_obstacle_detector/voxel_leaf_size $2
    rosparam set /zed_obstacle_detector/cluster_tolerance $3
    rosparam set /zed_obstacle_detector/min_cluster_size $4
    echo "Parameters updated! Check your obstacle detector output."
    echo ""
}

# Function to show current parameters
show_params() {
    echo "Current parameters:"
    echo "  Voxel Leaf Size: $(rosparam get /zed_obstacle_detector/voxel_leaf_size)"
    echo "  Cluster Tolerance: $(rosparam get /zed_obstacle_detector/cluster_tolerance)"
    echo "  Min Cluster Size: $(rosparam get /zed_obstacle_detector/min_cluster_size)"
    echo ""
}

# Function to load parameter set
load_param_set() {
    echo "Loading parameter set: $1"
    rosparam load config/$1.yaml /zed_obstacle_detector
    echo "Parameter set loaded! Check your obstacle detector output."
    echo ""
}

echo "Available commands:"
echo "  1. show_params          - Show current parameter values"
echo "  2. set_fine             - Set fine resolution (0.05m voxel, 0.25m cluster)"
echo "  3. set_medium           - Set medium resolution (0.08m voxel, 0.3m cluster)"
echo "  4. set_coarse           - Set coarse resolution (0.12m voxel, 0.4m cluster)"
echo "  5. load_test            - Load test parameter set"
echo "  6. load_high_perf       - Load high performance parameter set"
echo "  7. custom               - Set custom parameters"
echo "  8. exit                 - Exit the script"
echo ""

while true; do
    read -p "Enter command (1-8): " choice
    
    case $choice in
        1)
            show_params
            ;;
        2)
            set_params "Fine Resolution" 0.05 0.25 15
            ;;
        3)
            set_params "Medium Resolution" 0.08 0.3 20
            ;;
        4)
            set_params "Coarse Resolution" 0.12 0.4 25
            ;;
        5)
            load_param_set "test_params.yaml"
            ;;
        6)
            load_param_set "high_performance_params.yaml"
            ;;
        7)
            echo "Enter custom parameters:"
            read -p "Voxel Leaf Size (0.02-0.20): " voxel
            read -p "Cluster Tolerance (0.1-1.0): " cluster
            read -p "Min Cluster Size (5-100): " min_size
            set_params "Custom" $voxel $cluster $min_size
            ;;
        8)
            echo "Exiting..."
            exit 0
            ;;
        *)
            echo "Invalid choice. Please enter 1-8."
            ;;
    esac
done 