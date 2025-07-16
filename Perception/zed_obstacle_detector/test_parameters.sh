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

# Function to set a single parameter
set_param() {
    local param_name=$1
    local value=$2
    echo "Setting $param_name = $value"
    rosparam set /zed_obstacle_detector/$param_name $value
}

# Function to get a parameter value
get_param() {
    local param_name=$1
    rosparam get /zed_obstacle_detector/$param_name 2>/dev/null || echo "not set"
}

# Function to increment/decrement a parameter
adjust_param() {
    local param_name=$1
    local current_value=$(get_param $param_name)
    local operation=$2
    local step=$3
    
    if [[ "$current_value" == "not set" ]]; then
        echo "Parameter $param_name is not set. Cannot adjust."
        return
    fi
    
    local new_value
    case $operation in
        "increment")
            new_value=$(echo "$current_value + $step" | bc -l)
            ;;
        "decrement")
            new_value=$(echo "$current_value - $step" | bc -l)
            ;;
    esac
    
    echo "Adjusting $param_name: $current_value → $new_value"
    set_param $param_name $new_value
}

# Function to show all current parameters
show_all_params() {
    echo "=== Current Parameter Values ==="
    echo "Processing Parameters:"
    echo "  Voxel Leaf Size: $(get_param voxel_leaf_size) m"
    echo "  Enable Uniform Downsampling: $(get_param enable_uniform_downsampling)"
    echo "  Uniform Sampling Radius: $(get_param uniform_sampling_radius) m"
    echo "  Enable Early Exit: $(get_param enable_early_exit)"
    echo "  Min Points for Processing: $(get_param min_points_for_processing)"
    echo "  Max Points for Processing: $(get_param max_points_for_processing)"
    echo "  Z Range Min: $(get_param passthrough_z_min_camera) m"
    echo "  Z Range Max: $(get_param passthrough_z_max_camera) m"
    echo ""
    echo "Ground Detection:"
    echo "  Enable Ground Filtering: $(get_param enable_ground_filtering)"
    echo "  Ground Distance Threshold: $(get_param ground_filter_distance_threshold) m"
    echo "  Ground Angle Threshold: $(get_param ground_filter_angle_threshold_deg) deg"
    echo "  Ground Max Iterations: $(get_param ground_filter_max_iterations)"
    echo "  Mars Terrain Mode: $(get_param mars_terrain_mode)"
    echo ""
    echo "Clustering:"
    echo "  Cluster Tolerance: $(get_param cluster_tolerance) m"
    echo "  Min Cluster Size: $(get_param min_cluster_size)"
    echo "  Max Cluster Size: $(get_param max_cluster_size)"
    echo ""
    echo "Tracking:"
    echo "  Association Distance: $(get_param obstacle_association_distance) m"
    echo "  Timeout: $(get_param obstacle_timeout_sec) sec"
    echo "  Position Smoothing: $(get_param position_smoothing_factor)"
    echo "  Min Detections: $(get_param min_detections_for_confirmation)"
    echo ""
    echo "Performance:"
    echo "  Enable Detailed Timing: $(get_param enable_detailed_timing)"
    echo "  Enable Debug Publishers: $(get_param enable_debug_publishers)"
    echo "  Timing Report Interval: $(get_param timing_report_interval) sec"
    echo "  Enable Performance Logging: $(get_param enable_performance_logging)"
    echo "=========================================="
    echo ""
}

# Function to set clustering parameters
set_clustering_params() {
    echo "Setting clustering parameters: $1"
    set_param cluster_tolerance $2
    set_param min_cluster_size $3
    set_param max_cluster_size $4
    echo "Clustering parameters updated!"
    echo ""
}

# Function to set processing parameters
set_processing_params() {
    echo "Setting processing parameters: $1"
    set_param voxel_leaf_size $2
    set_param enable_uniform_downsampling $3
    set_param uniform_sampling_radius $4
    set_param enable_early_exit $5
    echo "Processing parameters updated!"
    echo ""
}

# Function to set ground detection parameters
set_ground_params() {
    echo "Setting ground detection parameters: $1"
    set_param enable_ground_filtering $2
    set_param ground_filter_distance_threshold $3
    set_param ground_filter_angle_threshold_deg $4
    set_param ground_filter_max_iterations $5
    echo "Ground detection parameters updated!"
    echo ""
}

# Function to load parameter set
load_param_set() {
    echo "Loading parameter set: $1"
    rosparam load config/$1.yaml /zed_obstacle_detector
    echo "Parameter set loaded!"
    echo ""
}

# Function to adjust individual parameter
adjust_individual_param() {
    echo "=== Individual Parameter Adjustment ==="
    echo "Available parameters:"
    echo "  1. voxel_leaf_size"
    echo "  2. cluster_tolerance"
    echo "  3. min_cluster_size"
    echo "  4. max_cluster_size"
    echo "  5. ground_filter_distance_threshold"
    echo "  6. ground_filter_angle_threshold_deg"
    echo "  7. obstacle_association_distance"
    echo "  8. obstacle_timeout_sec"
    echo "  9. position_smoothing_factor"
    echo "  10. uniform_sampling_radius"
    echo "  11. passthrough_z_min_camera"
    echo "  12. passthrough_z_max_camera"
    echo "  13. min_points_for_processing"
    echo "  14. max_points_for_processing"
    echo "  15. min_detections_for_confirmation"
    echo "  16. timing_report_interval"
    echo ""
    
    read -p "Select parameter (1-16): " param_choice
    
    case $param_choice in
        1) param_name="voxel_leaf_size"; step=0.01; ;;
        2) param_name="cluster_tolerance"; step=0.02; ;;
        3) param_name="min_cluster_size"; step=1; ;;
        4) param_name="max_cluster_size"; step=100; ;;
        5) param_name="ground_filter_distance_threshold"; step=0.01; ;;
        6) param_name="ground_filter_angle_threshold_deg"; step=1; ;;
        7) param_name="obstacle_association_distance"; step=0.1; ;;
        8) param_name="obstacle_timeout_sec"; step=1; ;;
        9) param_name="position_smoothing_factor"; step=0.05; ;;
        10) param_name="uniform_sampling_radius"; step=0.01; ;;
        11) param_name="passthrough_z_min_camera"; step=0.1; ;;
        12) param_name="passthrough_z_max_camera"; step=0.1; ;;
        13) param_name="min_points_for_processing"; step=10; ;;
        14) param_name="max_points_for_processing"; step=100; ;;
        15) param_name="min_detections_for_confirmation"; step=1; ;;
        16) param_name="timing_report_interval"; step=5; ;;
        *) echo "Invalid choice"; return; ;;
    esac
    
    echo "Current value: $(get_param $param_name)"
    echo "Step size: $step"
    echo ""
    echo "1. Increment (+$step)"
    echo "2. Decrement (-$step)"
    echo "3. Set custom value"
    echo "4. Cancel"
    echo ""
    
    read -p "Choose action (1-4): " action
    
    case $action in
        1)
            adjust_param $param_name "increment" $step
            ;;
        2)
            adjust_param $param_name "decrement" $step
            ;;
        3)
            read -p "Enter new value: " new_value
            set_param $param_name $new_value
            ;;
        4)
            echo "Cancelled."
            ;;
        *)
            echo "Invalid choice"
            ;;
    esac
    echo ""
}

echo "Available commands:"
echo "  1. show_all_params       - Show all current parameter values"
echo "  2. adjust_individual     - Adjust individual parameter (increment/decrement)"
echo "  3. set_clustering_fine   - Set fine clustering (0.08m tolerance)"
echo "  4. set_clustering_medium - Set medium clustering (0.12m tolerance)"
echo "  5. set_clustering_coarse - Set coarse clustering (0.18m tolerance)"
echo "  6. set_processing_fine   - Set fine processing (0.05m voxel)"
echo "  7. set_processing_medium - Set medium processing (0.08m voxel)"
echo "  8. set_processing_coarse - Set coarse processing (0.12m voxel)"
echo "  9. set_ground_aggressive - Set aggressive ground filtering"
echo "  10. set_ground_conservative - Set conservative ground filtering"
echo "  11. toggle_debug_output  - Toggle debug output on/off"
echo "  12. toggle_ground_filter - Toggle ground filtering on/off"
echo "  13. load_test            - Load test parameter set"
echo "  14. load_high_perf       - Load high performance parameter set"
echo "  15. exit                 - Exit the script"
echo ""

while true; do
    read -p "Enter command (1-15): " choice
    
    case $choice in
        1)
            show_all_params
            ;;
        2)
            adjust_individual_param
            ;;
        3)
            set_clustering_params "Fine" 0.08 15 2000
            ;;
        4)
            set_clustering_params "Medium" 0.12 20 3000
            ;;
        5)
            set_clustering_params "Coarse" 0.18 25 5000
            ;;
        6)
            set_processing_params "Fine" 0.05 true 0.05 true
            ;;
        7)
            set_processing_params "Medium" 0.08 false 0.08 true
            ;;
        8)
            set_processing_params "Coarse" 0.12 false 0.12 true
            ;;
        9)
            set_ground_params "Aggressive" true 0.05 10.0 150
            ;;
        10)
            set_ground_params "Conservative" true 0.15 25.0 300
            ;;
        11)
            current=$(get_param enable_debug_output)
            if [[ "$current" == "true" ]]; then
                set_param enable_debug_output false
                echo "Debug output disabled"
            else
                set_param enable_debug_output true
                echo "Debug output enabled"
            fi
            ;;
        12)
            current=$(get_param enable_ground_filtering)
            if [[ "$current" == "true" ]]; then
                set_param enable_ground_filtering false
                echo "Ground filtering disabled"
            else
                set_param enable_ground_filtering true
                echo "Ground filtering enabled"
            fi
            ;;
        13)
            load_param_set "test_params.yaml"
            ;;
        14)
            load_param_set "high_performance_params.yaml"
            ;;
        15)
            echo "Exiting..."
            exit 0
            ;;
        *)
            echo "Invalid choice. Please enter 1-15."
            ;;
    esac
done 