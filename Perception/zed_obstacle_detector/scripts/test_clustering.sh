#!/bin/bash

# ZED Obstacle Detector Clustering Parameter Testing Script
# This script helps find optimal clustering parameters for your environment

echo "=== ZED Obstacle Detector Clustering Parameter Testing ==="
echo "This script will help you find the best clustering parameters"
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

# Function to set clustering parameters
set_clustering_params() {
    echo "Setting clustering parameters: $1"
    echo "  Cluster Tolerance: $2 m"
    echo "  Min Cluster Size: $3"
    echo "  Max Cluster Size: $4"
    echo ""
    
    rosparam set /zed_obstacle_detector/cluster_tolerance $2
    rosparam set /zed_obstacle_detector/min_cluster_size $3
    rosparam set /zed_obstacle_detector/max_cluster_size $4
    
    echo "Parameters updated! Check your obstacle detector output in RViz."
    echo "Look for:"
    echo "  - Individual obstacles properly separated"
    echo "  - No massive single clusters"
    echo "  - Reasonable number of detected obstacles"
    echo ""
}

# Function to show current clustering parameters
show_clustering_params() {
    echo "Current clustering parameters:"
    echo "  Cluster Tolerance: $(rosparam get /zed_obstacle_detector/cluster_tolerance) m"
    echo "  Min Cluster Size: $(rosparam get /zed_obstacle_detector/min_cluster_size)"
    echo "  Max Cluster Size: $(rosparam get /zed_obstacle_detector/max_cluster_size)"
    echo ""
}

echo "Available clustering parameter sets:"
echo "  1. show_params          - Show current clustering parameters"
echo "  2. very_fine            - Very fine clustering (0.05m tolerance, good for small objects)"
echo "  3. fine                 - Fine clustering (0.08m tolerance, good for medium objects)"
echo "  4. medium               - Medium clustering (0.12m tolerance, balanced)"
echo "  5. coarse               - Coarse clustering (0.18m tolerance, for large objects)"
echo "  6. very_coarse          - Very coarse clustering (0.25m tolerance, current default)"
echo "  7. custom               - Set custom clustering parameters"
echo "  8. test_sequence        - Run through a sequence of parameters automatically"
echo "  9. exit                 - Exit the script"
echo ""

while true; do
    read -p "Enter command (1-9): " choice
    
    case $choice in
        1)
            show_clustering_params
            ;;
        2)
            set_clustering_params "Very Fine" 0.05 10 1000
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
            set_clustering_params "Very Coarse" 0.25 30 8000
            ;;
        7)
            echo "Enter custom clustering parameters:"
            read -p "Cluster Tolerance (0.02-0.5 m): " tolerance
            read -p "Min Cluster Size (5-100): " min_size
            read -p "Max Cluster Size (100-10000): " max_size
            set_clustering_params "Custom" $tolerance $min_size $max_size
            ;;
        8)
            echo "Running automatic parameter sequence..."
            echo "This will cycle through different parameters every 10 seconds"
            echo "Press Ctrl+C to stop"
            echo ""
            
            params=(
                "0.05 10 1000"   # Very fine
                "0.08 15 2000"   # Fine  
                "0.12 20 3000"   # Medium
                "0.18 25 5000"   # Coarse
                "0.25 30 8000"   # Very coarse
            )
            
            names=("Very Fine" "Fine" "Medium" "Coarse" "Very Coarse")
            
            for i in "${!params[@]}"; do
                echo "Testing: ${names[$i]} clustering"
                set_clustering_params "${names[$i]}" ${params[$i]}
                sleep 10
            done
            ;;
        9)
            echo "Exiting..."
            exit 0
            ;;
        *)
            echo "Invalid choice. Please enter 1-9."
            ;;
    esac
done 