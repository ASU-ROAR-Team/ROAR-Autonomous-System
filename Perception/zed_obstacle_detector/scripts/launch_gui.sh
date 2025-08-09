#!/bin/bash

# ZED Obstacle Detector RQT GUI Launcher
# This script launches the obstacle detector with RQT GUI for live parameter tuning

echo "=== ZED Obstacle Detector RQT GUI Launcher ==="
echo "This will launch the obstacle detector with live parameter tuning GUI"
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

# Function to show usage
show_usage() {
    echo "Usage: $0 [OPTIONS]"
    echo ""
    echo "Options:"
    echo "  -c, --camera MODEL     Camera model (zed, zed2, zed2i, zed_mini) [default: zed2i]"
    echo "  -m, --mode MODE        Performance mode (debug, production, high_performance) [default: debug]"
    echo "  -g, --gui TYPE         GUI type (reconfigure, plot, full) [default: reconfigure]"
    echo "  -r, --rviz             Enable RViz visualization (even in non-debug modes)"
    echo "  -h, --help             Show this help message"
    echo ""
    echo "Examples:"
    echo "  $0                                    # Default: zed2i, debug mode, reconfigure GUI"
    echo "  $0 -c zed2i -m debug -g full -r       # Full GUI with RViz"
    echo "  $0 -c zed2i -m production -g reconfigure # Production mode with reconfigure GUI"
    echo "  $0 -g plot                            # Just performance plotting"
    echo ""
}

# Default values
CAMERA_MODEL="zed2i"
PERFORMANCE_MODE="debug"
GUI_TYPE="reconfigure"
ENABLE_RVIZ=false

# Parse command line arguments
while [[ $# -gt 0 ]]; do
    case $1 in
        -c|--camera)
            CAMERA_MODEL="$2"
            shift 2
            ;;
        -m|--mode)
            PERFORMANCE_MODE="$2"
            shift 2
            ;;
        -g|--gui)
            GUI_TYPE="$2"
            shift 2
            ;;
        -r|--rviz)
            ENABLE_RVIZ=true
            shift
            ;;
        -h|--help)
            show_usage
            exit 0
            ;;
        *)
            echo "Unknown option: $1"
            show_usage
            exit 1
            ;;
    esac
done

echo "Configuration:"
echo "  Camera Model: $CAMERA_MODEL"
echo "  Performance Mode: $PERFORMANCE_MODE"
echo "  GUI Type: $GUI_TYPE"
echo "  RViz: $ENABLE_RVIZ"
echo ""

# Build launch arguments based on GUI type
LAUNCH_ARGS="camera_model:=$CAMERA_MODEL performance_mode:=$PERFORMANCE_MODE"

case $GUI_TYPE in
    "reconfigure")
        LAUNCH_ARGS="$LAUNCH_ARGS enable_rqt_reconfigure:=true"
        GUI_DESCRIPTION="RQT Reconfigure for live parameter tuning"
        ;;
        
    "plot")
        LAUNCH_ARGS="$LAUNCH_ARGS enable_rqt_plot:=true"
        GUI_DESCRIPTION="RQT Plot for performance monitoring"
        ;;
        
    "full")
        LAUNCH_ARGS="$LAUNCH_ARGS enable_full_gui:=true"
        GUI_DESCRIPTION="Full RQT GUI with multiple plugins"
        ;;
        
    *)
        echo "Unknown GUI type: $GUI_TYPE"
        echo "Valid options: reconfigure, plot, full"
        exit 1
        ;;
esac

# Add RViz if requested
if [ "$ENABLE_RVIZ" = true ]; then
    LAUNCH_ARGS="$LAUNCH_ARGS force_rviz:=true"
fi

# Launch the obstacle detector with GUI
echo "Launching ZED obstacle detector with $GUI_DESCRIPTION..."
echo "Launch command: roslaunch zed_obstacle_detector zed_camera_generic.launch $LAUNCH_ARGS"
echo ""

roslaunch zed_obstacle_detector zed_camera_generic.launch $LAUNCH_ARGS

echo ""
echo "🎉 ZED Obstacle Detector with RQT GUI launched successfully!"
echo ""
echo "📋 Available tools:"
echo "  • RQT Reconfigure: Live parameter tuning"
echo "  • RViz: 3D visualization of obstacles"
echo "  • RQT Plot: Performance monitoring"
echo ""
echo "🔧 Quick parameter changes:"
echo "  • Open RQT Reconfigure to adjust parameters in real-time"
echo "  • Use the test_parameters.sh script for quick parameter sets"
echo "  • Monitor performance with RQT Plot"
echo ""
echo "Press Ctrl+C to stop all processes"
echo "" 