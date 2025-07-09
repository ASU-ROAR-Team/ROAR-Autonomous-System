# ZED Obstacle Detector - Simple Guide

## Quick Start

### 1. Build and Source
```bash
cd ~/roar/roar_ws
catkin_make
source devel/setup.bash
```

### 2. Start the Node
```bash
# Debug mode (fine resolution, slow)
roslaunch zed_obstacle_detector zed_camera_generic.launch camera_model:=zed2i performance_mode:=debug

# Production mode (balanced)
roslaunch zed_obstacle_detector zed_camera_generic.launch camera_model:=zed2i performance_mode:=production

# High performance mode (coarse resolution, fast)
roslaunch zed_obstacle_detector zed_camera_generic.launch camera_model:=zed2i performance_mode:=high_performance
```

## Performance Modes vs Parameter Files

### Performance Modes (Launch File)
- **Automatic**: Set predefined values when you launch
- **Fixed**: Each mode has specific values (debug=0.05m, production=0.08m, high_performance=0.12m)
- **Use when**: You want quick, standard settings

### Parameter Files (YAML)
- **Custom**: Load manually with custom values
- **Flexible**: Can have any values you want
- **Use when**: You want specific tuning or real-time testing

| Mode | Voxel Size | Speed | Use Case |
|------|------------|-------|----------|
| **debug** | 0.05m | Slow | Development, testing |
| **production** | 0.08m | Medium | Normal operation |
| **high_performance** | 0.12m | Fast | Real-time, Jetson |

## Real-Time Parameter Changes

### Method 1: Interactive Script (Recommended)
```bash
# Terminal 2: Run the testing script
./test_parameters.sh
```

### Method 2: Manual Commands
```bash
# Change voxel size
rosparam set /zed_obstacle_detector/voxel_leaf_size 0.05

# Change cluster tolerance
rosparam set /zed_obstacle_detector/cluster_tolerance 0.25

# Check current value
rosparam get /zed_obstacle_detector/voxel_leaf_size
```

### Method 3: Load Custom Parameter Sets
```bash
# Load ultra-fine tuning (0.03m voxel, very strict)
rosparam load config/test_params.yaml /zed_obstacle_detector

# Load ultra-fast settings (0.15m voxel, very aggressive)
rosparam load config/high_performance_params.yaml /zed_obstacle_detector
```

## Key Parameters to Test

### Processing Parameters
- **voxel_leaf_size**: 0.03 (ultra-fine) to 0.15 (ultra-coarse)
- **cluster_tolerance**: 0.15 (very tight) to 0.6 (very loose)
- **min_cluster_size**: 10 (tiny) to 50 (large only)

### Quick Tests
```bash
# Ultra-fine resolution test
rosparam set /zed_obstacle_detector/voxel_leaf_size 0.03
rosparam set /zed_obstacle_detector/cluster_tolerance 0.15

# Ultra-coarse resolution test
rosparam set /zed_obstacle_detector/voxel_leaf_size 0.15
rosparam set /zed_obstacle_detector/cluster_tolerance 0.6
```

## Troubleshooting

### rqt_reconfigure Not Found
```bash
# Source ROS environment
source /opt/ros/noetic/setup.bash
source ~/roar/roar_ws/devel/setup.bash

# Then try
rqt_reconfigure
```

### No Obstacles Detected
- Check point cloud topic: `rostopic list | grep zed`
- Try different clustering parameters
- Check if ground filtering is too aggressive

## Project Structure
```
zed_obstacle_detector/
├── launch/
│   └── zed_camera_generic.launch    # Main launch file with performance modes
├── config/
│   ├── params.yaml                  # Default parameters
│   ├── high_performance_params.yaml # Ultra-fast parameters
│   └── test_params.yaml             # Fine-tuning parameters
├── src/                             # Source code modules
├── include/                         # Header files
└── test_parameters.sh               # Interactive parameter testing
```

## When to Use What

### Start with Performance Modes:
```bash
# Quick start with standard settings
roslaunch zed_obstacle_detector zed_camera_generic.launch performance_mode:=debug
```

### Then Use Parameter Files for Custom Tuning:
```bash
# Load ultra-fine settings for detailed testing
rosparam load config/test_params.yaml /zed_obstacle_detector

# Load ultra-fast settings for Jetson
rosparam load config/high_performance_params.yaml /zed_obstacle_detector
```

### Use Interactive Script for Real-Time Testing:
```bash
# Test different values while running
./test_parameters.sh
```

## That's It!

- **Performance modes** = Quick start with standard settings
- **Parameter files** = Custom tuning with specific values
- **Interactive script** = Real-time parameter testing
- **No rebuilding needed** = Parameters change instantly

Happy testing! 🚀 