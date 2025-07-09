# ZED2i Real-World Deployment Guide

This guide covers deploying the refactored modular obstacle detector on ZED2i camera with NVIDIA Jetson for real-world testing.

## Prerequisites

### Hardware Requirements
- ZED2i stereo camera
- NVIDIA Jetson (Xavier NX, AGX Xavier, or similar)
- ROS Noetic installed
- ZED SDK installed

### Software Dependencies
- ROS Noetic
- PCL (Point Cloud Library)
- ZED ROS Wrapper
- TF2 (Transform library)
- Visualization tools (RViz for debug)

## Installation Steps

### 1. Build the Package
```bash
cd ~/roar/roar_ws
catkin build zed_obstacle_detector
source devel/setup.bash
```

### 2. Verify ZED2i Setup
```bash
# Check if ZED2i is detected
lsusb | grep ZED

# Verify ZED ROS wrapper is working
roslaunch zed_wrapper zed2i.launch
# In another terminal:
rostopic list | grep zed
```

## Launch Options

### Generic Launch File (Recommended)
```bash
# ZED2i Production Mode
roslaunch zed_obstacle_detector zed_camera_generic.launch camera_model:=zed2i performance_mode:=production

# ZED2i Debug Mode
roslaunch zed_obstacle_detector zed_camera_generic.launch camera_model:=zed2i performance_mode:=debug

# ZED2i High Performance Mode
roslaunch zed_obstacle_detector zed_camera_generic.launch camera_model:=zed2i performance_mode:=high_performance

# Other ZED Models
roslaunch zed_obstacle_detector zed_camera_generic.launch camera_model:=zed2 performance_mode:=production
roslaunch zed_obstacle_detector zed_camera_generic.launch camera_model:=zed_mini performance_mode:=debug
```

### Performance Modes

#### Debug Mode
- **Voxel Leaf Size**: 0.05m (fine resolution)
- **Debug Publishers**: Enabled
- **Detailed Timing**: Enabled
- **RViz**: Enabled
- **Early Exit**: Disabled
- **Best for**: Development, testing, debugging

#### Production Mode
- **Voxel Leaf Size**: 0.08m (balanced)
- **Debug Publishers**: Disabled
- **Detailed Timing**: Disabled
- **RViz**: Disabled
- **Early Exit**: Enabled
- **Best for**: Real-world deployment, balanced performance

#### High Performance Mode
- **Voxel Leaf Size**: 0.12m (coarse resolution)
- **Debug Publishers**: Disabled
- **Detailed Timing**: Disabled
- **RViz**: Disabled
- **Early Exit**: Enabled
- **Best for**: Resource-constrained systems, maximum performance

## Key Optimizations for Jetson

### Performance Settings
- **Voxel Leaf Size**: 0.08m (increased from 0.05m)
- **Early Exit**: Enabled for small/large point clouds
- **Debug Publishers**: Disabled in production
- **TF Timeout**: Reduced to 0.05s
- **Ground Filter Iterations**: Increased to 200

### Memory Management
- **Max Points**: 15,000 (reduced from 20,000)
- **TF Buffer Duration**: 5.0s (reduced from 10.0s)
- **Timing Reports**: Every 30s (reduced frequency)

## Topic Configuration

### Input Topics
- **Point Cloud**: `/zed2i/zed_node/point_cloud/cloud_registered`

### Output Topics
- **Obstacle Array**: `/zed_obstacle/obstacle_array`
- **Visualization Markers**: `/zed_obstacle/markers`
- **Debug Point Clouds** (debug mode only):
  - `/zed_obstacle/debug/filtered_transformed_pc`
  - `/zed_obstacle/debug/pc_no_ground`
  - `/zed_obstacle/debug/raw_clusters_rgb`

## Frame Configuration

### Required TF Frames
- **Source**: `zed2i_left_camera_frame` (or other ZED camera frame)
- **Target**: `base_link`
- **World**: `world`

### Camera Frame Parameterization
The camera frame is now fully parameterized and can be configured for different ZED models:

#### Supported Camera Models
- **ZED**: `zed_left_camera_frame`
- **ZED2**: `zed2_left_camera_frame`
- **ZED2i**: `zed2i_left_camera_frame`
- **ZED Mini**: `zed_mini_left_camera_frame`

#### Custom Camera Frame
```bash
# Use custom camera frame
roslaunch zed_obstacle_detector zed_camera_generic.launch camera_frame:=my_custom_camera_frame

# Override camera model with custom frame
roslaunch zed_obstacle_detector zed_camera_generic.launch camera_model:=zed2i camera_frame:=zed2i_right_camera_frame
```

### TF Tree Setup
Ensure your robot's TF tree includes:
```
world
└── base_link
    └── [camera_frame] (e.g., zed2i_left_camera_frame)
```

## Parameter Tuning for Real-World

### Environment-Specific Adjustments

#### Indoor Environments
```yaml
voxel_leaf_size: 0.05
cluster_tolerance: 0.2
ground_filter_distance_threshold: 0.05
```

#### Outdoor Environments
```yaml
voxel_leaf_size: 0.1
cluster_tolerance: 0.4
ground_filter_distance_threshold: 0.1
ground_filter_angle_threshold_deg: 20.0
```

#### High-Performance Mode (Jetson)
```yaml
voxel_leaf_size: 0.12
enable_early_exit: true
enable_debug_publishers: false
enable_detailed_timing: false
```

## Monitoring and Debugging

### Performance Monitoring
- Check `/tmp/zed_obstacle_detector.log` for performance metrics
- Monitor CPU and GPU usage on Jetson
- Watch for TF timeout warnings

### Common Issues

#### TF Errors
```
[ERROR] TF transform failed for new cluster
```
**Solution**: Verify TF tree is properly configured and published

#### Performance Issues
```
[WARN] Processing time too high
```
**Solution**: Increase voxel_leaf_size or enable early_exit

#### No Obstacles Detected
```
[INFO] No raw clusters found
```
**Solution**: Check point cloud topic, adjust clustering parameters

### Debug Tools

#### RViz Configuration
Load the provided RViz config:
```bash
rviz -d $(find zed_obstacle_detector)/config/zed_obstacle_detector.rviz
```

#### Topic Monitoring
```bash
# Monitor obstacle detections
rostopic echo /zed_obstacle/obstacle_array

# Monitor point cloud processing
rostopic echo /zed_obstacle/debug/filtered_transformed_pc
```

## Testing Checklist

### Pre-Deployment
- [ ] ZED2i camera detected and calibrated
- [ ] TF tree properly configured
- [ ] Point cloud topic publishing
- [ ] Package builds successfully
- [ ] Launch files work without errors

### Runtime Verification
- [ ] Obstacle detection working
- [ ] Performance within acceptable limits
- [ ] No TF timeout errors
- [ ] Memory usage stable
- [ ] CPU usage reasonable

### Performance Targets
- **Processing Time**: < 100ms per frame
- **Memory Usage**: < 2GB
- **CPU Usage**: < 80% on Jetson
- **Detection Range**: 0.2m - 7.0m

## Troubleshooting

### Build Issues
```bash
# Clean and rebuild
catkin clean zed_obstacle_detector
catkin build zed_obstacle_detector
```

### Runtime Issues
```bash
# Check ROS master
roscore

# Verify topics
rostopic list | grep zed

# Check TF
rosrun tf tf_echo zed2i_left_camera_frame base_link
```

### Performance Issues
- Reduce voxel_leaf_size if too coarse
- Increase voxel_leaf_size if too slow
- Enable early_exit for performance
- Disable debug publishers in production

## Support

For issues specific to the modular architecture:
- Check the performance logs in `/tmp/`
- Verify all modular components are initialized
- Ensure parameter configuration is correct
- Monitor individual component performance

For ZED2i-specific issues:
- Refer to ZED SDK documentation
- Check ZED ROS wrapper configuration
- Verify camera calibration 