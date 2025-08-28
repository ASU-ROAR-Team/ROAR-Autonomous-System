# ZED Obstacle Detector - Quick Reference

## Quick Start Commands

### Generic Launch (Recommended)
```bash
# ZED2i Production Mode
roslaunch zed_obstacle_detector zed_camera_generic.launch camera_model:=zed2i performance_mode:=production

# ZED2i Debug Mode
roslaunch zed_obstacle_detector zed_camera_generic.launch camera_model:=zed2i performance_mode:=debug

# ZED2i High Performance Mode
roslaunch zed_obstacle_detector zed_camera_generic.launch camera_model:=zed2i performance_mode:=high_performance
```

## Camera Models

| Camera Model | Default Frame | Topic Pattern |
|--------------|---------------|---------------|
| `zed` | `zed_left_camera_frame` | `/zed/zed_node/point_cloud/cloud_registered` |
| `zed2` | `zed2_left_camera_frame` | `/zed2/zed_node/point_cloud/cloud_registered` |
| `zed2i` | `zed2i_left_camera_frame` | `/zed2i/zed_node/point_cloud/cloud_registered` |
| `zed_mini` | `zed_mini_left_camera_frame` | `/zed_mini/zed_node/point_cloud/cloud_registered` |

## Performance Modes

| Mode | Voxel Size | Debug | Timing | RViz | Early Exit | Use Case |
|------|------------|-------|--------|------|------------|----------|
| `debug` | 0.05m | ✅ | ✅ | ✅ | ❌ | Development |
| `production` | 0.08m | ❌ | ❌ | ❌ | ✅ | Real-world |
| `high_performance` | 0.12m | ❌ | ❌ | ❌ | ✅ | Resource-constrained |

## Key Parameters

### Camera Configuration
- `camera_model`: ZED camera model (zed, zed2, zed2i, zed_mini)
- `camera_frame`: Custom camera frame (overrides camera_model)
- `point_cloud_topic`: Custom point cloud topic

### Performance Configuration
- `performance_mode`: debug, production, high_performance
- `voxel_leaf_size`: Point cloud downsampling (0.05-0.12m)
- `enable_early_exit`: Skip processing for small/large clouds

### Frame Configuration
- `base_link_frame`: Robot's main frame (default: base_link)
- `world_frame`: Global tracking frame (default: world)

## Real-Time Parameter Changes

### Method 1: Interactive Script
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

## Common Use Cases

### Development & Testing
```bash
# Debug mode with fine resolution
roslaunch zed_obstacle_detector zed_camera_generic.launch camera_model:=zed2i performance_mode:=debug
```

### Real-World Deployment
```bash
# Production mode with balanced performance
roslaunch zed_obstacle_detector zed_camera_generic.launch camera_model:=zed2i performance_mode:=production
```

### Resource-Constrained Systems
```bash
# High performance mode for Jetson or low-power systems
roslaunch zed_obstacle_detector zed_camera_generic.launch camera_model:=zed2i performance_mode:=high_performance
```

### Custom Camera Frame
```bash
# Use right camera instead of left
roslaunch zed_obstacle_detector zed_camera_generic.launch camera_model:=zed2i camera_frame:=zed2i_right_camera_frame performance_mode:=production
```

## Troubleshooting

### Check Node Status
```bash
# List all parameters
rosparam list /zed_obstacle_detector

# Check node info
rosnode info /zed_obstacle_detector

# Check topics
rostopic list | grep zed_obstacle
```

### TF Issues
```bash
# Check TF tree
rosrun tf view_frames

# Check specific transform
rosrun tf tf_echo zed2i_left_camera_frame base_link
```

### Performance Issues
```bash
# Monitor CPU usage
htop

# Check memory usage
free -h

# Monitor ROS topics
rostopic hz /zed_obstacle/obstacle_array
``` 