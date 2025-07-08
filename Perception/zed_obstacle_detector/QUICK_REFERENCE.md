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

### Legacy Launch Files
```bash
# Production Mode
roslaunch zed_obstacle_detector zed2i_real_world.launch

# Debug Mode
roslaunch zed_obstacle_detector zed2i_debug.launch
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
- `enable_debug_publishers`: Enable debug point cloud topics

### Frame Configuration
- `base_link_frame`: Robot base frame (default: base_link)
- `world_frame`: World coordinate frame (default: world)

## Output Topics

### Main Outputs
- `/zed_obstacle/obstacle_array` - Detected obstacles
- `/zed_obstacle/markers` - Visualization markers

### Debug Outputs (debug mode only)
- `/zed_obstacle/debug/filtered_transformed_pc` - Filtered point cloud
- `/zed_obstacle/debug/pc_no_ground` - Ground-removed point cloud
- `/zed_obstacle/debug/raw_clusters_rgb` - Clustered point cloud

## Common Use Cases

### Development & Testing
```bash
roslaunch zed_obstacle_detector zed_camera_generic.launch camera_model:=zed2i performance_mode:=debug
```

### Real-World Deployment
```bash
roslaunch zed_obstacle_detector zed_camera_generic.launch camera_model:=zed2i performance_mode:=production
```

### Resource-Constrained Systems
```bash
roslaunch zed_obstacle_detector zed_camera_generic.launch camera_model:=zed2i performance_mode:=high_performance
```

### Custom Configuration
```bash
roslaunch zed_obstacle_detector zed_camera_generic.launch camera_model:=zed2i camera_frame:=zed2i_right_camera_frame performance_mode:=production
```

## Troubleshooting

### TF Errors
```bash
# Check TF tree
rosrun tf tf_echo zed2i_left_camera_frame base_link

# Verify camera frame exists
rostopic echo /tf_static
```

### Performance Issues
- Use `high_performance` mode for Jetson
- Increase `voxel_leaf_size` parameter
- Enable `enable_early_exit`

### No Obstacles Detected
- Check point cloud topic: `rostopic list | grep zed`
- Verify camera frame matches TF tree
- Adjust clustering parameters

## Log Files
- Production: `/tmp/zed_obstacle_detector_zed2i_production.log`
- Debug: `/tmp/zed_obstacle_detector_zed2i_debug.log`
- High Performance: `/tmp/zed_obstacle_detector_zed2i_high_performance.log` 