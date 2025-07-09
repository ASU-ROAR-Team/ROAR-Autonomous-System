# ZED Obstacle Detector - Parameter Namespace & TF Transformation Analysis

## 1. Parameter Namespace Structure

### Current Architecture
The node uses **private parameters** (accessed via `ros::NodeHandle private_nh("~")`), which means all parameters are in the namespace `/zed_obstacle_detector/`.

### Parameter Loading Hierarchy
1. **Launch File Parameters** (highest priority)
   - Set directly in launch files via `<param>` tags
   - Example: `<param name="voxel_leaf_size" value="0.08" />`
   - These are in the node's private namespace: `/zed_obstacle_detector/voxel_leaf_size`

2. **YAML Parameter Files** (medium priority)
   - Loaded via `rosparam load` or `<rosparam>` tags
   - Must be structured under the node's namespace
   - Example: `zed_obstacle_detector: voxel_leaf_size: 0.08`

3. **Command Line Overrides** (highest priority)
   - Set via `rosparam set` or launch file arguments
   - Example: `rosparam set /zed_obstacle_detector/voxel_leaf_size 0.05`

### Parameter Categories

#### A. Frame Configuration Parameters
```yaml
zed_obstacle_detector:
  camera_frame: "zed2i_left_camera_frame"      # Source frame for point clouds
  base_link_frame: "base_link"                 # Target frame for transformations
  world_frame: "world"                         # Global frame for tracking
```

#### B. Processing Parameters
```yaml
zed_obstacle_detector:
  # Point cloud filtering
  passthrough_z_min_camera: 0.2               # Min distance in camera frame
  passthrough_z_max_camera: 7.0               # Max distance in camera frame
  voxel_leaf_size: 0.08                       # Downsampling resolution
  enable_uniform_downsampling: false          # Additional downsampling
  uniform_sampling_radius: 0.08               # Uniform sampling radius
  
  # Performance optimization
  enable_early_exit: true                     # Skip processing for small/large clouds
  min_points_for_processing: 200              # Minimum points to process
  max_points_for_processing: 15000            # Maximum points before downsampling
```

#### C. Ground Detection Parameters
```yaml
zed_obstacle_detector:
  enable_ground_filtering: true               # Enable/disable ground removal
  ground_filter_distance_threshold: 0.08      # Distance threshold for ground plane
  ground_filter_angle_threshold_deg: 15.0     # Angle threshold around Z-axis
  ground_filter_max_iterations: 200           # RANSAC iterations
  mars_terrain_mode: false                    # Mars-specific optimizations
```

#### D. Clustering Parameters
```yaml
zed_obstacle_detector:
  cluster_tolerance: 0.3                      # Distance between points in cluster
  min_cluster_size: 20                        # Minimum points per cluster
  max_cluster_size: 15000                     # Maximum points per cluster
```

#### E. Transform Parameters
```yaml
zed_obstacle_detector:
  tf_lookup_timeout: 0.05                     # TF lookup timeout (seconds)
  tf_buffer_duration: 5.0                     # TF buffer duration (seconds)
```

#### F. Tracking Parameters
```yaml
zed_obstacle_detector:
  obstacle_association_distance: 1.5          # Distance to associate obstacles
  obstacle_timeout_sec: 30.0                  # Time before obstacle expires
  position_smoothing_factor: 0.2              # Smoothing factor (0-1)
  min_detections_for_confirmation: 3          # Detections needed to confirm
```

#### G. Performance Monitoring Parameters
```yaml
zed_obstacle_detector:
  enable_detailed_timing: false               # Enable detailed timing output
  enable_debug_publishers: false              # Enable debug point cloud publishers
  timing_report_interval: 30.0                # Timing report frequency
  enable_performance_logging: true            # Enable performance logging
  log_file_path: "/tmp/zed_obstacle_detector.log"
```

## 2. TF (Transform) System Deep Dive

### What is TF?
**TF (Transform)** is ROS's coordinate frame transformation system. It maintains a tree of coordinate frames and provides transformations between any two frames in the tree.

### Key Concepts

#### A. Coordinate Frames
- **Camera Frame** (`zed2i_left_camera_frame`): Where the point cloud originates
- **Base Link Frame** (`base_link`): Robot's main coordinate frame
- **World Frame** (`world`): Global coordinate frame for tracking

#### B. Transform Tree Structure
```
world
└── base_link
    └── zed2i_left_camera_frame
```

### How Transformations Work

#### A. Point Cloud Transformation Process
1. **Input**: Point cloud in camera frame
2. **Transform**: Each point is transformed using a 4x4 transformation matrix
3. **Output**: Point cloud in target frame (base_link or world)

#### B. Mathematical Process
For each point P in the camera frame:
```
P_base_link = T_camera_to_base_link * P_camera
P_world = T_base_link_to_world * P_base_link
```

Where T is a 4x4 homogeneous transformation matrix:
```
T = [R  t]
    [0  1]
```
- R: 3x3 rotation matrix
- t: 3x1 translation vector

#### C. Code Implementation
```cpp
// In coordinate_transformer.cpp
bool CoordinateTransformer::transformPointCloud(
    const pcl::PointCloud<pcl::PointXYZ>::Ptr& input_cloud,
    const std::string& source_frame,
    const std::string& target_frame,
    pcl::PointCloud<pcl::PointXYZ>::Ptr& output_cloud,
    const ros::Time& timestamp,
    TransformTiming& timing) {
    
    // 1. Check if transform is available
    if (!isTransformAvailable(source_frame, target_frame, timestamp)) {
        return false;
    }
    
    // 2. Perform transformation using PCL's transformPointCloud
    if (!pcl_ros::transformPointCloud(target_frame, *input_cloud, *output_cloud, *tf_buffer_)) {
        return false;
    }
    
    return true;
}
```

### Transformation Pipeline in the Obstacle Detector

#### Stage 1: Camera Frame Processing
- Point cloud arrives in camera frame
- PassThrough filtering applied in camera frame
- Voxel downsampling applied in camera frame

#### Stage 2: Ground Detection (Optional)
- If `enable_ground_filtering = true`:
  - Transform to base_link frame for ground detection
  - Apply RANSAC ground plane detection
  - Remove ground points
  - Transform remaining points back to camera frame

#### Stage 3: Clustering
- Clustering performed in camera frame
- Cluster centroids calculated in camera frame

#### Stage 4: Coordinate Transformation
- Transform cluster centroids to base_link frame
- Transform cluster centroids to world frame (for tracking)

#### Stage 5: Tracking
- Obstacle tracking performed in world frame
- Output published in world frame

### TF Buffer and Listener

#### A. TF Buffer
- Stores recent transforms in memory
- Configurable duration via `tf_buffer_duration`
- Provides fast lookup for recent transforms

#### B. TF Listener
- Subscribes to `/tf` and `/tf_static` topics
- Updates the TF buffer with new transforms
- Handles transform interpolation

#### C. Transform Availability Check
```cpp
bool CoordinateTransformer::isTransformAvailable(
    const std::string& source_frame,
    const std::string& target_frame,
    const ros::Time& timestamp) const {
    
    return tf_buffer_->canTransform(target_frame, source_frame, timestamp, 
                                   ros::Duration(params_.tf_lookup_timeout));
}
```

## 3. Parameter Namespace Issues and Solutions

### Common Issues

#### A. Parameter Not Found
**Problem**: `rosparam get /zed_obstacle_detector/voxel_leaf_size` returns "Parameter not found"

**Solution**: Ensure parameter is set in the correct namespace:
```bash
# Correct way
rosparam set /zed_obstacle_detector/voxel_leaf_size 0.05

# Check if it's set
rosparam get /zed_obstacle_detector/voxel_leaf_size
```

#### B. YAML Loading Issues
**Problem**: Parameters from YAML file not being loaded

**Solution**: Ensure YAML structure matches node namespace:
```yaml
# Correct structure
zed_obstacle_detector:
  voxel_leaf_size: 0.05
  cluster_tolerance: 0.3

# Load correctly
rosparam load config/params.yaml /zed_obstacle_detector
```

#### C. Launch File Parameter Conflicts
**Problem**: Launch file parameters overriding YAML parameters

**Solution**: Use parameter precedence:
1. Launch file parameters (highest priority)
2. Command line overrides
3. YAML file parameters (lowest priority)

### Best Practices

#### A. Parameter Organization
```yaml
zed_obstacle_detector:
  # Frame configuration
  camera_frame: "zed2i_left_camera_frame"
  base_link_frame: "base_link"
  world_frame: "world"
  
  # Processing parameters
  processing:
    voxel_leaf_size: 0.08
    cluster_tolerance: 0.3
  
  # Performance parameters
  performance:
    enable_early_exit: true
    min_points_for_processing: 200
```

#### B. Parameter Validation
```cpp
// In the node
private_nh.param("voxel_leaf_size", voxel_size, 0.08);
if (voxel_size <= 0.0) {
    ROS_ERROR("Invalid voxel_leaf_size: %f", voxel_size);
    return -1;
}
```

#### C. Default Values
Always provide sensible default values in the node:
```cpp
private_nh.param("voxel_leaf_size", params.processing_params.voxel_leaf_size, 0.08);
```

## 4. Performance Considerations

### A. TF Performance
- **Transform Lookup**: ~0.1ms per lookup
- **Point Cloud Transform**: ~1-10ms for typical clouds
- **Buffer Size**: Larger buffer = more memory, faster lookup
- **Timeout**: Shorter timeout = faster failure detection

### B. Parameter Access Performance
- **NodeHandle::param**: ~1μs per parameter
- **rosparam get/set**: ~10μs per operation
- **YAML loading**: ~1-10ms for typical files

### C. Memory Usage
- **TF Buffer**: ~1-10MB depending on buffer duration
- **Point Clouds**: ~1-100MB depending on resolution
- **Parameter Storage**: Negligible

## 5. Debugging and Troubleshooting

### A. TF Issues
```bash
# Check TF tree
rosrun tf view_frames
evince frames.pdf

# Check specific transform
rosrun tf tf_echo zed2i_left_camera_frame base_link

# Check TF topics
rostopic echo /tf
rostopic echo /tf_static
```

### B. Parameter Issues
```bash
# List all parameters in namespace
rosparam list /zed_obstacle_detector

# Get specific parameter
rosparam get /zed_obstacle_detector/voxel_leaf_size

# Set parameter
rosparam set /zed_obstacle_detector/voxel_leaf_size 0.05

# Load YAML file
rosparam load config/params.yaml /zed_obstacle_detector
```

### C. Node Debugging
```bash
# Check node parameters
rosnode info /zed_obstacle_detector

# Check node topics
rostopic list | grep zed_obstacle

# Check node logs
rosnode log /zed_obstacle_detector
```

## 6. Recommendations

### A. For Live Camera
1. Use `zed_camera_generic.launch` with appropriate performance mode
2. Ensure TF tree is properly configured
3. Use `base_link` and `world` frames for tracking

### B. For Rosbag Data
1. Use YAML config with camera frame as both source and target
2. Disable ground filtering if TF is not available
3. Set `enable_ground_filtering: false` and `world_frame: "camera_frame"`

### C. For Development
1. Use debug mode for detailed timing and debug output
2. Enable debug publishers for visualization
3. Use smaller voxel sizes for higher resolution

### D. For Production
1. Use production or high_performance mode
2. Disable debug output for better performance
3. Use appropriate voxel sizes for your use case 