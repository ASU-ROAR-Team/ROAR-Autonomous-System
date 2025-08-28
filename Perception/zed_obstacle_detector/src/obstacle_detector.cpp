#include "zed_obstacle_detector/obstacle_detector.h"
#include <pcl_conversions/pcl_conversions.h>
#include <ros/ros.h>

namespace zed_obstacle_detector {

/**
 * @brief Constructor for ObstacleDetector
 * 
 * Initializes the obstacle detection system with the provided parameters.
 * Creates and initializes all sub-components (processor, ground detector, 
 * cluster detector, transformer, tracker, and monitor).
 * 
 * @param params Configuration parameters for the obstacle detector
 * @throws std::exception if any component fails to initialize
 */
ObstacleDetector::ObstacleDetector(const ObstacleDetectorParams& params)
    : params_(params), initialized_(false) {
    
    if (initializeComponents()) {
        initialized_ = true;
        ROS_INFO("ObstacleDetector initialized successfully");
    } else {
        ROS_ERROR("Failed to initialize ObstacleDetector");
    }
}

/**
 * @brief Main processing function for point cloud obstacle detection
 * 
 * Processes a ROS PointCloud2 message through the complete obstacle detection pipeline:
 * 1. Preprocesses the point cloud (downsampling, filtering)
 * 2. Detects and removes ground plane
 * 3. Clusters remaining points into potential obstacles
 * 4. Transforms clusters to world coordinates
 * 5. Tracks obstacles across frames
 * 6. Creates output messages (ObstacleArray and visualization markers)
 * 
 * @param input_msg ROS PointCloud2 message containing the input point cloud
 * @return ObstacleDetectorResult containing detected obstacles, debug clouds, and performance metrics
 * 
 * @note This function is the main entry point for obstacle detection processing
 * @note Performance monitoring is automatically enabled if configured
 * @note Debug clouds are only populated if debug publishers are enabled
 */
ObstacleDetectorResult ObstacleDetector::processPointCloud(const sensor_msgs::PointCloud2ConstPtr& input_msg) {
    ObstacleDetectorResult result;
    
    if (!initialized_) {
        result.error_message = "ObstacleDetector not initialized";
        return result;
    }

    if (!input_msg) {
        result.error_message = "Input message is null";
        return result;
    }

    // Start performance monitoring
    monitor_->startFrame();
    monitor_->setFrameInfo(params_.world_frame, input_msg->header.stamp);
    monitor_->recordInputPoints(input_msg->width * input_msg->height);

    // Stage 1: Preprocess point cloud
    monitor_->startTimer("input_validation");
    pcl::PointCloud<pcl::PointXYZ>::Ptr processed_cloud(new pcl::PointCloud<pcl::PointXYZ>);
    if (!preprocessPointCloud(input_msg, processed_cloud)) {
        result.error_message = "Point cloud preprocessing failed";
        monitor_->endTimer("input_validation");
        monitor_->endFrame();
        return result;
    }
    monitor_->endTimer("input_validation");
    monitor_->recordOutputPoints(processed_cloud->size());

    // Fill debug cloud for publishing (only if debug publishers are enabled)
    if (params_.monitor_params.enable_debug_publishers) {
    pcl::toROSMsg(*processed_cloud, result.filtered_transformed_cloud);
    result.filtered_transformed_cloud.header.stamp = input_msg->header.stamp;
    result.filtered_transformed_cloud.header.frame_id = params_.input_frame_id;
    }

    // Stage 2: Ground detection and obstacle extraction
    monitor_->startTimer("ground_filter");
    pcl::PointCloud<pcl::PointXYZ>::Ptr obstacle_cloud(new pcl::PointCloud<pcl::PointXYZ>);
    bool ground_success = false;
    if (params_.enable_ground_filtering) {
        ground_success = ground_detector_->detectGround(processed_cloud, obstacle_cloud);
    } else {
        *obstacle_cloud = *processed_cloud;
        ground_success = true;
    }
    if (!ground_success) {
        result.error_message = "Ground detection failed";
        monitor_->endTimer("ground_filter");
        monitor_->endFrame();
        return result;
    }
    double ground_filter_duration = monitor_->endTimer("ground_filter");
    ROS_INFO("Ground filter duration: %.2f ms", ground_filter_duration);

    // Fill no-ground debug cloud for publishing
    if (params_.monitor_params.enable_debug_publishers) {
        pcl::toROSMsg(*obstacle_cloud, result.no_ground_cloud);
        result.no_ground_cloud.header.stamp = input_msg->header.stamp;
        result.no_ground_cloud.header.frame_id = params_.input_frame_id;
    }

    // Stage 3: Cluster detection
    monitor_->startTimer("clustering");
    std::vector<Cluster> clusters;
    if (!detectClusters(obstacle_cloud, clusters)) {
        result.error_message = "Cluster detection failed";
        monitor_->endTimer("clustering");
        monitor_->endFrame();
        return result;
    }
    monitor_->endTimer("clustering");
    monitor_->recordClustersDetected(clusters.size());

    // Fill debug cluster cloud for publishing
    if (params_.monitor_params.enable_debug_publishers) {
        pcl::PointCloud<pcl::PointXYZRGB>::Ptr debug_cloud = cluster_detector_->createDebugCloud(clusters, monitor_);
        pcl::toROSMsg(*debug_cloud, result.clusters_debug_cloud);
        result.clusters_debug_cloud.header.stamp = input_msg->header.stamp;
        result.clusters_debug_cloud.header.frame_id = params_.input_frame_id;
    }

    // Stage 4: Transform and track
    monitor_->startTimer("tracking");
    if (!transformAndTrack(clusters, input_msg->header.stamp, result, monitor_)) {
        result.error_message = "Transform and tracking failed";
        monitor_->endTimer("tracking");
        monitor_->endFrame();
        return result;
    }
    monitor_->endTimer("tracking");

    // End performance monitoring
    monitor_->endFrame();
    result.metrics = monitor_->getCurrentMetrics();
    result.success = true;

    // Log performance metrics
    monitor_->logPerformanceMetrics(result.metrics);

    return result;
}

/**
 * @brief Preprocesses a ROS PointCloud2 message into a PCL point cloud
 * 
 * Converts the ROS message to PCL format and validates the input.
 * Delegates the actual processing to the PointCloudProcessor component.
 * 
 * @param input_msg ROS PointCloud2 message to process
 * @param processed_cloud Output PCL point cloud after preprocessing
 * @return true if preprocessing succeeded, false otherwise
 * 
 * @note This function handles the conversion from ROS to PCL format
 * @note Empty clouds are rejected with a warning message
 */
bool ObstacleDetector::preprocessPointCloud(const sensor_msgs::PointCloud2ConstPtr& input_msg,
                                           pcl::PointCloud<pcl::PointXYZ>::Ptr& processed_cloud) {
    // Convert ROS message to PCL cloud
    pcl::PointCloud<pcl::PointXYZ>::Ptr input_cloud(new pcl::PointCloud<pcl::PointXYZ>);
    pcl::fromROSMsg(*input_msg, *input_cloud);

    if (input_cloud->empty()) {
        ROS_WARN_THROTTLE(1.0, "Input PCL cloud is empty!");
        return false;
    }

    // Process point cloud
    return processor_->processPointCloud(input_cloud, processed_cloud, monitor_);
}

/**
 * @brief Detects ground plane and extracts obstacle points
 * 
 * Either performs ground detection to separate obstacles from ground,
 * or skips ground filtering entirely based on configuration.
 * 
 * @param input_cloud Input point cloud to process
 * @param obstacle_cloud Output cloud containing only obstacle points (ground removed)
 * @return true if ground detection succeeded, false otherwise
 * 
 * @note Ground detection is performed in camera coordinates before transformation
 * @note If ground filtering is disabled, the entire input cloud is treated as obstacles
 */
bool ObstacleDetector::detectGroundAndObstacles(const pcl::PointCloud<pcl::PointXYZ>::Ptr& input_cloud,
                                               pcl::PointCloud<pcl::PointXYZ>::Ptr& obstacle_cloud) {
    if (!params_.enable_ground_filtering) {
        // Skip ground filtering, use input cloud as obstacle cloud
        *obstacle_cloud = *input_cloud;
        return true;
    }

    // Ground detection BEFORE transformation (all modes)
    // Detect ground directly in camera frame
    return ground_detector_->detectGround(input_cloud, obstacle_cloud);
}

/**
 * @brief Detects clusters in the obstacle point cloud
 * 
 * Uses the ClusterDetector component to group nearby points into clusters
 * representing potential obstacles.
 * 
 * @param obstacle_cloud Point cloud containing obstacle points (ground removed)
 * @param clusters Output vector of detected clusters
 * @return true if clustering succeeded and clusters were found, false otherwise
 * 
 * @note Empty obstacle clouds are rejected with a warning
 * @note Clusters contain centroid, radius, and point data for each detected obstacle
 */
bool ObstacleDetector::detectClusters(const pcl::PointCloud<pcl::PointXYZ>::Ptr& obstacle_cloud,
                                     std::vector<Cluster>& clusters) {
    if (obstacle_cloud->empty()) {
        ROS_WARN_THROTTLE(1.0, "Obstacle cloud is empty for clustering!");
        return false;
    }

    clusters = cluster_detector_->detectClusters(obstacle_cloud, monitor_);
    return !clusters.empty();
}

/**
 * @brief Transforms clusters to world coordinates and updates obstacle tracking
 * 
 * Converts cluster centroids and radii from camera coordinates to world coordinates,
 * then updates the obstacle tracker with the new detections. Creates output
 * messages for confirmed obstacles.
 * 
 * @param clusters Vector of clusters detected in camera coordinates
 * @param timestamp Timestamp of the current frame
 * @param result ObstacleDetectorResult to populate with output messages
 * @param monitor Performance monitor for timing measurements
 * @return true if transformation and tracking succeeded, false otherwise
 * 
 * @note Clusters are transformed to world frame only if transformations are enabled
 * @note Only confirmed obstacles (detected multiple times) are included in output
 * @note Output messages include ObstacleArray and visualization markers
 */
bool ObstacleDetector::transformAndTrack(const std::vector<Cluster>& clusters,
                                        const ros::Time& timestamp,
                                        ObstacleDetectorResult& result,
                                        std::shared_ptr<PerformanceMonitor> monitor) {
    // Convert clusters to detection pairs
    std::vector<std::pair<geometry_msgs::Point, float>> clusters_camera;
    clusters_camera.reserve(clusters.size());
    
    for (const auto& cluster : clusters) {
        clusters_camera.push_back({cluster.centroid, cluster.radius});
    }

    // Transform clusters from camera frame to world frame (or keep in input frame if transformations disabled)
    std::vector<std::pair<geometry_msgs::Point, float>> clusters_world;
    if (!transformer_->transformClustersToWorld(clusters_camera, params_.input_frame_id,
                                               clusters_world, timestamp, monitor)) {
        ROS_WARN_THROTTLE(1.0, "Failed to transform clusters from camera to world frame");
        return false;
    }

    // Update tracking
    tracker_->updateTracks(clusters_world, timestamp, monitor);
    
    // Get confirmed obstacles
    std::vector<TrackedObstacle> confirmed_obstacles = tracker_->getConfirmedObstacles();
    monitor_->recordObstaclesTracked(tracker_->getAllObstacles().size());
    monitor_->recordConfirmedObstacles(confirmed_obstacles.size());

    // Create output messages with correct frame
    createObstacleArray(confirmed_obstacles, result.obstacle_array);
    createMarkers(confirmed_obstacles, result.markers);

    return true;
}

/**
 * @brief Updates parameters for the obstacle detector and all its components
 * 
 * Propagates parameter changes to all sub-components (processor, ground detector,
 * cluster detector, transformer, tracker, and monitor).
 * 
 * @param params New parameters to apply
 * 
 * @note This function allows runtime parameter updates without restarting the system
 * @note All components are updated atomically
 */
void ObstacleDetector::setParams(const ObstacleDetectorParams& params) {
    params_ = params;
    
    // Update component parameters
    if (processor_) processor_->setParams(params_.processing_params);
    if (ground_detector_) ground_detector_->setParams(params_.ground_params);
    if (cluster_detector_) cluster_detector_->setParams(params_.cluster_params);
    if (transformer_) transformer_->setParams(params_.transform_params);
    if (tracker_) tracker_->setParams(params_.tracking_params);
    if (monitor_) monitor_->setParams(params_.monitor_params);
    
    ROS_DEBUG("ObstacleDetector parameters updated");
}

/**
 * @brief Resets the obstacle detector to initial state
 * 
 * Clears all tracked obstacles and resets performance monitoring metrics.
 * Useful for starting fresh after system reconfiguration or error recovery.
 * 
 * @note This function clears all obstacle tracking history
 * @note Performance metrics are reset to zero
 */
void ObstacleDetector::reset() {
    if (tracker_) {
        // Clear all tracked obstacles
        tracker_->updateTracks({}, ros::Time::now(), nullptr);
    }
    if (monitor_) {
        monitor_->resetMetrics();
    }
    ROS_DEBUG("ObstacleDetector reset");
}

/**
 * @brief Initializes all sub-components of the obstacle detector
 * 
 * Creates and initializes the processor, ground detector, cluster detector,
 * transformer, tracker, and monitor components with their respective parameters.
 * 
 * @return true if all components initialized successfully, false otherwise
 * 
 * @throws std::exception if any component fails to initialize
 * @note This function is called during construction
 */
bool ObstacleDetector::initializeComponents() {
    try {
        // Initialize all components
        processor_ = std::make_shared<PointCloudProcessor>(params_.processing_params);
        ground_detector_ = std::make_shared<GroundDetector>(params_.ground_params);
        cluster_detector_ = std::make_shared<ClusterDetector>(params_.cluster_params);
        transformer_ = std::make_shared<CoordinateTransformer>(params_.transform_params);
        tracker_ = std::make_shared<ObstacleTracker>(params_.tracking_params);
        monitor_ = std::make_shared<PerformanceMonitor>(params_.monitor_params);
        
        return true;
    } catch (const std::exception& e) {
        ROS_ERROR("Failed to initialize components: %s", e.what());
        return false;
    }
}

/**
 * @brief Creates visualization markers for tracked obstacles
 * 
 * Generates RViz visualization markers for all tracked obstacles.
 * Creates a DELETEALL marker to clear previous markers, then adds
 * sphere markers for each obstacle with position and radius information.
 * 
 * @param obstacles Vector of tracked obstacles to visualize
 * @param markers Output MarkerArray message containing visualization markers
 * 
 * @note Markers are created in the appropriate coordinate frame (world or input)
 * @note Each obstacle is represented as a red sphere with diameter = 2 * radius
 * @note Markers persist for 30 seconds and are automatically cleared
 */
void ObstacleDetector::createMarkers(const std::vector<TrackedObstacle>& obstacles,
                                    visualization_msgs::MarkerArray& markers) {
    // Determine the correct frame to use
    std::string frame_id = params_.transform_params.enable_transformations ? 
                          params_.world_frame : params_.input_frame_id;
    
    // Clear all existing markers
    visualization_msgs::Marker deleteAllMarker;
    deleteAllMarker.header.stamp = ros::Time::now();
    deleteAllMarker.header.frame_id = frame_id;
    deleteAllMarker.ns = "tracked_world_obstacles";
    deleteAllMarker.id = 0;
    deleteAllMarker.action = visualization_msgs::Marker::DELETEALL;
    markers.markers.push_back(deleteAllMarker);

    // Create markers for each obstacle
    for (const auto& obs : obstacles) {
        visualization_msgs::Marker marker;
        marker.header.stamp = ros::Time::now();
        marker.header.frame_id = frame_id;
        marker.ns = "tracked_world_obstacles";
        marker.id = obs.id;
        marker.type = visualization_msgs::Marker::SPHERE;
        marker.action = visualization_msgs::Marker::ADD;
        marker.pose.position = obs.position_world;  // This is correct regardless of frame
        marker.pose.orientation.w = 1.0;
        marker.scale.x = 2.0 * obs.radius_world;
        marker.scale.y = 2.0 * obs.radius_world;
        marker.scale.z = 2.0 * obs.radius_world;
        marker.color.r = 1.0f;
        marker.color.g = 0.0f;
        marker.color.b = 0.0f;
        marker.color.a = 1.0f;
        marker.lifetime = ros::Duration(30.0); // Persist for 30 seconds
        markers.markers.push_back(marker);
    }
}

/**
 * @brief Creates ROS ObstacleArray message from tracked obstacles
 * 
 * Converts tracked obstacles into a standardized ROS message format
 * for communication with other nodes in the system.
 * 
 * @param obstacles Vector of tracked obstacles to convert
 * @param obstacle_array Output ObstacleArray message
 * 
 * @note Each obstacle includes ID, position, radius, and timestamp information
 * @note The message frame_id is set to world frame if transformations are enabled
 * @note Obstacle positions are in the appropriate coordinate frame
 */
void ObstacleDetector::createObstacleArray(const std::vector<TrackedObstacle>& obstacles,
                                          roar_msgs::ObstacleArray& obstacle_array) {
    // Determine the correct frame to use
    std::string frame_id = params_.transform_params.enable_transformations ? 
                          params_.world_frame : params_.input_frame_id;
    
    obstacle_array.header.stamp = ros::Time::now();
    obstacle_array.header.frame_id = frame_id;
    obstacle_array.obstacles.clear();
    obstacle_array.obstacles.reserve(obstacles.size());

    for (const auto& obs : obstacles) {
        roar_msgs::Obstacle obs_msg;
        obs_msg.header.stamp = obs.last_seen;
        obs_msg.header.frame_id = frame_id;
        obs_msg.id.data = obs.id;

        obs_msg.position.header.stamp = obs.last_seen;
        obs_msg.position.header.frame_id = frame_id;
        obs_msg.position.pose.position = obs.position_world;  // This is correct regardless of frame
        obs_msg.position.pose.orientation.w = 1.0;

        obs_msg.radius.data = obs.radius_world;
        obstacle_array.obstacles.push_back(obs_msg);
    }
}

} // namespace zed_obstacle_detector 