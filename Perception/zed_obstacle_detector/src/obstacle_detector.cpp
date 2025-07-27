#include "zed_obstacle_detector/obstacle_detector.h"
#include <pcl_conversions/pcl_conversions.h>
#include <ros/ros.h>

namespace zed_obstacle_detector {

ObstacleDetector::ObstacleDetector(const ObstacleDetectorParams& params)
    : params_(params), initialized_(false) {
    
    if (initializeComponents()) {
        initialized_ = true;
        ROS_INFO("ObstacleDetector initialized successfully");
    } else {
        ROS_ERROR("Failed to initialize ObstacleDetector");
    }
}

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
    if (!transformAndTrack(clusters, input_msg->header.stamp, result)) {
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

bool ObstacleDetector::detectClusters(const pcl::PointCloud<pcl::PointXYZ>::Ptr& obstacle_cloud,
                                     std::vector<Cluster>& clusters) {
    if (obstacle_cloud->empty()) {
        ROS_WARN_THROTTLE(1.0, "Obstacle cloud is empty for clustering!");
        return false;
    }

    clusters = cluster_detector_->detectClusters(obstacle_cloud, monitor_);
    return !clusters.empty();
}

bool ObstacleDetector::transformAndTrack(const std::vector<Cluster>& clusters,
                                        const ros::Time& timestamp,
                                        ObstacleDetectorResult& result) {
    // Convert clusters to detection pairs
    std::vector<std::pair<geometry_msgs::Point, float>> clusters_camera;
    clusters_camera.reserve(clusters.size());
    
    for (const auto& cluster : clusters) {
        clusters_camera.push_back({cluster.centroid, cluster.radius});
    }

    // Transform clusters from camera frame to world frame (or keep in input frame if transformations disabled)
    std::vector<std::pair<geometry_msgs::Point, float>> clusters_world;
    if (!transformer_->transformClustersToWorld(clusters_camera, params_.input_frame_id,
                                               clusters_world, timestamp, monitor_)) {
        ROS_WARN_THROTTLE(1.0, "Failed to transform clusters from camera to world frame");
        return false;
    }

    // Update tracking
    tracker_->updateTracks(clusters_world, timestamp);
    
    // Get confirmed obstacles
    std::vector<TrackedObstacle> confirmed_obstacles = tracker_->getConfirmedObstacles();
    monitor_->recordObstaclesTracked(tracker_->getAllObstacles().size());
    monitor_->recordConfirmedObstacles(confirmed_obstacles.size());

    // Create output messages with correct frame
    createObstacleArray(confirmed_obstacles, result.obstacle_array);
    createMarkers(confirmed_obstacles, result.markers);

    return true;
}

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

void ObstacleDetector::reset() {
    if (tracker_) {
        // Clear all tracked obstacles
        tracker_->updateTracks({}, ros::Time::now());
    }
    if (monitor_) {
        monitor_->resetMetrics();
    }
    ROS_DEBUG("ObstacleDetector reset");
}

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