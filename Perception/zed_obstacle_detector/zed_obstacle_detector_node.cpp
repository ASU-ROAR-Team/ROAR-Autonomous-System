// zed_obstacle_detector_node_new.cpp
// Simplified ROS node using modular ObstacleDetector architecture

#include <ros/ros.h>
#include <sensor_msgs/PointCloud2.h>
#include <roar_msgs/ObstacleArray.h>
#include <visualization_msgs/MarkerArray.h>
#include <sensor_msgs/PointCloud2.h>

#include "zed_obstacle_detector/obstacle_detector.h"

// Publishers
ros::Publisher pub_obstacle_array;
ros::Publisher pub_markers;
ros::Publisher pub_filtered_transformed;
ros::Publisher pub_no_ground;
ros::Publisher pub_clusters_debug;

// Main obstacle detector instance
std::unique_ptr<zed_obstacle_detector::ObstacleDetector> obstacle_detector;

// Helper function to reload parameters live
void reload_params(ros::NodeHandle& nh, zed_obstacle_detector::ObstacleDetectorParams& params) {
    // Processing
    nh.getParam("voxel_leaf_size", params.processing_params.voxel_leaf_size);
    nh.getParam("enable_uniform_downsampling", params.processing_params.enable_uniform_downsampling);
    nh.getParam("uniform_sampling_radius", params.processing_params.uniform_sampling_radius);
    nh.getParam("enable_early_exit", params.processing_params.enable_early_exit);
    nh.getParam("min_points_for_processing", params.processing_params.min_points_for_processing);
    nh.getParam("max_points_for_processing", params.processing_params.max_points_for_processing);
    nh.getParam("passthrough_z_min_camera", params.processing_params.passthrough_z_min);
    nh.getParam("passthrough_z_max_camera", params.processing_params.passthrough_z_max);
    // Ground
    nh.getParam("enable_ground_filtering", params.enable_ground_filtering);
    nh.getParam("ground_filter_distance_threshold", params.ground_params.distance_threshold);
    nh.getParam("ground_filter_angle_threshold_deg", params.ground_params.angle_threshold_deg);
    nh.getParam("ground_filter_max_iterations", params.ground_params.max_iterations);
    nh.getParam("mars_terrain_mode", params.ground_params.mars_terrain_mode);
    // Clustering
    nh.getParam("cluster_tolerance", params.cluster_params.cluster_tolerance);
    nh.getParam("min_cluster_size", params.cluster_params.min_cluster_size);
    nh.getParam("max_cluster_size", params.cluster_params.max_cluster_size);
    // Tracking
    double assoc_dist = 1.5;
    if (nh.getParam("obstacle_association_distance", assoc_dist))
        params.tracking_params.association_distance_sq = assoc_dist * assoc_dist;
    nh.getParam("obstacle_timeout_sec", params.tracking_params.timeout_sec);
    nh.getParam("position_smoothing_factor", params.tracking_params.position_smoothing_factor);
    nh.getParam("min_detections_for_confirmation", params.tracking_params.min_detections_for_confirmation);
    // Monitor
    nh.getParam("enable_detailed_timing", params.monitor_params.enable_detailed_timing);
    nh.getParam("enable_debug_publishers", params.monitor_params.enable_debug_publishers);
    nh.getParam("timing_report_interval", params.monitor_params.timing_report_interval);
    nh.getParam("enable_performance_logging", params.monitor_params.enable_performance_logging);
    nh.getParam("log_file_path", params.monitor_params.log_file_path);
}

void pointCloudCallback(const sensor_msgs::PointCloud2ConstPtr& input_msg) {
    // Reload parameters live
    static ros::NodeHandle private_nh("~");
    static zed_obstacle_detector::ObstacleDetectorParams params = obstacle_detector->getParams();
    reload_params(private_nh, params);
    obstacle_detector->setParams(params);

    // Print updated parameters
    ROS_INFO_THROTTLE(5.0, "Updated parameters:");
    ROS_INFO_THROTTLE(5.0, "  Processing:");
    ROS_INFO_THROTTLE(5.0, "    Voxel leaf size: %.3f", params.processing_params.voxel_leaf_size);
    ROS_INFO_THROTTLE(5.0, "    Uniform sampling: %s (radius: %.3f)", 
        params.processing_params.enable_uniform_downsampling ? "enabled" : "disabled",
        params.processing_params.uniform_sampling_radius);
    ROS_INFO_THROTTLE(5.0, "    Z range: [%.1f, %.1f]", 
        params.processing_params.passthrough_z_min,
        params.processing_params.passthrough_z_max);
    
    ROS_INFO_THROTTLE(5.0, "  Ground filtering: %s", params.enable_ground_filtering ? "enabled" : "disabled");
    if (params.enable_ground_filtering) {
        ROS_INFO_THROTTLE(5.0, "    Distance threshold: %.3f", params.ground_params.distance_threshold);
        ROS_INFO_THROTTLE(5.0, "    Angle threshold: %.1f deg", params.ground_params.angle_threshold_deg);
    }
    
    ROS_INFO_THROTTLE(5.0, "  Clustering:");
    ROS_INFO_THROTTLE(5.0, "    Tolerance: %.3f", params.cluster_params.cluster_tolerance);
    ROS_INFO_THROTTLE(5.0, "    Size range: [%d, %d]", 
        params.cluster_params.min_cluster_size,
        params.cluster_params.max_cluster_size);
    
    ROS_INFO_THROTTLE(5.0, "  Tracking:");
    ROS_INFO_THROTTLE(5.0, "    Association distance: %.2f", sqrt(params.tracking_params.association_distance_sq));
    ROS_INFO_THROTTLE(5.0, "    Timeout: %.1f sec", params.tracking_params.timeout_sec);
    
    ROS_INFO_THROTTLE(5.0, "Received point cloud. Processing for obstacle tracking...");
    
    // Process point cloud using modular detector
    zed_obstacle_detector::ObstacleDetectorResult result = obstacle_detector->processPointCloud(input_msg);
    
    if (!result.success) {
        ROS_ERROR_THROTTLE(1.0, "Obstacle detection failed: %s", result.error_message.c_str());
        return;
    }
    
    // Publish results
    pub_obstacle_array.publish(result.obstacle_array);
    pub_markers.publish(result.markers);
    
    // Publish debug clouds if enabled
    if (obstacle_detector->getParams().monitor_params.enable_debug_publishers) {
        pub_filtered_transformed.publish(result.filtered_transformed_cloud);
        pub_clusters_debug.publish(result.clusters_debug_cloud);
        pub_no_ground.publish(result.no_ground_cloud);
    }
}

int main(int argc, char** argv) {
    ros::init(argc, argv, "zed_obstacle_detector");
    ros::NodeHandle nh;
    ros::NodeHandle private_nh("~");

    // Load parameters - Updated for ZED2i real-world testing
    std::string point_cloud_topic;
    private_nh.param<std::string>("point_cloud_topic", point_cloud_topic, "/zed2i/zed_node/point_cloud/cloud_registered");

    // Initialize obstacle detector parameters
    zed_obstacle_detector::ObstacleDetectorParams params;
    
    // General settings - Updated for real-world ZED2i
    private_nh.param<std::string>("base_link_frame", params.base_link_frame, "base_link");
    private_nh.param<std::string>("world_frame", params.world_frame, "world");
    private_nh.param<bool>("enable_ground_filtering", params.enable_ground_filtering, true);
    private_nh.param<bool>("enable_debug_output", params.enable_debug_output, false);
    
    // Parameterize camera frame
    std::string camera_frame;
    private_nh.param<std::string>("camera_frame", camera_frame, "zed2i_left_camera_frame");
    params.input_frame_id = camera_frame;  // Set input frame to camera frame
    
    // Processing parameters - Optimized for ZED2i and Jetson performance
    private_nh.param("passthrough_z_min_camera", params.processing_params.passthrough_z_min, 0.2);
    private_nh.param("passthrough_z_max_camera", params.processing_params.passthrough_z_max, 7.0);
    private_nh.param("voxel_leaf_size", params.processing_params.voxel_leaf_size, 0.08); // Increased for performance
    private_nh.param("enable_uniform_downsampling", params.processing_params.enable_uniform_downsampling, false);
    private_nh.param("uniform_sampling_radius", params.processing_params.uniform_sampling_radius, 0.08);
    private_nh.param("enable_early_exit", params.processing_params.enable_early_exit, true); // Enable for performance
    private_nh.param("min_points_for_processing", params.processing_params.min_points_for_processing, 200);
    private_nh.param("max_points_for_processing", params.processing_params.max_points_for_processing, 15000);
    
    // Ground detection parameters - Optimized for real-world terrain
    private_nh.param("ground_filter_distance_threshold", params.ground_params.distance_threshold, 0.08);
    private_nh.param("ground_filter_angle_threshold_deg", params.ground_params.angle_threshold_deg, 15.0);
    private_nh.param("ground_filter_max_iterations", params.ground_params.max_iterations, 200);
    private_nh.param("mars_terrain_mode", params.ground_params.mars_terrain_mode, false); // Disable for real-world
    
    // Clustering parameters - Optimized for real-world obstacles
    private_nh.param("cluster_tolerance", params.cluster_params.cluster_tolerance, 0.3);
    private_nh.param("min_cluster_size", params.cluster_params.min_cluster_size, 20);
    private_nh.param("max_cluster_size", params.cluster_params.max_cluster_size, 15000);
    params.cluster_params.enable_debug_output = params.enable_debug_output;
    
    // Transform parameters - Optimized for real-world TF
    private_nh.param("tf_lookup_timeout", params.transform_params.tf_lookup_timeout, 0.05); // Faster timeout
    private_nh.param("tf_buffer_duration", params.transform_params.tf_buffer_duration, 5.0); // Shorter buffer
    private_nh.param("enable_transformations", params.transform_params.enable_transformations, true); // New parameter to disable transformations
    
    params.transform_params.source_frame = camera_frame;
    params.transform_params.target_frame = params.base_link_frame;
    params.transform_params.world_frame = params.world_frame;
    params.transform_params.enable_debug_output = params.enable_debug_output;
    
    // Tracking parameters - Optimized for real-world tracking
    double assoc_dist;
    private_nh.param("obstacle_association_distance", assoc_dist, 1.5); // Reduced for tighter tracking
    params.tracking_params.association_distance_sq = assoc_dist * assoc_dist;
    private_nh.param("obstacle_timeout_sec", params.tracking_params.timeout_sec, 30.0); // Shorter timeout
    private_nh.param("position_smoothing_factor", params.tracking_params.position_smoothing_factor, 0.2); // Less smoothing
    private_nh.param("min_detections_for_confirmation", params.tracking_params.min_detections_for_confirmation, 3); // More confirmations
    
    // Monitor parameters - Optimized for Jetson performance
    private_nh.param("enable_detailed_timing", params.monitor_params.enable_detailed_timing, false); // Disable for performance
    private_nh.param("enable_debug_publishers", params.monitor_params.enable_debug_publishers, false); // Disable for performance
    private_nh.param("timing_report_interval", params.monitor_params.timing_report_interval, 30.0); // Less frequent
    private_nh.param("enable_performance_logging", params.monitor_params.enable_performance_logging, true);
    private_nh.param<std::string>("log_file_path", params.monitor_params.log_file_path, "/tmp/zed_obstacle_detector.log");

    // Initialize obstacle detector
    obstacle_detector = std::make_unique<zed_obstacle_detector::ObstacleDetector>(params);
    
    if (!obstacle_detector->isInitialized()) {
        ROS_ERROR("Failed to initialize ObstacleDetector. Exiting.");
        return -1;
    }

    // Print configuration
    ROS_INFO("--- ZED2i Real-World Obstacle Detector Configuration ---");
    ROS_INFO("Input Point Cloud Topic: %s", point_cloud_topic.c_str());
    ROS_INFO("Base Link Frame: %s", params.base_link_frame.c_str());
    ROS_INFO("World Frame: %s", params.world_frame.c_str());
    ROS_INFO("Camera Frame: %s", camera_frame.c_str());
    ROS_INFO("Ground Filtering: %s", params.enable_ground_filtering ? "Enabled" : "Disabled");
    ROS_INFO("Voxel Leaf Size: %.3f m (optimized for Jetson)", params.processing_params.voxel_leaf_size);
    ROS_INFO("Cluster Tolerance: %.2f m", params.cluster_params.cluster_tolerance);
    ROS_INFO("Association Distance: %.2f m", assoc_dist);
    ROS_INFO("Early Exit: %s", params.processing_params.enable_early_exit ? "Enabled" : "Disabled");
    ROS_INFO("Detailed Timing: %s", params.monitor_params.enable_detailed_timing ? "Enabled" : "Disabled");
    ROS_INFO("Debug Publishers: %s", params.monitor_params.enable_debug_publishers ? "Enabled" : "Disabled");
    ROS_INFO("Performance Logging: %s", params.monitor_params.enable_performance_logging ? "Enabled" : "Disabled");
    ROS_INFO("Log File: %s", params.monitor_params.log_file_path.c_str());
    ROS_INFO("--------------------------------------------------------");

    // Setup subscribers and publishers
    ros::Subscriber sub = nh.subscribe(point_cloud_topic, 1, pointCloudCallback);
    
    pub_obstacle_array = nh.advertise<roar_msgs::ObstacleArray>("/zed_obstacle/obstacle_array", 10);
    pub_markers = nh.advertise<visualization_msgs::MarkerArray>("/zed_obstacle/markers", 10);
    
    // Debug publishers (only if enabled)
    if (params.monitor_params.enable_debug_publishers) {
        pub_filtered_transformed = nh.advertise<sensor_msgs::PointCloud2>("/zed_obstacle/debug/filtered_transformed_pc", 1);
        pub_no_ground = nh.advertise<sensor_msgs::PointCloud2>("/zed_obstacle/debug/pc_no_ground", 1);
        pub_clusters_debug = nh.advertise<sensor_msgs::PointCloud2>("/zed_obstacle/debug/raw_clusters_rgb", 1);
    }

    ROS_INFO("ZED2i Real-World Obstacle Detector initialized. Waiting for point clouds on %s...", point_cloud_topic.c_str());
    ros::spin();
    return 0;
} 