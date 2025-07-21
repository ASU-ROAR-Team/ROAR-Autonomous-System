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

// Helper function to print parameters
void print_parameters(const zed_obstacle_detector::ObstacleDetectorParams& params, 
                     const std::string& point_cloud_topic, const std::string& camera_frame) {
    ROS_INFO("--- ZED2i Real-World Obstacle Detector Configuration ---");
    ROS_INFO("Input Point Cloud Topic: %s", point_cloud_topic.c_str());
    ROS_INFO("Base Link Frame: %s", params.base_link_frame.c_str());
    ROS_INFO("World Frame: %s", params.world_frame.c_str());
    ROS_INFO("Camera Frame: %s", camera_frame.c_str());
    ROS_INFO("Ground Filtering: %s", params.enable_ground_filtering ? "Enabled" : "Disabled");
    ROS_INFO("Voxel Leaf Size: %.3f m (optimized for Jetson)", params.processing_params.voxel_leaf_size);
    ROS_INFO("Cluster Tolerance: %.2f m", params.cluster_params.cluster_tolerance);
    ROS_INFO("Association Distance: %.2f m", sqrt(params.tracking_params.association_distance_sq));
    ROS_INFO("Early Exit: %s", params.processing_params.enable_early_exit ? "Enabled" : "Disabled");
    ROS_INFO("Detailed Timing: %s", params.monitor_params.enable_detailed_timing ? "Enabled" : "Disabled");
    ROS_INFO("Debug Publishers: %s", params.monitor_params.enable_debug_publishers ? "Enabled" : "Disabled");
    ROS_INFO("Performance Logging: %s", params.monitor_params.enable_performance_logging ? "Enabled" : "Disabled");
    ROS_INFO("Log File: %s", params.monitor_params.log_file_path.c_str());
    ROS_INFO("--------------------------------------------------------");
}


// Helper function to load/update parameters with defaults for initialization
void load_params(ros::NodeHandle& nh, zed_obstacle_detector::ObstacleDetectorParams& params, 
                 std::string& point_cloud_topic, std::string& camera_frame, bool is_initialization = false) {
    
    if (is_initialization) {
        // Load topic and frame parameters (only during initialization)
        nh.param<std::string>("point_cloud_topic", point_cloud_topic, "/zed2i/zed_node/point_cloud/cloud_registered");
        nh.param<std::string>("camera_frame", camera_frame, "zed2i_left_camera_frame");
        
        // General settings with defaults
        nh.param<std::string>("base_link_frame", params.base_link_frame, "base_link");
        nh.param<std::string>("world_frame", params.world_frame, "world");
        nh.param<bool>("enable_ground_filtering", params.enable_ground_filtering, true);
        nh.param<bool>("enable_debug_output", params.enable_debug_output, false);
        
        // Set input frame
        params.input_frame_id = camera_frame;
        
        // Processing parameters with defaults
        nh.param("passthrough_z_min_camera", params.processing_params.passthrough_z_min, 0.2);
        nh.param("passthrough_z_max_camera", params.processing_params.passthrough_z_max, 7.0);
        nh.param("voxel_leaf_size", params.processing_params.voxel_leaf_size, 0.08);
        nh.param("enable_uniform_downsampling", params.processing_params.enable_uniform_downsampling, false);
        nh.param("uniform_sampling_radius", params.processing_params.uniform_sampling_radius, 0.08);
        nh.param("enable_early_exit", params.processing_params.enable_early_exit, true);
        nh.param("min_points_for_processing", params.processing_params.min_points_for_processing, 200);
        nh.param("max_points_for_processing", params.processing_params.max_points_for_processing, 15000);
        
        // Ground detection parameters with defaults
        nh.param("ground_filter_distance_threshold", params.ground_params.distance_threshold, 0.08);
        nh.param("ground_filter_angle_threshold_deg", params.ground_params.angle_threshold_deg, 15.0);
        nh.param("ground_filter_max_iterations", params.ground_params.max_iterations, 200);
        nh.param("mars_terrain_mode", params.ground_params.mars_terrain_mode, false);
        
        // Clustering parameters with defaults
        nh.param("cluster_tolerance", params.cluster_params.cluster_tolerance, 0.3);
        nh.param("min_cluster_size", params.cluster_params.min_cluster_size, 20);
        nh.param("max_cluster_size", params.cluster_params.max_cluster_size, 15000);
        
        // Transform parameters with defaults
        nh.param("tf_lookup_timeout", params.transform_params.tf_lookup_timeout, 0.05);
        nh.param("tf_buffer_duration", params.transform_params.tf_buffer_duration, 5.0);
        nh.param("enable_transformations", params.transform_params.enable_transformations, true);
        nh.param<std::string>("camera_frame", params.transform_params.camera_frame, camera_frame);
        
        // Set transform frame references
        params.transform_params.source_frame = camera_frame;
        params.transform_params.target_frame = params.base_link_frame;
        params.transform_params.world_frame = params.world_frame;
        params.transform_params.enable_debug_output = params.enable_debug_output;
        
        // Tracking parameters with defaults
        double assoc_dist;
        nh.param("obstacle_association_distance", assoc_dist, 1.5);
        params.tracking_params.association_distance_sq = assoc_dist * assoc_dist;
        nh.param("obstacle_timeout_sec", params.tracking_params.timeout_sec, 30.0);
        nh.param("position_smoothing_factor", params.tracking_params.position_smoothing_factor, 0.2);
        nh.param("min_detections_for_confirmation", params.tracking_params.min_detections_for_confirmation, 3);
        
        // Monitor parameters with defaults
        nh.param("enable_detailed_timing", params.monitor_params.enable_detailed_timing, false);
        nh.param("enable_debug_publishers", params.monitor_params.enable_debug_publishers, false);
        nh.param("timing_report_interval", params.monitor_params.timing_report_interval, 30.0);
        nh.param("enable_performance_logging", params.monitor_params.enable_performance_logging, true);
        nh.param<std::string>("log_file_path", params.monitor_params.log_file_path, "/tmp/zed_obstacle_detector.log");
        
        // Set derived parameters
        params.cluster_params.enable_debug_output = params.enable_debug_output;
        
    } else {
        // Update existing parameters (preserve current values if not found)
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
        double assoc_dist = sqrt(params.tracking_params.association_distance_sq);
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
        
        // Update derived parameters
        params.cluster_params.enable_debug_output = params.enable_debug_output;
    }
}

void pointCloudCallback(const sensor_msgs::PointCloud2ConstPtr& input_msg) {
    // Reload parameters live
    static ros::NodeHandle private_nh("~");
    static zed_obstacle_detector::ObstacleDetectorParams params = obstacle_detector->getParams();
    static std::string dummy_topic, dummy_frame;  // Not used in update mode
    load_params(private_nh, params, dummy_topic, dummy_frame, false);  // false = update mode
    obstacle_detector->setParams(params);

    // Simple debug print to verify parameters are updating
    static int frame_count = 0;
    frame_count++;
    if (frame_count % 30 == 0) {  // Print every 30 frames
        ROS_INFO("Frame %d: voxel_leaf_size=%.3f, cluster_tolerance=%.3f, min_cluster_size=%d", 
                 frame_count, params.processing_params.voxel_leaf_size, 
                 params.cluster_params.cluster_tolerance, params.cluster_params.min_cluster_size);
    }

    // Print updated parameters (throttled)
    static std::string last_topic, last_frame;  // For print_parameters compatibility
    ROS_INFO_THROTTLE(5.0, "=== Parameter Update ===");
    print_parameters(params, last_topic, last_frame);
    
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

    // Initialize obstacle detector parameters
    zed_obstacle_detector::ObstacleDetectorParams params;
    std::string point_cloud_topic;
    std::string camera_frame;
    
    // Load parameters with defaults for initialization
    load_params(private_nh, params, point_cloud_topic, camera_frame, true);
    
    // Initialize obstacle detector
    obstacle_detector = std::make_unique<zed_obstacle_detector::ObstacleDetector>(params);
    
    if (!obstacle_detector->isInitialized()) {
        ROS_ERROR("Failed to initialize ObstacleDetector. Exiting.");
        return -1;
    }

    // Print configuration
    print_parameters(params, point_cloud_topic, camera_frame);

    // Setup subscribers and publishers
    ros::Subscriber sub = nh.subscribe(point_cloud_topic, 2, pointCloudCallback);
    
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