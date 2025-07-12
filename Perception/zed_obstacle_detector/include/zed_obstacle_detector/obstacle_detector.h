#ifndef OBSTACLE_DETECTOR_H
#define OBSTACLE_DETECTOR_H

#include "point_cloud_processor.h"
#include "ground_detector.h"
#include "cluster_detector.h"
#include "coordinate_transformer.h"
#include "obstacle_tracker.h"
#include "performance_monitor.h"

#include <pcl/point_types.h>
#include <pcl/point_cloud.h>
#include <sensor_msgs/PointCloud2.h>
#include <roar_msgs/ObstacleArray.h>
#include <visualization_msgs/MarkerArray.h>
#include <ros/ros.h>
#include <memory>

namespace zed_obstacle_detector {

struct ObstacleDetectorParams {
    // Component parameters
    ProcessingParams processing_params;
    GroundDetectionParams ground_params;
    ClusterParams cluster_params;
    TransformParams transform_params;
    TrackingParams tracking_params;
    MonitorParams monitor_params;
    
    // General settings
    std::string input_frame_id = "camera_frame";
    std::string base_link_frame = "base_link";
    std::string world_frame = "world";
    bool enable_ground_filtering = true;
    bool enable_debug_output = false;
};

struct ObstacleDetectorResult {
    roar_msgs::ObstacleArray obstacle_array;
    visualization_msgs::MarkerArray markers;
    PerformanceMetrics metrics;
    bool success = false;
    std::string error_message;
    sensor_msgs::PointCloud2 filtered_transformed_cloud; // Debug cloud output
    sensor_msgs::PointCloud2 clusters_debug_cloud; // Colored clusters debug cloud
    sensor_msgs::PointCloud2 no_ground_cloud; // No-ground debug cloud
};

class ObstacleDetector {
public:
    ObstacleDetector(const ObstacleDetectorParams& params);
    ~ObstacleDetector() = default;

    // Main processing interface
    ObstacleDetectorResult processPointCloud(const sensor_msgs::PointCloud2ConstPtr& input_msg);

    // Individual processing stages (for testing and debugging)
    bool preprocessPointCloud(const sensor_msgs::PointCloud2ConstPtr& input_msg,
                             pcl::PointCloud<pcl::PointXYZ>::Ptr& processed_cloud);
    
    bool detectGroundAndObstacles(const pcl::PointCloud<pcl::PointXYZ>::Ptr& input_cloud,
                                 pcl::PointCloud<pcl::PointXYZ>::Ptr& obstacle_cloud);
    
    bool detectClusters(const pcl::PointCloud<pcl::PointXYZ>::Ptr& obstacle_cloud,
                       std::vector<Cluster>& clusters);
    
    bool transformAndTrack(const std::vector<Cluster>& clusters,
                          const ros::Time& timestamp,
                          ObstacleDetectorResult& result);

    // Parameter management
    void setParams(const ObstacleDetectorParams& params);
    ObstacleDetectorParams getParams() const { return params_; }

    // Component access (for testing and external use)
    std::shared_ptr<PointCloudProcessor> getProcessor() const { return processor_; }
    std::shared_ptr<GroundDetector> getGroundDetector() const { return ground_detector_; }
    std::shared_ptr<ClusterDetector> getClusterDetector() const { return cluster_detector_; }
    std::shared_ptr<CoordinateTransformer> getTransformer() const { return transformer_; }
    std::shared_ptr<ObstacleTracker> getTracker() const { return tracker_; }
    std::shared_ptr<PerformanceMonitor> getMonitor() const { return monitor_; }

    // Utility functions
    void reset();
    bool isInitialized() const { return initialized_; }

private:
    ObstacleDetectorParams params_;
    bool initialized_;
    
    // Component instances
    std::shared_ptr<PointCloudProcessor> processor_;
    std::shared_ptr<GroundDetector> ground_detector_;
    std::shared_ptr<ClusterDetector> cluster_detector_;
    std::shared_ptr<CoordinateTransformer> transformer_;
    std::shared_ptr<ObstacleTracker> tracker_;
    std::shared_ptr<PerformanceMonitor> monitor_;
    
    // Internal processing methods
    bool initializeComponents();
    void createMarkers(const std::vector<TrackedObstacle>& obstacles,
                      visualization_msgs::MarkerArray& markers);
    void createObstacleArray(const std::vector<TrackedObstacle>& obstacles,
                           roar_msgs::ObstacleArray& obstacle_array);
};

} // namespace zed_obstacle_detector

#endif // OBSTACLE_DETECTOR_H 