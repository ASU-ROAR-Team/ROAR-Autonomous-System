#ifndef CLUSTER_DETECTOR_H
#define CLUSTER_DETECTOR_H

#include <pcl/point_types.h>
#include <pcl/point_cloud.h>
#include <pcl/PointIndices.h>
#include <pcl/search/kdtree.h>
#include <pcl/segmentation/extract_clusters.h>
#include <geometry_msgs/Point.h>
#include "zed_obstacle_detector/performance_monitor.h"
#include <vector>
#include <memory>

namespace zed_obstacle_detector {

struct ClusterParams {
    double cluster_tolerance;
    int min_cluster_size;
    int max_cluster_size;
    bool enable_debug_output;
};

struct BoundingBox{
    float min_x = std::numeric_limits<float>::max();
    float min_y = std::numeric_limits<float>::max();
    float min_z = std::numeric_limits<float>::max();
    float max_x = std::numeric_limits<float>::lowest();
    float max_y = std::numeric_limits<float>::lowest();
    float max_z = std::numeric_limits<float>::lowest();
};

struct Cluster {
    int id;
    pcl::PointCloud<pcl::PointXYZ>::Ptr points;
    geometry_msgs::Point centroid;
    float radius;
    
    Cluster() : id(-1), points(new pcl::PointCloud<pcl::PointXYZ>), radius(0.0f) {}
};

class ClusterDetector {
public:
    ClusterDetector(const ClusterParams& params);
    ~ClusterDetector() = default;

    // Main interface
    std::vector<Cluster> detectClusters(const pcl::PointCloud<pcl::PointXYZ>::Ptr& input_cloud,
                                       std::shared_ptr<PerformanceMonitor> monitor = nullptr);

    // Set parameters
    void setParams(const ClusterParams& params);
    ClusterParams getParams() const { return params_; }

    // Debug interface
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr createDebugCloud(const std::vector<Cluster>& clusters,
                                                            std::shared_ptr<PerformanceMonitor> monitor = nullptr) const;

private:
    // Cluster extraction
    std::vector<pcl::PointIndices> extractClusterIndices(const pcl::PointCloud<pcl::PointXYZ>::Ptr& input_cloud);
    
    // Optimized cluster processing
    Cluster processCluster(const pcl::PointIndices& cluster_indices,
                          const pcl::PointCloud<pcl::PointXYZ>::Ptr& input_cloud,
                          int cluster_id,
                          std::shared_ptr<PerformanceMonitor> monitor = nullptr);

    // Optimized batch processing
    void processClustersBatch(const std::vector<pcl::PointIndices>& cluster_indices,
                                   const pcl::PointCloud<pcl::PointXYZ>::Ptr& input_cloud,
                             std::vector<Cluster>& clusters,
                             std::shared_ptr<PerformanceMonitor> monitor = nullptr);

    // Single-pass geometry computation (optimized)
    std::pair<geometry_msgs::Point, float> computeCentroidRadius(
        const pcl::PointCloud<pcl::PointXYZ>::Ptr& cluster_points,
        std::shared_ptr<PerformanceMonitor> monitor = nullptr) const;

    ClusterParams params_;
    int next_cluster_id_;
};

} // namespace zed_obstacle_detector

#endif // CLUSTER_DETECTOR_H 