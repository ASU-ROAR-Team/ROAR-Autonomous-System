#ifndef CLUSTER_DETECTOR_H
#define CLUSTER_DETECTOR_H

#include <pcl/point_types.h>
#include <pcl/point_cloud.h>
#include <pcl/segmentation/extract_clusters.h>
#include <pcl/search/kdtree.h>
#include <pcl/common/centroid.h>
#include <pcl/common/geometry.h>
#include <geometry_msgs/Point.h>
#include <chrono>
#include <vector>
#include <utility>

namespace zed_obstacle_detector {

struct ClusterParams {
    double cluster_tolerance = 0.25;
    int min_cluster_size = 15;
    int max_cluster_size = 10000;
    bool enable_debug_output = false;
};

struct ClusterTiming {
    std::chrono::high_resolution_clock::time_point start_clustering;
    std::chrono::high_resolution_clock::time_point end_clustering;
    
    // Helper function to get duration in milliseconds
    template<typename T>
    static long getDurationMs(const T& start, const T& end) {
        return std::chrono::duration_cast<std::chrono::microseconds>(end - start).count() / 1000;
    }
};

struct Cluster {
    pcl::PointCloud<pcl::PointXYZ>::Ptr points;
    geometry_msgs::Point centroid;
    float radius;
    int id;
    
    Cluster() : points(new pcl::PointCloud<pcl::PointXYZ>), radius(0.0f), id(0) {}
};

class ClusterDetector {
public:
    ClusterDetector(const ClusterParams& params);
    ~ClusterDetector() = default;

    // Main clustering interface
    std::vector<Cluster> detectClusters(const pcl::PointCloud<pcl::PointXYZ>::Ptr& input_cloud,
                                       ClusterTiming& timing);

    // Individual processing steps (for testing and debugging)
    std::vector<pcl::PointIndices> extractClusterIndices(const pcl::PointCloud<pcl::PointXYZ>::Ptr& input_cloud);
    Cluster processCluster(const pcl::PointIndices& cluster_indices,
                          const pcl::PointCloud<pcl::PointXYZ>::Ptr& input_cloud,
                          int cluster_id);
    Cluster processClusterOptimized(const pcl::PointIndices& cluster_indices,
                                   const pcl::PointCloud<pcl::PointXYZ>::Ptr& input_cloud,
                                   int cluster_id);

    // Parameter management
    void setParams(const ClusterParams& params);
    ClusterParams getParams() const { return params_; }

    // Utility functions
    geometry_msgs::Point computeCentroid(const pcl::PointCloud<pcl::PointXYZ>::Ptr& cluster_points) const;
    float computeRadius(const pcl::PointCloud<pcl::PointXYZ>::Ptr& cluster_points,
                       const geometry_msgs::Point& centroid) const;
    std::pair<geometry_msgs::Point, float> computeCentroidAndRadiusOptimized(
        const pcl::PointCloud<pcl::PointXYZ>::Ptr& cluster_points) const;
    
    // Debug output
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr createDebugCloud(const std::vector<Cluster>& clusters) const;

private:
    ClusterParams params_;
    int next_cluster_id_ = 0;
};

} // namespace zed_obstacle_detector

#endif // CLUSTER_DETECTOR_H 