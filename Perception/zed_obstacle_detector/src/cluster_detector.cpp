#include "zed_obstacle_detector/cluster_detector.h"
#include <pcl/segmentation/extract_clusters.h>
#include <pcl/search/kdtree.h>
#include <pcl/common/centroid.h>
#include <pcl/common/geometry.h>
#include <pcl/point_types.h>
#include <ros/ros.h>
#include <random>

namespace zed_obstacle_detector {

ClusterDetector::ClusterDetector(const ClusterParams& params)
    : params_(params) {
    ROS_DEBUG("ClusterDetector initialized with tolerance: %.3f, min_size: %d, max_size: %d",
              params_.cluster_tolerance, params_.min_cluster_size, params_.max_cluster_size);
}

std::vector<Cluster> ClusterDetector::detectClusters(const pcl::PointCloud<pcl::PointXYZ>::Ptr& input_cloud,
                                                    ClusterTiming& timing) {
    std::vector<Cluster> clusters;
    
    if (!input_cloud || input_cloud->empty()) {
        ROS_WARN_THROTTLE(1.0, "Input cloud is empty or null for clustering!");
        return clusters;
    }

    timing.start_clustering = std::chrono::high_resolution_clock::now();

    // Extract cluster indices
    std::vector<pcl::PointIndices> cluster_indices = extractClusterIndices(input_cloud);
    
    if (cluster_indices.empty()) {
        ROS_DEBUG_THROTTLE(2.0, "No clusters found in point cloud");
        timing.end_clustering = std::chrono::high_resolution_clock::now();
        return clusters;
    }

    ROS_DEBUG("Found %zu raw clusters", cluster_indices.size());

    // Process each cluster
    clusters.reserve(cluster_indices.size());
    for (const auto& cluster_idx : cluster_indices) {
        Cluster cluster = processCluster(cluster_idx, input_cloud, next_cluster_id_++);
        if (!cluster.points->empty()) {
            clusters.push_back(cluster);
        }
    }

    timing.end_clustering = std::chrono::high_resolution_clock::now();
    
    ROS_DEBUG("Processed %zu valid clusters", clusters.size());
    return clusters;
}

std::vector<pcl::PointIndices> ClusterDetector::extractClusterIndices(const pcl::PointCloud<pcl::PointXYZ>::Ptr& input_cloud) {
    std::vector<pcl::PointIndices> cluster_indices;
    
    try {
        // Create KdTree for search
        pcl::search::KdTree<pcl::PointXYZ>::Ptr tree(new pcl::search::KdTree<pcl::PointXYZ>);
        tree->setInputCloud(input_cloud);

        // Extract clusters
        pcl::EuclideanClusterExtraction<pcl::PointXYZ> ec;
        ec.setClusterTolerance(params_.cluster_tolerance);
        ec.setMinClusterSize(params_.min_cluster_size);
        ec.setMaxClusterSize(params_.max_cluster_size);
        ec.setSearchMethod(tree);
        ec.setInputCloud(input_cloud);
        ec.extract(cluster_indices);
        
    } catch (const std::exception& e) {
        ROS_ERROR_THROTTLE(1.0, "Cluster extraction exception: %s", e.what());
    }
    
    return cluster_indices;
}

Cluster ClusterDetector::processCluster(const pcl::PointIndices& cluster_indices,
                                       const pcl::PointCloud<pcl::PointXYZ>::Ptr& input_cloud,
                                       int cluster_id) {
    Cluster cluster;
    cluster.id = cluster_id;
    
    if (cluster_indices.indices.empty()) {
        return cluster;
    }

    // Extract points for this cluster
    cluster.points->reserve(cluster_indices.indices.size());
    for (const auto& idx : cluster_indices.indices) {
        if (idx < input_cloud->size()) {
            cluster.points->push_back(input_cloud->points[idx]);
        }
    }

    if (cluster.points->empty()) {
        return cluster;
    }

    // Compute centroid
    cluster.centroid = computeCentroid(cluster.points);
    
    // Compute radius
    cluster.radius = computeRadius(cluster.points, cluster.centroid);

    return cluster;
}

geometry_msgs::Point ClusterDetector::computeCentroid(const pcl::PointCloud<pcl::PointXYZ>::Ptr& cluster_points) const {
    geometry_msgs::Point centroid;
    
    if (!cluster_points || cluster_points->empty()) {
        return centroid;
    }

    try {
        Eigen::Vector4f centroid_eigen;
        pcl::compute3DCentroid(*cluster_points, centroid_eigen);
        
        centroid.x = centroid_eigen[0];
        centroid.y = centroid_eigen[1];
        centroid.z = centroid_eigen[2];
        
    } catch (const std::exception& e) {
        ROS_ERROR_THROTTLE(1.0, "Centroid computation exception: %s", e.what());
    }
    
    return centroid;
}

float ClusterDetector::computeRadius(const pcl::PointCloud<pcl::PointXYZ>::Ptr& cluster_points,
                                    const geometry_msgs::Point& centroid) const {
    if (!cluster_points || cluster_points->empty()) {
        return 0.0f;
    }

    float max_radius_sq = 0.0f;
    
    try {
        pcl::PointXYZ centroid_pcl(centroid.x, centroid.y, centroid.z);
        
        for (const auto& point : cluster_points->points) {
            float dist_sq = pcl::geometry::squaredDistance(point, centroid_pcl);
            if (dist_sq > max_radius_sq) {
                max_radius_sq = dist_sq;
            }
        }
        
    } catch (const std::exception& e) {
        ROS_ERROR_THROTTLE(1.0, "Radius computation exception: %s", e.what());
        return 0.0f;
    }
    
    return std::sqrt(max_radius_sq);
}

pcl::PointCloud<pcl::PointXYZRGB>::Ptr ClusterDetector::createDebugCloud(const std::vector<Cluster>& clusters) const {
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr debug_cloud(new pcl::PointCloud<pcl::PointXYZRGB>);
    
    if (!params_.enable_debug_output) {
        return debug_cloud;
    }

    // Random number generator for colors
    std::random_device rd;
    std::mt19937 gen(rd());
    std::uniform_int_distribution<> color_dist(55, 255);

    for (const auto& cluster : clusters) {
        if (!cluster.points || cluster.points->empty()) {
            continue;
        }

        // Generate random color for this cluster
        uint8_t r = color_dist(gen);
        uint8_t g = color_dist(gen);
        uint8_t b = color_dist(gen);

        // Add all points from this cluster with the same color
        for (const auto& point : cluster.points->points) {
            pcl::PointXYZRGB colored_point;
            colored_point.x = point.x;
            colored_point.y = point.y;
            colored_point.z = point.z;
            colored_point.r = r;
            colored_point.g = g;
            colored_point.b = b;
            debug_cloud->points.push_back(colored_point);
        }
    }

    // Set cloud properties
    debug_cloud->width = debug_cloud->points.size();
    debug_cloud->height = 1;
    debug_cloud->is_dense = true;

    return debug_cloud;
}

void ClusterDetector::setParams(const ClusterParams& params) {
    params_ = params;
    ROS_DEBUG("ClusterDetector parameters updated");
}

} // namespace zed_obstacle_detector 