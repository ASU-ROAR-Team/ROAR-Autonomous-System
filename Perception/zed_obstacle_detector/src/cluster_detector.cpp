#include "zed_obstacle_detector/cluster_detector.h"
#include <pcl/segmentation/extract_clusters.h>
#include <pcl/search/kdtree.h>
#include <pcl/point_types.h>
#include <ros/ros.h>
#include <random>

namespace zed_obstacle_detector {

ClusterDetector::ClusterDetector(const ClusterParams& params)
    : params_(params), next_cluster_id_(0) {
    ROS_DEBUG("ClusterDetector initialized with tolerance: %.3f, min_size: %d, max_size: %d",
              params_.cluster_tolerance, params_.min_cluster_size, params_.max_cluster_size);
}

std::vector<Cluster> ClusterDetector::detectClusters(const pcl::PointCloud<pcl::PointXYZ>::Ptr& input_cloud,
                                                    std::shared_ptr<PerformanceMonitor> monitor) {
    std::vector<Cluster> clusters;
    
    if (!input_cloud || input_cloud->empty()) {
        ROS_WARN_THROTTLE(1.0, "Input cloud is empty or null for clustering!");
        return clusters;
    }

    // Early exit for very large point clouds (performance optimization)
    if (input_cloud->size() > 50000) {
        ROS_WARN_THROTTLE(1.0, "Point cloud too large for clustering (%zu points), skipping", input_cloud->size());
        return clusters;
    }

    // Timer: Extract cluster indices
    if (monitor) {
        monitor->startTimer("extract_cluster_indices");
    }

    // Extract cluster indices with cached KdTree
    std::vector<pcl::PointIndices> cluster_indices = extractClusterIndices(input_cloud);

    if (monitor) {
        double duration_ms = monitor->endTimer("extract_cluster_indices");
        ROS_INFO("Extract cluster indices: %.2f ms", duration_ms);
    }

    if (cluster_indices.empty()) {
        ROS_DEBUG_THROTTLE(2.0, "No clusters found in point cloud");
        return clusters;
    }

    ROS_DEBUG("Found %zu raw clusters", cluster_indices.size());

    // Timer: Process clusters in batch
    if (monitor) {
        monitor->startTimer("process_clusters_batch");
    }

    // Process clusters in batch for better performance
    processClustersBatch(cluster_indices, input_cloud, clusters, monitor);

    if (monitor) {
        double duration_ms = monitor->endTimer("process_clusters_batch");
        ROS_INFO("Process clusters in batch: %.2f ms", duration_ms);
    }

    
    ROS_DEBUG("Processed %zu valid clusters", clusters.size());
    return clusters;
}

std::vector<pcl::PointIndices> ClusterDetector::extractClusterIndices(const pcl::PointCloud<pcl::PointXYZ>::Ptr& input_cloud) {
    std::vector<pcl::PointIndices> cluster_indices;
    
    try {
        // Extract clusters with optimized parameters (PCL creates KdTree internally)
        pcl::EuclideanClusterExtraction<pcl::PointXYZ> ec;
        ec.setClusterTolerance(params_.cluster_tolerance);
        ec.setMinClusterSize(params_.min_cluster_size);
        ec.setMaxClusterSize(params_.max_cluster_size);
        ec.setInputCloud(input_cloud);
        ec.extract(cluster_indices);
        
    } catch (const std::exception& e) {
        ROS_ERROR_THROTTLE(1.0, "Cluster extraction exception: %s", e.what());
    }
    
    return cluster_indices;
}

void ClusterDetector::processClustersBatch(const std::vector<pcl::PointIndices>& cluster_indices,
                                       const pcl::PointCloud<pcl::PointXYZ>::Ptr& input_cloud,
                                          std::vector<Cluster>& clusters,
                                          std::shared_ptr<PerformanceMonitor> monitor) {
    // Pre-allocate clusters vector
    clusters.clear();
    clusters.reserve(cluster_indices.size());
    
    // Process each cluster with optimized processing
    for (const auto& cluster_idx : cluster_indices) {
        // Early exit for empty clusters
        if (cluster_idx.indices.empty()) {
            continue;
        }
        
        // Early exit for clusters that are too small (already filtered by PCL, but double-check)
        if (cluster_idx.indices.size() < static_cast<size_t>(params_.min_cluster_size)) {
            continue;
    }

        Cluster cluster = processCluster(cluster_idx, input_cloud, next_cluster_id_++, monitor);
        if (!cluster.points->empty()) {
            clusters.push_back(std::move(cluster)); // Use move semantics for better performance
        }
    }
}

Cluster ClusterDetector::processCluster(const pcl::PointIndices& cluster_indices,
                                                const pcl::PointCloud<pcl::PointXYZ>::Ptr& input_cloud,
                                       int cluster_id,
                                       std::shared_ptr<PerformanceMonitor> monitor) {
    Cluster cluster;
    cluster.id = cluster_id;
    
    if (cluster_indices.indices.empty()) {
        return cluster;
    }

    // Optimized point extraction with direct indexing
    pcl::copyPointCloud(*input_cloud, cluster_indices.indices, *cluster.points);

    if (cluster.points->empty()) {
        return cluster;
    }

    // Compute centroid and radius with improved 2-pass logic
    std::tie(cluster.centroid, cluster.radius) = computeCentroidRadius(cluster.points, monitor);

    return cluster;
}

std::pair<geometry_msgs::Point, float> ClusterDetector::computeCentroidRadius(
    const pcl::PointCloud<pcl::PointXYZ>::Ptr& cluster_points,
    std::shared_ptr<PerformanceMonitor> monitor) const {
    
    geometry_msgs::Point centroid;
    float radius = 0.0f;
    
    if (!cluster_points || cluster_points->empty()) {
        return {centroid, radius};
    }

    // Start timing if monitor is provided
    if (monitor) {
        monitor->startTimer("centroid_radius_computation");
    }

    try {
        // First pass: compute centroid
        float sum_x = 0.0f, sum_y = 0.0f, sum_z = 0.0f;
        size_t num_points = cluster_points->size();
        BoundingBox bb;
        
        // Single pass: compute centroid
        for (size_t i = 0; i < num_points; ++i) {
            const auto& point = cluster_points->points[i];
            
            // Update running sums for centroid
            sum_x += point.x;
            sum_y += point.y;
            sum_z += point.z;

            // Update bounding box
            bb.min_x = std::min(bb.min_x, point.x);
            bb.min_y = std::min(bb.min_y, point.y);
            bb.min_z = std::min(bb.min_z, point.z);
            bb.max_x = std::max(bb.max_x, point.x);
            bb.max_y = std::max(bb.max_y, point.y);
            bb.max_z = std::max(bb.max_z, point.z);
        }
        
        // Final centroid
        centroid.x = sum_x / num_points;
        centroid.y = sum_y / num_points;
        centroid.z = sum_z / num_points;

        // Calculate radius using bounding box
        float radius_x = bb.max_x - bb.min_x;
        float radius_y = bb.max_y - bb.min_y;
        float radius_z = bb.max_z - bb.min_z;
        radius = std::max(radius_x, std::max(radius_y, radius_z)) / 2.0f; // Divide by 2 for radius from diameter
        
    } catch (const std::exception& e) {
        ROS_ERROR_THROTTLE(1.0, "Centroid/radius computation exception: %s", e.what());
    }
    
    // End timing if monitor is provided
    if (monitor) {
        double duration_ms = monitor->endTimer("centroid_radius_computation");
        if (cluster_points->size() > 100) { // Only log for clusters with more than 100 points
            ROS_INFO("Centroid/radius computation: %.2f ms for %zu points", duration_ms, cluster_points->size());
        }
    }
    
    return {centroid, radius};
}

pcl::PointCloud<pcl::PointXYZRGB>::Ptr ClusterDetector::createDebugCloud(const std::vector<Cluster>& clusters,
                                                                          std::shared_ptr<PerformanceMonitor> monitor) const {
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr debug_cloud(new pcl::PointCloud<pcl::PointXYZRGB>);
    
    // Early exit if debug is disabled or no clusters
    if (!params_.enable_debug_output || clusters.empty()) {
        return debug_cloud;
    }

    // Start timing if monitor is provided
    if (monitor) {
        monitor->startTimer("debug_cloud_creation");
            }

    // Calculate total points for pre-allocation
    size_t total_points = 0;
    for (const auto& cluster : clusters) {
        if (cluster.points && !cluster.points->empty()) {
            total_points += cluster.points->size();
        }
    }
    
    if (total_points == 0) {
        if (monitor) {
            monitor->endTimer("debug_cloud_creation");
        }
        return debug_cloud;
    }

    // Pre-allocate memory to avoid reallocations
    debug_cloud->points.reserve(total_points);

    // Static random number generator (created once, reused)
    static std::mt19937 gen(std::random_device{}());
    static std::uniform_int_distribution<> color_dist(55, 255);

    // Process clusters with optimized bulk operations
    for (const auto& cluster : clusters) {
        if (!cluster.points || cluster.points->empty()) {
            continue;
        }

        // Generate random color for this cluster (cached)
        uint8_t r = color_dist(gen);
        uint8_t g = color_dist(gen);
        uint8_t b = color_dist(gen);

        // Bulk copy points with color assignment
        size_t start_idx = debug_cloud->points.size();
        debug_cloud->points.resize(start_idx + cluster.points->size());
        
        for (size_t i = 0; i < cluster.points->size(); ++i) {
            const auto& src_point = cluster.points->points[i];
            auto& dst_point = debug_cloud->points[start_idx + i];
            
            // Direct assignment (faster than individual field assignment)
            dst_point.x = src_point.x;
            dst_point.y = src_point.y;
            dst_point.z = src_point.z;
            dst_point.r = r;
            dst_point.g = g;
            dst_point.b = b;
        }
    }

    // Set cloud properties
    debug_cloud->width = debug_cloud->points.size();
    debug_cloud->height = 1;
    debug_cloud->is_dense = true;

    // End timing if monitor is provided and log the result
    if (monitor) {
        double duration_ms = monitor->endTimer("debug_cloud_creation");
        ROS_INFO("Debug cloud creation: %.2f ms for %zu clusters (%zu points)", 
                 duration_ms, clusters.size(), debug_cloud->points.size());
    }

    return debug_cloud;
}

void ClusterDetector::setParams(const ClusterParams& params) {
    params_ = params;
    ROS_DEBUG("ClusterDetector parameters updated");
}

} // namespace zed_obstacle_detector 