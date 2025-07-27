#include "zed_obstacle_detector/cluster_detector.h"
#include <pcl/segmentation/extract_clusters.h>
#include <pcl/search/kdtree.h>
#include <pcl/point_types.h>
#include <ros/ros.h>
#include <random>

namespace zed_obstacle_detector {

ClusterDetector::ClusterDetector(const ClusterParams& params)
    : params_(params), next_cluster_id_(0), last_cloud_size_(0) {
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

    // Start timing if monitor is provided
    if (monitor) {
        monitor->startTimer("clustering");
    }

    // Extract cluster indices with cached KdTree
    std::vector<pcl::PointIndices> cluster_indices = extractClusterIndices(input_cloud);
    
    if (cluster_indices.empty()) {
        ROS_DEBUG_THROTTLE(2.0, "No clusters found in point cloud");
        if (monitor) {
            double duration_ms = monitor->endTimer("clustering");
            ROS_INFO("Cluster detection completed in %.2f ms (no clusters found)", duration_ms);
        }
        return clusters;
    }

    ROS_DEBUG("Found %zu raw clusters", cluster_indices.size());

    // Process clusters in batch for better performance
    clusters.reserve(cluster_indices.size());
    processClustersBatch(cluster_indices, input_cloud, clusters, monitor);
    
    // End timing if monitor is provided and log the result
    if (monitor) {
        double duration_ms = monitor->endTimer("clustering");
        ROS_INFO("Cluster detection completed in %.2f ms (%zu clusters)", duration_ms, clusters.size());
    }
    
    ROS_DEBUG("Processed %zu valid clusters", clusters.size());
    return clusters;
}

std::vector<pcl::PointIndices> ClusterDetector::extractClusterIndices(const pcl::PointCloud<pcl::PointXYZ>::Ptr& input_cloud) {
    std::vector<pcl::PointIndices> cluster_indices;
    
    try {
        // Update cached KdTree if needed
        updateCachedKdTree(input_cloud);

        // Extract clusters with optimized parameters using cached tree
        pcl::EuclideanClusterExtraction<pcl::PointXYZ> ec;
        ec.setClusterTolerance(params_.cluster_tolerance);
        ec.setMinClusterSize(params_.min_cluster_size);
        ec.setMaxClusterSize(params_.max_cluster_size);
        ec.setSearchMethod(cached_tree_);
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
    size_t num_points = cluster_indices.indices.size();
    cluster.points->resize(num_points); // Direct resize instead of reserve + push_back
    
    size_t valid_points = 0;
    for (const auto& idx : cluster_indices.indices) {
        if (idx < input_cloud->size()) {
            cluster.points->points[valid_points++] = input_cloud->points[idx];
        }
    }
    
    // Resize to actual number of valid points
    cluster.points->resize(valid_points);
    cluster.points->width = valid_points;
    cluster.points->height = 1;
    cluster.points->is_dense = true;

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
        
        // Single pass: compute centroid
        for (size_t i = 0; i < num_points; ++i) {
            const auto& point = cluster_points->points[i];
            
            // Update running sums for centroid
            sum_x += point.x;
            sum_y += point.y;
            sum_z += point.z;
        }
        
        // Final centroid
        centroid.x = sum_x / num_points;
        centroid.y = sum_y / num_points;
        centroid.z = sum_z / num_points;
        
        // Second pass: compute radius using final centroid
        float max_radius_sq = 0.0f;
        for (size_t i = 0; i < num_points; ++i) {
            const auto& point = cluster_points->points[i];
            
            // Compute distance from point to final centroid
            float dx = point.x - centroid.x;
            float dy = point.y - centroid.y;
            float dz = point.z - centroid.z;
            float dist_sq = dx*dx + dy*dy + dz*dz;
            
            if (dist_sq > max_radius_sq) {
                max_radius_sq = dist_sq;
            }
        }
        
        radius = std::sqrt(max_radius_sq);
        
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

void ClusterDetector::updateCachedKdTree(const pcl::PointCloud<pcl::PointXYZ>::Ptr& input_cloud) {
    // Check if we need to update the cached tree
    if (!isKdTreeValid(input_cloud)) {
        // Create new KdTree
        cached_tree_.reset(new pcl::search::KdTree<pcl::PointXYZ>);
        cached_tree_->setInputCloud(input_cloud);
        
        // Update cache info
        last_input_cloud_ = input_cloud;
        last_cloud_size_ = input_cloud->size();
        
        ROS_DEBUG("Updated cached KdTree for cloud with %zu points", input_cloud->size());
    }
}

bool ClusterDetector::isKdTreeValid(const pcl::PointCloud<pcl::PointXYZ>::Ptr& input_cloud) const {
    // Check if tree exists and matches current cloud
    return cached_tree_ && 
           last_input_cloud_ && 
           last_input_cloud_ == input_cloud && 
           last_cloud_size_ == input_cloud->size();
}

} // namespace zed_obstacle_detector 