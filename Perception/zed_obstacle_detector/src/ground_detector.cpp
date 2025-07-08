#include "zed_obstacle_detector/ground_detector.h"
#include <pcl/segmentation/sac_segmentation.h>
#include <pcl/filters/extract_indices.h>
#include <pcl/filters/morphological_filter.h>
#include <pcl/filters/conditional_removal.h>
#include <pcl/filters/statistical_outlier_removal.h>
#include <pcl/features/normal_3d.h>
#include <pcl/search/kdtree.h>
#include <ros/console.h>
#include <cmath>

namespace zed_obstacle_detector {

GroundDetector::GroundDetector(const GroundDetectionParams& params) : params_(params) {}

bool GroundDetector::detectGround(const pcl::PointCloud<pcl::PointXYZ>::Ptr& input_cloud,
                                 pcl::PointCloud<pcl::PointXYZ>::Ptr& ground_cloud,
                                 pcl::PointCloud<pcl::PointXYZ>::Ptr& obstacle_cloud) {
    
    if (!input_cloud || input_cloud->empty()) {
        ROS_WARN_THROTTLE(1.0, "Input cloud is empty for ground detection");
        return false;
    }

    GroundDetectionMethod method = parseMethod(params_.method);
    
    switch (method) {
        case GroundDetectionMethod::RANSAC:
            return detectGroundRANSAC(input_cloud, ground_cloud, obstacle_cloud);
        case GroundDetectionMethod::MORPHOLOGICAL:
            return detectGroundMorphological(input_cloud, ground_cloud, obstacle_cloud);
        case GroundDetectionMethod::CONDITIONAL:
            return detectGroundConditional(input_cloud, ground_cloud, obstacle_cloud);
        default:
            ROS_ERROR("Unknown ground detection method: %s", params_.method.c_str());
            return false;
    }
}

bool GroundDetector::detectGroundRANSAC(const pcl::PointCloud<pcl::PointXYZ>::Ptr& input_cloud,
                                       pcl::PointCloud<pcl::PointXYZ>::Ptr& ground_cloud,
                                       pcl::PointCloud<pcl::PointXYZ>::Ptr& obstacle_cloud) {
    
    pcl::SACSegmentation<pcl::PointXYZ> seg;
    seg.setOptimizeCoefficients(true);
    seg.setModelType(pcl::SACMODEL_PERPENDICULAR_PLANE);
    seg.setMethodType(pcl::SAC_RANSAC);
    seg.setDistanceThreshold(params_.distance_threshold);
    seg.setAxis(Eigen::Vector3f(0.0, 0.0, 1.0));
    seg.setEpsAngle(deg2rad(params_.angle_threshold_deg));
    seg.setMaxIterations(params_.max_iterations);

    pcl::ModelCoefficients::Ptr coefficients(new pcl::ModelCoefficients);
    pcl::PointIndices::Ptr inliers(new pcl::PointIndices);
    
    seg.setInputCloud(input_cloud);
    seg.segment(*inliers, *coefficients);

    if (inliers->indices.empty()) {
        ROS_WARN_THROTTLE(1.0, "RANSAC ground detection failed - no ground plane found");
        *obstacle_cloud = *input_cloud; // Treat all points as obstacles
        ground_cloud->clear();
        return false;
    }

    // Extract ground points
    pcl::ExtractIndices<pcl::PointXYZ> extract;
    extract.setInputCloud(input_cloud);
    extract.setIndices(inliers);
    extract.setNegative(false);
    extract.filter(*ground_cloud);

    // Extract obstacle points (non-ground)
    extract.setNegative(true);
    extract.filter(*obstacle_cloud);

    ROS_DEBUG("RANSAC ground detection: %zu ground points, %zu obstacle points", 
              ground_cloud->size(), obstacle_cloud->size());
    
    return true;
}

bool GroundDetector::detectGroundMorphological(const pcl::PointCloud<pcl::PointXYZ>::Ptr& input_cloud,
                                              pcl::PointCloud<pcl::PointXYZ>::Ptr& ground_cloud,
                                              pcl::PointCloud<pcl::PointXYZ>::Ptr& obstacle_cloud) {
    
    // Step 1: Remove statistical outliers
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_filtered(new pcl::PointCloud<pcl::PointXYZ>);
    pcl::StatisticalOutlierRemoval<pcl::PointXYZ> sor;
    sor.setInputCloud(input_cloud);
    sor.setMeanK(50);
    sor.setStddevMulThresh(1.0);
    sor.filter(*cloud_filtered);

    // Step 2: Create a grid-based morphological filter
    // This is a simplified version - in practice you might want to use PCL's morphological filter
    // or implement a more sophisticated algorithm
    
    // For now, we'll use a height-based approach with slope consideration
    double min_z = std::numeric_limits<double>::max();
    double max_z = std::numeric_limits<double>::lowest();
    
    // Find height range
    for (const auto& point : cloud_filtered->points) {
        min_z = std::min(min_z, static_cast<double>(point.z));
        max_z = std::max(max_z, static_cast<double>(point.z));
    }
    
    // Calculate height threshold based on Mars terrain parameters
    double height_threshold = min_z + params_.min_obstacle_height;
    if (params_.mars_terrain_mode) {
        // For Mars, be more conservative with ground detection
        height_threshold = min_z + (max_z - min_z) * 0.3; // 30% of height range
    }
    
    // Step 3: Separate ground and obstacles based on height and slope
    ground_cloud->clear();
    obstacle_cloud->clear();
    
    for (const auto& point : cloud_filtered->points) {
        bool is_ground = true;
        
        // Height check
        if (point.z > height_threshold) {
            is_ground = false;
        }
        
        // Slope check (if Mars terrain mode is enabled)
        if (params_.mars_terrain_mode && !is_ground) {
            // Additional slope-based filtering could be added here
            // For now, we'll use the height-based approach
        }
        
        if (is_ground) {
            ground_cloud->points.push_back(point);
        } else {
            obstacle_cloud->points.push_back(point);
        }
    }
    
    ground_cloud->width = ground_cloud->points.size();
    ground_cloud->height = 1;
    ground_cloud->is_dense = true;
    
    obstacle_cloud->width = obstacle_cloud->points.size();
    obstacle_cloud->height = 1;
    obstacle_cloud->is_dense = true;
    
    ROS_DEBUG("Morphological ground detection: %zu ground points, %zu obstacle points", 
              ground_cloud->size(), obstacle_cloud->size());
    
    return !ground_cloud->empty();
}

bool GroundDetector::detectGroundConditional(const pcl::PointCloud<pcl::PointXYZ>::Ptr& input_cloud,
                                            pcl::PointCloud<pcl::PointXYZ>::Ptr& ground_cloud,
                                            pcl::PointCloud<pcl::PointXYZ>::Ptr& obstacle_cloud) {
    
    // This is a placeholder for conditional Euclidean clustering approach
    // In practice, you would implement a more sophisticated algorithm that:
    // 1. Groups points based on height differences
    // 2. Uses local slope analysis
    // 3. Considers connectivity between points
    
    // For now, fall back to RANSAC
    ROS_WARN("Conditional ground detection not yet implemented, falling back to RANSAC");
    return detectGroundRANSAC(input_cloud, ground_cloud, obstacle_cloud);
}

void GroundDetector::setParams(const GroundDetectionParams& params) {
    params_ = params;
}

double GroundDetector::deg2rad(double degrees) const {
    return degrees * M_PI / 180.0;
}

zed_obstacle_detector::GroundDetectionMethod GroundDetector::parseMethod(const std::string& method_str) const {
    if (method_str == "ransac") return GroundDetectionMethod::RANSAC;
    if (method_str == "morphological") return GroundDetectionMethod::MORPHOLOGICAL;
    if (method_str == "conditional") return GroundDetectionMethod::CONDITIONAL;
    
    ROS_WARN("Unknown ground detection method: %s, defaulting to RANSAC", method_str.c_str());
    return GroundDetectionMethod::RANSAC;
}

} // namespace zed_obstacle_detector 