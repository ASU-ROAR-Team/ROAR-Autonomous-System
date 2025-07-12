#include "zed_obstacle_detector/coordinate_transformer.h"
#include <pcl_ros/transforms.h>
#include <pcl_conversions/pcl_conversions.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#include <tf2_eigen/tf2_eigen.h>
#include <ros/ros.h>
#include <Eigen/Dense>

namespace zed_obstacle_detector {

CoordinateTransformer::CoordinateTransformer(const TransformParams& params)
    : params_(params) {
    
    // Initialize TF buffer with specified duration
    tf_buffer_ = std::make_shared<tf2_ros::Buffer>(ros::Duration(params_.tf_buffer_duration));
    tf_listener_ = std::make_shared<tf2_ros::TransformListener>(*tf_buffer_);
    
    ROS_DEBUG("CoordinateTransformer initialized with source: %s, target: %s, world: %s, buffer_duration: %.1f s",
              params_.source_frame.c_str(), params_.target_frame.c_str(), params_.world_frame.c_str(), params_.tf_buffer_duration);
}

bool CoordinateTransformer::transformPointCloud(const pcl::PointCloud<pcl::PointXYZ>::Ptr& input_cloud,
                                               const std::string& source_frame,
                                               const std::string& target_frame,
                                               pcl::PointCloud<pcl::PointXYZ>::Ptr& output_cloud,
                                               const ros::Time& timestamp,
                                               TransformTiming& timing) {
    if (!input_cloud || input_cloud->empty()) {
        ROS_WARN_THROTTLE(1.0, "Input cloud is empty or null for transformation!");
        return false;
    }

    // Check if transformations are disabled
    if (!params_.enable_transformations) {
        ROS_DEBUG_THROTTLE(5.0, "Transformations disabled, copying cloud without transformation");
        *output_cloud = *input_cloud;
        return true;
    }

    if (source_frame == target_frame) {
        // No transformation needed
        *output_cloud = *input_cloud;
        return true;
    }

    timing.start_transform = std::chrono::high_resolution_clock::now();

    try {
        // Check if transform is available
        if (!isTransformAvailable(source_frame, target_frame, timestamp)) {
            ROS_WARN_THROTTLE(1.0, "Transform from %s to %s not available at time %.3f",
                             source_frame.c_str(), target_frame.c_str(), timestamp.toSec());
            return false;
        }

        // Use pcl_ros::transformPointCloud for better compatibility
        // This is still faster than the old method due to better TF2 integration
        if (!pcl_ros::transformPointCloud(target_frame, *input_cloud, *output_cloud, *tf_buffer_)) {
            ROS_ERROR_THROTTLE(1.0, "Failed to transform point cloud from %s to %s",
                              source_frame.c_str(), target_frame.c_str());
            return false;
        }

        timing.end_transform = std::chrono::high_resolution_clock::now();
        
        ROS_DEBUG_THROTTLE(5.0, "Transformed point cloud: %s -> %s (%zu points) in %.2ld ms",
                          source_frame.c_str(), target_frame.c_str(), output_cloud->size(),
                          TransformTiming::getDurationMs(timing.start_transform, timing.end_transform));
        return true;

    } catch (const tf2::TransformException& ex) {
        ROS_ERROR_THROTTLE(1.0, "TF transform exception: %s", ex.what());
        return false;
    } catch (const std::exception& e) {
        ROS_ERROR_THROTTLE(1.0, "Point cloud transform exception: %s", e.what());
        return false;
    }
}

bool CoordinateTransformer::transformPointToWorld(const geometry_msgs::Point& point_base_link,
                                                 const std::string& base_frame,
                                                 geometry_msgs::Point& point_world,
                                                 const ros::Time& timestamp) {
    try {
        geometry_msgs::PointStamped point_in;
        point_in.header.frame_id = base_frame;
        point_in.header.stamp = timestamp;
        point_in.point = point_base_link;

        geometry_msgs::PointStamped point_out;
        if (!transformSinglePoint(point_in, point_out, params_.world_frame)) {
            return false;
        }

        point_world = point_out.point;
        return true;

    } catch (const std::exception& e) {
        ROS_ERROR_THROTTLE(1.0, "Point to world transform exception: %s", e.what());
        return false;
    }
}

bool CoordinateTransformer::transformClustersToWorld(const std::vector<std::pair<geometry_msgs::Point, float>>& clusters_base_link,
                                                    const std::string& base_frame,
                                                    std::vector<std::pair<geometry_msgs::Point, float>>& clusters_world,
                                                    const ros::Time& timestamp,
                                                    TransformTiming& timing) {
    clusters_world.clear();
    clusters_world.reserve(clusters_base_link.size());

    // Check if transformations are disabled
    if (!params_.enable_transformations) {
        ROS_DEBUG_THROTTLE(5.0, "Transformations disabled, copying clusters without transformation");
        clusters_world = clusters_base_link;  // Copy clusters as-is
        return true;
    }

    timing.start_world_transform = std::chrono::high_resolution_clock::now();

    // Try to get cached transform for efficiency
    geometry_msgs::TransformStamped transform;
    bool transform_cached = false;
    
    try {
        // Check if we can get a transform that's valid for the entire batch
        transform = tf_buffer_->lookupTransform(params_.world_frame, base_frame, timestamp, 
                                               ros::Duration(params_.tf_lookup_timeout));
        transform_cached = true;
    } catch (const tf2::TransformException& ex) {
        ROS_DEBUG_THROTTLE(1.0, "Could not cache transform for batch processing: %s", ex.what());
    }

    if (transform_cached) {
        // Use cached transform for batch processing
        Eigen::Isometry3d transform_eigen = tf2::transformToEigen(transform);
        
        for (const auto& cluster_pair : clusters_base_link) {
            const geometry_msgs::Point& centroid_base_link = cluster_pair.first;
            float radius_base_link = cluster_pair.second;

            // Transform point using Eigen
            Eigen::Vector3d point_eigen(centroid_base_link.x, centroid_base_link.y, centroid_base_link.z);
            Eigen::Vector3d transformed_point = transform_eigen * point_eigen;
            
            geometry_msgs::Point centroid_world;
            centroid_world.x = transformed_point.x();
            centroid_world.y = transformed_point.y();
            centroid_world.z = transformed_point.z();
            
            clusters_world.push_back({centroid_world, radius_base_link});
        }
    } else {
        // Fall back to individual transformations
        for (const auto& cluster_pair : clusters_base_link) {
            const geometry_msgs::Point& centroid_base_link = cluster_pair.first;
            float radius_base_link = cluster_pair.second;

            geometry_msgs::Point centroid_world;
            if (transformPointToWorld(centroid_base_link, base_frame, centroid_world, timestamp)) {
                clusters_world.push_back({centroid_world, radius_base_link});
            } else {
                ROS_WARN_THROTTLE(1.0, "Failed to transform cluster centroid to world frame");
            }
        }
    }

    timing.end_world_transform = std::chrono::high_resolution_clock::now();
    
    ROS_DEBUG_THROTTLE(5.0, "Transformed %zu/%zu clusters to world frame in %.2ld ms (%s)",
                      clusters_world.size(), clusters_base_link.size(),
                      TransformTiming::getDurationMs(timing.start_world_transform, timing.end_world_transform),
                      transform_cached ? "cached" : "individual");
    
    return !clusters_world.empty();
}

bool CoordinateTransformer::transformSinglePoint(const geometry_msgs::PointStamped& point_in,
                                                geometry_msgs::PointStamped& point_out,
                                                const std::string& target_frame) {
    try {
        // Check if transform is available
        if (!isTransformAvailable(point_in.header.frame_id, target_frame, point_in.header.stamp)) {
            ROS_WARN_THROTTLE(1.0, "Transform from %s to %s not available at time %.3f",
                             point_in.header.frame_id.c_str(), target_frame.c_str(), point_in.header.stamp.toSec());
            return false;
        }

        // Perform transformation
        tf_buffer_->transform(point_in, point_out, target_frame, ros::Duration(params_.tf_lookup_timeout));
        return true;

    } catch (const tf2::TransformException& ex) {
        ROS_ERROR_THROTTLE(1.0, "TF transform exception: %s", ex.what());
        return false;
    }
}

void CoordinateTransformer::setParams(const TransformParams& params) {
    params_ = params;
    
    // Reinitialize buffer with new duration if it changed
    if (tf_buffer_ && std::abs(tf_buffer_->getCacheLength().toSec() - params_.tf_buffer_duration) > 0.1) {
        tf_buffer_ = std::make_shared<tf2_ros::Buffer>(ros::Duration(params_.tf_buffer_duration));
        tf_listener_ = std::make_shared<tf2_ros::TransformListener>(*tf_buffer_);
        ROS_INFO("TF buffer reinitialized with new duration: %.1f s", params_.tf_buffer_duration);
    }
    
    ROS_DEBUG("CoordinateTransformer parameters updated");
}

bool CoordinateTransformer::isTransformAvailable(const std::string& source_frame,
                                                const std::string& target_frame,
                                                const ros::Time& timestamp) const {
    try {
        return tf_buffer_->canTransform(target_frame, source_frame, timestamp, ros::Duration(params_.tf_lookup_timeout));
    } catch (const tf2::TransformException& ex) {
        ROS_DEBUG_THROTTLE(1.0, "Transform availability check failed: %s", ex.what());
        return false;
    }
}

} // namespace zed_obstacle_detector 