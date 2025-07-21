#include "zed_obstacle_detector/coordinate_transformer.h"
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

bool CoordinateTransformer::transformClustersToWorld(const std::vector<std::pair<geometry_msgs::Point, float>>& clusters_base_link,
                                                    const std::string& base_frame,
                                                    std::vector<std::pair<geometry_msgs::Point, float>>& clusters_world,
                                                    const ros::Time& timestamp,
                                                    std::shared_ptr<PerformanceMonitor> monitor) {
    clusters_world.clear();
    clusters_world.reserve(clusters_base_link.size());

    // Check if transformations are disabled
    if (!params_.enable_transformations) {
        ROS_DEBUG_THROTTLE(5.0, "Transformations disabled, copying clusters without transformation");
        clusters_world = clusters_base_link;  // Copy clusters as-is
        return true;
    }

    // Start timing if monitor is provided
    if (monitor) {
        monitor->startTimer("world_transform");
    }

    // Use iterative transformation for cluster centroids (more efficient for small number of points)
    for (const auto& cluster_pair : clusters_base_link) {
        const geometry_msgs::Point& centroid_base_link = cluster_pair.first;
        float radius_base_link = cluster_pair.second;

        // Create PointStamped for transformation
        geometry_msgs::PointStamped point_in;
        point_in.header.frame_id = base_frame;
        point_in.header.stamp = timestamp;
        point_in.point = centroid_base_link;

        geometry_msgs::PointStamped point_out;
        if (transformSinglePoint(point_in, point_out, params_.world_frame)) {
            clusters_world.push_back({point_out.point, radius_base_link});
        } else {
            ROS_WARN_THROTTLE(1.0, "Failed to transform cluster centroid to world frame");
        }
    }

    // End timing if monitor is provided
    long world_transform_duration = 0;
    if (monitor) {
        world_transform_duration = monitor->endTimer("world_transform");
    }
    
    ROS_DEBUG_THROTTLE(5.0, "Transformed %zu/%zu clusters to world frame in %ld ms (iterative)",
                      clusters_world.size(), clusters_base_link.size(), world_transform_duration);
    
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