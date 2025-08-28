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

bool CoordinateTransformer::transformClustersToWorld(const std::vector<std::pair<geometry_msgs::Point, float>>& clusters_camera,
                                                    const std::string& camera_frame,
                                                    std::vector<std::pair<geometry_msgs::Point, float>>& clusters_world,
                                                    const ros::Time& timestamp,
                                                    std::shared_ptr<PerformanceMonitor> monitor) {
    clusters_world.clear();
    clusters_world.reserve(clusters_camera.size());

    // Check if transformations are disabled
    if (!params_.enable_transformations) {
        ROS_DEBUG_THROTTLE(5.0, "Transformations disabled, copying clusters without transformation");
        clusters_world = clusters_camera;  // Copy clusters as-is
        return true;
    }

    // Start timing if monitor is provided
    if (monitor) {
        monitor->startTimer("world_transform");
    }

    // Direct transformation: camera frame → world frame (TF2 handles chaining automatically)
    for (const auto& cluster_pair : clusters_camera) {
        const geometry_msgs::Point& centroid_camera = cluster_pair.first;
        float radius_camera = cluster_pair.second;

        // Create PointStamped for transformation
        geometry_msgs::PointStamped point_camera;
        point_camera.header.frame_id = camera_frame;
        point_camera.header.stamp = timestamp;
        point_camera.point = centroid_camera;
            
        geometry_msgs::PointStamped point_world;
        if (transformSinglePoint(point_camera, point_world, params_.world_frame)) {
            clusters_world.push_back({point_world.point, radius_camera});
    } else {
            ROS_WARN_THROTTLE(1.0, "Failed to transform cluster centroid from camera to world frame");
        }
    }

    // End timing if monitor is provided
    long world_transform_duration = 0;
    if (monitor) {
        world_transform_duration = monitor->endTimer("world_transform");
    }
    
    ROS_DEBUG_THROTTLE(5.0, "Transformed %zu/%zu clusters from camera to world frame in %ld ms (direct)",
                      clusters_world.size(), clusters_camera.size(), world_transform_duration);
    
    return !clusters_world.empty();
}

bool CoordinateTransformer::transformSinglePoint(const geometry_msgs::PointStamped& point_in,
                                                geometry_msgs::PointStamped& point_out,
                                                const std::string& target_frame) {
    try {
        // Log input point details
        ROS_DEBUG_THROTTLE(2.0, "Transforming point from %s to %s at time %.3f: (%.3f, %.3f, %.3f)",
                          point_in.header.frame_id.c_str(), target_frame.c_str(), 
                          point_in.header.stamp.toSec(),
                          point_in.point.x, point_in.point.y, point_in.point.z);

        // Check if transform is available
        if (!isTransformAvailable(point_in.header.frame_id, target_frame, point_in.header.stamp)) {
            ROS_WARN_THROTTLE(1.0, "Transform from %s to %s not available at time %.3f",
                             point_in.header.frame_id.c_str(), target_frame.c_str(), point_in.header.stamp.toSec());
            
            // Log TF tree status for debugging
            logTFTreeStatus(point_in.header.frame_id, target_frame, point_in.header.stamp);
            return false;
        }

        // Log transform availability
        ROS_DEBUG_THROTTLE(5.0, "Transform from %s to %s is available at time %.3f",
                          point_in.header.frame_id.c_str(), target_frame.c_str(), point_in.header.stamp.toSec());

        // Perform transformation
        tf_buffer_->transform(point_in, point_out, target_frame, ros::Duration(params_.tf_lookup_timeout));
        
        // Log successful transformation with before/after comparison
        ROS_DEBUG_THROTTLE(2.0, "Transform SUCCESS: (%.3f, %.3f, %.3f) -> (%.3f, %.3f, %.3f) [%s -> %s]",
                          point_in.point.x, point_in.point.y, point_in.point.z,
                          point_out.point.x, point_out.point.y, point_out.point.z,
                          point_in.header.frame_id.c_str(), target_frame.c_str());
        
        return true;

    } catch (const tf2::LookupException& ex) {
        ROS_ERROR_THROTTLE(1.0, "TF lookup exception (frame not found): %s", ex.what());
        logTFTreeStatus(point_in.header.frame_id, target_frame, point_in.header.stamp);
        return false;
    } catch (const tf2::ConnectivityException& ex) {
        ROS_ERROR_THROTTLE(1.0, "TF connectivity exception (no connection): %s", ex.what());
        return false;
    } catch (const tf2::ExtrapolationException& ex) {
        ROS_ERROR_THROTTLE(1.0, "TF extrapolation exception (timestamp too old/new): %s", ex.what());
        logTFTreeStatus(point_in.header.frame_id, target_frame, point_in.header.stamp);
        return false;
    } catch (const tf2::TransformException& ex) {
        ROS_ERROR_THROTTLE(1.0, "TF transform exception (timeout or other): %s", ex.what());
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

void CoordinateTransformer::logTFTreeStatus(const std::string& source_frame,
                                           const std::string& target_frame,
                                           const ros::Time& timestamp) const {
    if (!params_.enable_debug_output) {
        return;
    }

    ROS_WARN_THROTTLE(5.0, "=== TF Tree Debug Info ===");
    
    try {
        // Get all frame IDs in the buffer
        std::vector<std::string> frame_ids;
        tf_buffer_->_getFrameStrings(frame_ids);
        std::string frame_list;
        for (const auto& frame : frame_ids) {
            if (!frame_list.empty()) frame_list += ", ";
            frame_list += frame;
        }
        ROS_WARN_THROTTLE(5.0, "Available frames in TF buffer: %s", frame_list.c_str());
        
        // Check if source frame exists
        if (tf_buffer_->_frameExists(source_frame)) {
            ROS_WARN_THROTTLE(5.0, "Source frame '%s' EXISTS in TF tree", source_frame.c_str());
        } else {
            ROS_WARN_THROTTLE(5.0, "Source frame '%s' NOT FOUND in TF tree", source_frame.c_str());
        }
        
        // Check if target frame exists
        if (tf_buffer_->_frameExists(target_frame)) {
            ROS_WARN_THROTTLE(5.0, "Target frame '%s' EXISTS in TF tree", target_frame.c_str());
        } else {
            ROS_WARN_THROTTLE(5.0, "Target frame '%s' NOT FOUND in TF tree", target_frame.c_str());
        }
        
        // Try to get the latest transform time
        ros::Time latest_time;
        std::string error_string;
        
        // Check if transform is available at the requested time
        bool transform_available = tf_buffer_->canTransform(target_frame, source_frame, timestamp, ros::Duration(0.1));
        if (transform_available) {
            ROS_WARN_THROTTLE(5.0, "Transform available between %s and %s at time %.3f",
                             source_frame.c_str(), target_frame.c_str(), timestamp.toSec());
        } else {
            ROS_WARN_THROTTLE(5.0, "Transform NOT available between %s and %s at time %.3f", 
                             source_frame.c_str(), target_frame.c_str(), timestamp.toSec());
        }
        
        // Check buffer cache length
        ros::Duration cache_length = tf_buffer_->getCacheLength();
        ROS_WARN_THROTTLE(5.0, "TF buffer cache length: %.3f seconds", cache_length.toSec());
        
    } catch (const std::exception& e) {
        ROS_WARN_THROTTLE(5.0, "Error while logging TF tree status: %s", e.what());
    }
    
    ROS_WARN_THROTTLE(5.0, "=== End TF Tree Debug Info ===");
}

} // namespace zed_obstacle_detector 