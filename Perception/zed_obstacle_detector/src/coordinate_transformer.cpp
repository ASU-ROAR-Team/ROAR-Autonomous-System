#include "zed_obstacle_detector/coordinate_transformer.h"
#include <pcl_ros/transforms.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#include <ros/ros.h>

namespace zed_obstacle_detector {

CoordinateTransformer::CoordinateTransformer(const TransformParams& params)
    : params_(params) {
    
    // Initialize TF buffer and listener
    tf_buffer_ = std::make_shared<tf2_ros::Buffer>();
    tf_listener_ = std::make_shared<tf2_ros::TransformListener>(*tf_buffer_);
    
    ROS_DEBUG("CoordinateTransformer initialized with source: %s, target: %s, world: %s",
              params_.source_frame.c_str(), params_.target_frame.c_str(), params_.world_frame.c_str());
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

        // Perform transformation
        if (!pcl_ros::transformPointCloud(target_frame, *input_cloud, *output_cloud, *tf_buffer_)) {
            ROS_ERROR_THROTTLE(1.0, "Failed to transform point cloud from %s to %s",
                              source_frame.c_str(), target_frame.c_str());
            return false;
        }

        timing.end_transform = std::chrono::high_resolution_clock::now();
        
        ROS_DEBUG_THROTTLE(5.0, "Transformed point cloud: %s -> %s (%zu points)",
                          source_frame.c_str(), target_frame.c_str(), output_cloud->size());
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

    timing.start_world_transform = std::chrono::high_resolution_clock::now();

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

    timing.end_world_transform = std::chrono::high_resolution_clock::now();
    
    ROS_DEBUG_THROTTLE(5.0, "Transformed %zu/%zu clusters to world frame",
                      clusters_world.size(), clusters_base_link.size());
    
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