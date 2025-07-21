#ifndef COORDINATE_TRANSFORMER_H
#define COORDINATE_TRANSFORMER_H

#include <pcl/point_types.h>
#include <pcl/point_cloud.h>
#include <pcl_ros/transforms.h>
#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_listener.h>
#include <tf2_eigen/tf2_eigen.h>
#include <geometry_msgs/PointStamped.h>
#include <geometry_msgs/Point.h>
#include <geometry_msgs/TransformStamped.h>
#include <sensor_msgs/PointCloud2.h>
#include <ros/time.h>
#include <chrono>
#include <string>
#include <vector>
#include <memory>
#include <Eigen/Dense>
#include "zed_obstacle_detector/performance_monitor.h"

namespace zed_obstacle_detector {

struct TransformParams {
    std::string source_frame;
    std::string target_frame;
    std::string world_frame;
    double tf_lookup_timeout;
    double tf_buffer_duration;
    bool enable_debug_output;
    bool enable_transformations;
};

class CoordinateTransformer {
public:
    CoordinateTransformer(const TransformParams& params);
    ~CoordinateTransformer() = default;

    // Main transformation interfaces
    bool transformPointCloud(const pcl::PointCloud<pcl::PointXYZ>::Ptr& input_cloud,
                            const std::string& source_frame,
                            const std::string& target_frame,
                            pcl::PointCloud<pcl::PointXYZ>::Ptr& output_cloud,
                            const ros::Time& timestamp,
                            std::shared_ptr<PerformanceMonitor> monitor = nullptr);

    bool transformPointToWorld(const geometry_msgs::Point& point_base_link,
                              const std::string& base_frame,
                              geometry_msgs::Point& point_world,
                              const ros::Time& timestamp);

    bool transformClustersToWorld(const std::vector<std::pair<geometry_msgs::Point, float>>& clusters_base_link,
                                 const std::string& base_frame,
                                 std::vector<std::pair<geometry_msgs::Point, float>>& clusters_world,
                                 const ros::Time& timestamp,
                                 std::shared_ptr<PerformanceMonitor> monitor = nullptr);

    // Individual transformation steps (for testing and debugging)
    bool transformSinglePoint(const geometry_msgs::PointStamped& point_in,
                             geometry_msgs::PointStamped& point_out,
                             const std::string& target_frame);

    // Parameter management
    void setParams(const TransformParams& params);
    TransformParams getParams() const { return params_; }

    // TF buffer access (for external use)
    std::shared_ptr<tf2_ros::Buffer> getTFBuffer() const { return tf_buffer_; }
    std::shared_ptr<tf2_ros::TransformListener> getTFListener() const { return tf_listener_; }

    // Utility functions
    bool isTransformAvailable(const std::string& source_frame,
                             const std::string& target_frame,
                             const ros::Time& timestamp) const;

private:
    TransformParams params_;
    std::shared_ptr<tf2_ros::Buffer> tf_buffer_;
    std::shared_ptr<tf2_ros::TransformListener> tf_listener_;
};

} // namespace zed_obstacle_detector

#endif // COORDINATE_TRANSFORMER_H 