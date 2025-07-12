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

namespace zed_obstacle_detector {

struct TransformParams {
    std::string source_frame = "camera_frame";
    std::string target_frame = "base_link";
    std::string world_frame = "world";
    double tf_lookup_timeout = 0.1;
    double tf_buffer_duration = 10.0;
    bool enable_debug_output = false;
    bool enable_transformations = true;  // New parameter to properly disable transformations
};

struct TransformTiming {
    std::chrono::high_resolution_clock::time_point start_transform;
    std::chrono::high_resolution_clock::time_point end_transform;
    std::chrono::high_resolution_clock::time_point start_world_transform;
    std::chrono::high_resolution_clock::time_point end_world_transform;
    
    // Helper function to get duration in milliseconds
    template<typename T>
    static long getDurationMs(const T& start, const T& end) {
        return std::chrono::duration_cast<std::chrono::microseconds>(end - start).count() / 1000;
    }
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
                            TransformTiming& timing);

    bool transformPointToWorld(const geometry_msgs::Point& point_base_link,
                              const std::string& base_frame,
                              geometry_msgs::Point& point_world,
                              const ros::Time& timestamp);

    bool transformClustersToWorld(const std::vector<std::pair<geometry_msgs::Point, float>>& clusters_base_link,
                                 const std::string& base_frame,
                                 std::vector<std::pair<geometry_msgs::Point, float>>& clusters_world,
                                 const ros::Time& timestamp,
                                 TransformTiming& timing);

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