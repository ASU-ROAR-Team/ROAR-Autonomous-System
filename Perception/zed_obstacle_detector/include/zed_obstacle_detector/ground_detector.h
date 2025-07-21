#ifndef GROUND_DETECTOR_H
#define GROUND_DETECTOR_H

#include <pcl/point_types.h>
#include <pcl/point_cloud.h>
#include <pcl/ModelCoefficients.h>
#include <pcl/PointIndices.h>
#include <string>

namespace zed_obstacle_detector {

enum class GroundDetectionMethod {
    RANSAC,
    MORPHOLOGICAL,
    CONDITIONAL
};

struct GroundDetectionParams {
    std::string method = "ransac";
    double distance_threshold = 0.05;
    double angle_threshold_deg = 10.0;
    int max_iterations = 150;
    double max_ground_slope_deg = 25.0;
    double min_obstacle_height = 0.15;
    bool mars_terrain_mode = true;
};

class GroundDetector {
public:
    GroundDetector(const GroundDetectionParams& params);
    ~GroundDetector() = default;

    // Main interface
    bool detectGround(const pcl::PointCloud<pcl::PointXYZ>::Ptr& input_cloud,
                     pcl::PointCloud<pcl::PointXYZ>::Ptr& obstacle_cloud);

    // Set parameters
    void setParams(const GroundDetectionParams& params);
    GroundDetectionParams getParams() const { return params_; }

private:
    // Different ground detection methods
    bool detectGroundRANSAC(const pcl::PointCloud<pcl::PointXYZ>::Ptr& input_cloud,
                           pcl::PointCloud<pcl::PointXYZ>::Ptr& obstacle_cloud);
    
    bool detectGroundMorphological(const pcl::PointCloud<pcl::PointXYZ>::Ptr& input_cloud,
                                  pcl::PointCloud<pcl::PointXYZ>::Ptr& obstacle_cloud);
    
    bool detectGroundConditional(const pcl::PointCloud<pcl::PointXYZ>::Ptr& input_cloud,
                                pcl::PointCloud<pcl::PointXYZ>::Ptr& obstacle_cloud);

    // Utility functions
    double deg2rad(double degrees) const;
    GroundDetectionMethod parseMethod(const std::string& method_str) const;

    GroundDetectionParams params_;
};

} // namespace zed_obstacle_detector

#endif // GROUND_DETECTOR_H 