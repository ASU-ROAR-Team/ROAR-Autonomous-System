#ifndef GROUND_DETECTOR_H
#define GROUND_DETECTOR_H

#include <pcl/point_types.h>
#include <pcl/point_cloud.h>
#include <pcl/ModelCoefficients.h>
#include <pcl/PointIndices.h>
#include <string>

namespace zed_obstacle_detector {

struct GroundDetectionParams {
    std::string method;  // "ransac", "morphological", or "conditional"
    double distance_threshold;
    double angle_threshold_deg;
    int max_iterations;
    double max_ground_slope_deg;
    double min_obstacle_height;
    bool mars_terrain_mode;
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

    GroundDetectionParams params_;
};

} // namespace zed_obstacle_detector

#endif // GROUND_DETECTOR_H 