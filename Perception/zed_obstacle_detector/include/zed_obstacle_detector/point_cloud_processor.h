#ifndef POINT_CLOUD_PROCESSOR_H
#define POINT_CLOUD_PROCESSOR_H

#include <pcl/point_types.h>
#include <pcl/point_cloud.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/filters/passthrough.h>
#include <pcl/filters/uniform_sampling.h>
#include <chrono>
#include <string>

namespace zed_obstacle_detector {

struct ProcessingParams {
    // PassThrough filter parameters
    double passthrough_z_min;
    double passthrough_z_max;
    
    // Voxel grid parameters
    double voxel_leaf_size;
    
    // Uniform sampling parameters (optional)
    bool enable_uniform_downsampling;
    double uniform_sampling_radius;
    
    // Performance thresholds
    bool enable_early_exit;
    int min_points_for_processing;
    int max_points_for_processing;
    
    // Frame information
    std::string input_frame_id;
    std::string target_frame_id;
};

struct ProcessingTiming {
    std::chrono::high_resolution_clock::time_point start_passthrough;
    std::chrono::high_resolution_clock::time_point end_passthrough;
    std::chrono::high_resolution_clock::time_point start_uniform_sampling;
    std::chrono::high_resolution_clock::time_point end_uniform_sampling;
    std::chrono::high_resolution_clock::time_point start_voxel_grid;
    std::chrono::high_resolution_clock::time_point end_voxel_grid;
    
    // Helper function to get duration in milliseconds
    template<typename T>
    static long getDurationMs(const T& start, const T& end) {
        return std::chrono::duration_cast<std::chrono::microseconds>(end - start).count() / 1000;
    }
};

class PointCloudProcessor {
public:
    PointCloudProcessor(const ProcessingParams& params);
    ~PointCloudProcessor() = default;

    // Main processing interface
    bool processPointCloud(const pcl::PointCloud<pcl::PointXYZ>::Ptr& input_cloud,
                          pcl::PointCloud<pcl::PointXYZ>::Ptr& output_cloud,
                          ProcessingTiming& timing);

    // Individual processing steps (for testing and debugging)
    bool applyPassThroughFilter(const pcl::PointCloud<pcl::PointXYZ>::Ptr& input_cloud,
                               pcl::PointCloud<pcl::PointXYZ>::Ptr& output_cloud);
    
    bool applyUniformSampling(const pcl::PointCloud<pcl::PointXYZ>::Ptr& input_cloud,
                             pcl::PointCloud<pcl::PointXYZ>::Ptr& output_cloud);
    
    bool applyVoxelGridFilter(const pcl::PointCloud<pcl::PointXYZ>::Ptr& input_cloud,
                             pcl::PointCloud<pcl::PointXYZ>::Ptr& output_cloud);

    // Parameter management
    void setParams(const ProcessingParams& params);
    ProcessingParams getParams() const { return params_; }

    // Utility functions
    bool shouldSkipProcessing(const pcl::PointCloud<pcl::PointXYZ>::Ptr& cloud) const;
    double getAdaptiveVoxelSize(const pcl::PointCloud<pcl::PointXYZ>::Ptr& cloud) const;

private:
    ProcessingParams params_;
};

} // namespace zed_obstacle_detector

#endif // POINT_CLOUD_PROCESSOR_H 