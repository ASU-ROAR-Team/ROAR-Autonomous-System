#include "zed_obstacle_detector/point_cloud_processor.h"
#include <pcl/filters/voxel_grid.h>
#include <pcl/filters/passthrough.h>
#include <pcl/filters/uniform_sampling.h>
#include <ros/ros.h>

namespace zed_obstacle_detector {

PointCloudProcessor::PointCloudProcessor(const ProcessingParams& params)
    : params_(params) {
    ROS_DEBUG("PointCloudProcessor initialized with voxel size: %.3f", params_.voxel_leaf_size);
}

bool PointCloudProcessor::processPointCloud(const pcl::PointCloud<pcl::PointXYZ>::Ptr& input_cloud,
                                           pcl::PointCloud<pcl::PointXYZ>::Ptr& output_cloud,
                                           ProcessingTiming& timing) {
    if (!input_cloud || input_cloud->empty()) {
        ROS_WARN_THROTTLE(1.0, "Input cloud is empty or null!");
        return false;
    }

    // Check if we should skip processing
    if (shouldSkipProcessing(input_cloud)) {
        ROS_DEBUG_THROTTLE(5.0, "Skipping processing for small cloud (%zu points)", input_cloud->size());
        return false;
    }

    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_processed = input_cloud;

    // 1. Apply PassThrough filter
    timing.start_passthrough = std::chrono::high_resolution_clock::now();
    if (!applyPassThroughFilter(cloud_processed, cloud_processed)) {
        ROS_WARN_THROTTLE(1.0, "PassThrough filter failed!");
        return false;
    }
    timing.end_passthrough = std::chrono::high_resolution_clock::now();

    if (cloud_processed->empty()) {
        ROS_WARN_THROTTLE(1.0, "Cloud empty after PassThrough filter!");
        return false;
    }

    // 2. Optional Uniform Sampling
    if (params_.enable_uniform_downsampling) {
        timing.start_uniform_sampling = std::chrono::high_resolution_clock::now();
        if (!applyUniformSampling(cloud_processed, cloud_processed)) {
            ROS_WARN_THROTTLE(1.0, "Uniform sampling failed!");
            return false;
        }
        timing.end_uniform_sampling = std::chrono::high_resolution_clock::now();

        if (cloud_processed->empty()) {
            ROS_WARN_THROTTLE(1.0, "Cloud empty after uniform sampling!");
            return false;
        }
    }

    // 3. Apply VoxelGrid filter
    timing.start_voxel_grid = std::chrono::high_resolution_clock::now();
    if (!applyVoxelGridFilter(cloud_processed, output_cloud)) {
        ROS_WARN_THROTTLE(1.0, "VoxelGrid filter failed!");
        return false;
    }
    timing.end_voxel_grid = std::chrono::high_resolution_clock::now();

    if (output_cloud->empty()) {
        ROS_WARN_THROTTLE(1.0, "Cloud empty after VoxelGrid filter!");
        return false;
    }

    ROS_DEBUG_THROTTLE(5.0, "PointCloud processing complete: %zu -> %zu points", 
                      input_cloud->size(), output_cloud->size());
    return true;
}

bool PointCloudProcessor::applyPassThroughFilter(const pcl::PointCloud<pcl::PointXYZ>::Ptr& input_cloud,
                                                pcl::PointCloud<pcl::PointXYZ>::Ptr& output_cloud) {
    try {
        pcl::PassThrough<pcl::PointXYZ> pass;
        pass.setInputCloud(input_cloud);
        pass.setFilterFieldName("z");
        pass.setFilterLimits(params_.passthrough_z_min, params_.passthrough_z_max);
        pass.filter(*output_cloud);
        return true;
    } catch (const std::exception& e) {
        ROS_ERROR_THROTTLE(1.0, "PassThrough filter exception: %s", e.what());
        return false;
    }
}

bool PointCloudProcessor::applyUniformSampling(const pcl::PointCloud<pcl::PointXYZ>::Ptr& input_cloud,
                                              pcl::PointCloud<pcl::PointXYZ>::Ptr& output_cloud) {
    try {
        pcl::UniformSampling<pcl::PointXYZ> uniform;
        uniform.setInputCloud(input_cloud);
        uniform.setRadiusSearch(params_.uniform_sampling_radius);
        uniform.filter(*output_cloud);
        return true;
    } catch (const std::exception& e) {
        ROS_ERROR_THROTTLE(1.0, "Uniform sampling exception: %s", e.what());
        return false;
    }
}

bool PointCloudProcessor::applyVoxelGridFilter(const pcl::PointCloud<pcl::PointXYZ>::Ptr& input_cloud,
                                              pcl::PointCloud<pcl::PointXYZ>::Ptr& output_cloud) {
    try {
        pcl::VoxelGrid<pcl::PointXYZ> voxel;
        voxel.setInputCloud(input_cloud);
        
        double current_voxel_size = getAdaptiveVoxelSize(input_cloud);
        voxel.setLeafSize(current_voxel_size, current_voxel_size, current_voxel_size);
        voxel.filter(*output_cloud);
        
        if (current_voxel_size > params_.voxel_leaf_size) {
            ROS_DEBUG_THROTTLE(5.0, "Using aggressive downsampling: voxel size %.3f", current_voxel_size);
        }
        
        return true;
    } catch (const std::exception& e) {
        ROS_ERROR_THROTTLE(1.0, "VoxelGrid filter exception: %s", e.what());
        return false;
    }
}

void PointCloudProcessor::setParams(const ProcessingParams& params) {
    params_ = params;
    ROS_DEBUG("PointCloudProcessor parameters updated");
}

bool PointCloudProcessor::shouldSkipProcessing(const pcl::PointCloud<pcl::PointXYZ>::Ptr& cloud) const {
    if (!params_.enable_early_exit) {
        return false;
    }
    return cloud->size() < params_.min_points_for_processing;
}

double PointCloudProcessor::getAdaptiveVoxelSize(const pcl::PointCloud<pcl::PointXYZ>::Ptr& cloud) const {
    if (!params_.enable_early_exit || cloud->size() <= params_.max_points_for_processing) {
        return params_.voxel_leaf_size;
    }
    
    // Use larger voxel size for very large clouds
    return params_.voxel_leaf_size * 2.0;
}

} // namespace zed_obstacle_detector 