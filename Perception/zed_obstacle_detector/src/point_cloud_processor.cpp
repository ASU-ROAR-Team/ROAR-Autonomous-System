#include "zed_obstacle_detector/point_cloud_processor.h"
#include <pcl/filters/voxel_grid.h>
#include <pcl/filters/passthrough.h>
#include <pcl/filters/uniform_sampling.h>
#include <ros/ros.h>

namespace zed_obstacle_detector {

PointCloudProcessor::PointCloudProcessor(const ProcessingParams& params)
    : params_(params) {
    ROS_INFO("PointCloudProcessor initialized: voxel=%.3f, uniform_sampling=%s, radius=%.3f", 
         params.voxel_leaf_size, 
         params.enable_uniform_downsampling ? "enabled" : "disabled",
         params.uniform_sampling_radius);
}

bool PointCloudProcessor::processPointCloud(const pcl::PointCloud<pcl::PointXYZ>::Ptr& input_cloud,
                                           pcl::PointCloud<pcl::PointXYZ>::Ptr& output_cloud,
                                           std::shared_ptr<PerformanceMonitor> monitor) {
    if (!input_cloud || input_cloud->empty()) {
        ROS_WARN_THROTTLE(1.0, "Input cloud is empty or null!");
        return false;
    }

    // Check if we should skip processing
    if (shouldSkipProcessing(input_cloud)) {
        ROS_DEBUG_THROTTLE(5.0, "Skipping processing for small cloud (%zu points)", input_cloud->size());
        return false;
    }

    // Start timing if monitor is provided
    if (monitor) {
        monitor->startTimer("point_cloud_processing");
    }

    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_processed = input_cloud;

    // 1. Apply PassThrough filter
    if (monitor) {
        monitor->startTimer("passthrough_filter");
    }
    if (!applyPassThroughFilter(cloud_processed, cloud_processed, monitor)) {
        ROS_WARN_THROTTLE(1.0, "PassThrough filter failed!");
        if (monitor) {
            monitor->endTimer("point_cloud_processing");
        }
        return false;
    }
    if (monitor) {
        double duration_ms = monitor->endTimer("passthrough_filter");
        ROS_INFO("PassThrough filter: %.2f ms (%zu -> %zu points)", 
                 duration_ms, input_cloud->size(), cloud_processed->size());
    }

    if (cloud_processed->empty()) {
        ROS_WARN_THROTTLE(1.0, "Cloud empty after PassThrough filter!");
        if (monitor) {
            monitor->endTimer("point_cloud_processing");
        }
        return false;
    }

    // 2. Optional Uniform Sampling
    if (params_.enable_uniform_downsampling) {
        if (monitor) {
            monitor->startTimer("uniform_sampling");
        }
        if (!applyUniformSampling(cloud_processed, cloud_processed, monitor)) {
            ROS_WARN_THROTTLE(1.0, "Uniform sampling failed!");
            if (monitor) {
                monitor->endTimer("point_cloud_processing");
            }
            return false;
        }
        if (monitor) {
            double duration_ms = monitor->endTimer("uniform_sampling");
            ROS_INFO("Uniform sampling: %.2f ms", duration_ms);
        }

        if (cloud_processed->empty()) {
            ROS_WARN_THROTTLE(1.0, "Cloud empty after uniform sampling!");
            if (monitor) {
                monitor->endTimer("point_cloud_processing");
            }
            return false;
        }
    }

    // 3. Apply VoxelGrid filter
    if (monitor) {
        monitor->startTimer("voxel_grid_filter");
    }
    if (!applyVoxelGridFilter(cloud_processed, output_cloud, monitor)) {
        ROS_WARN_THROTTLE(1.0, "VoxelGrid filter failed!");
        if (monitor) {
            monitor->endTimer("point_cloud_processing");
        }
        return false;
    }
    if (monitor) {
        double duration_ms = monitor->endTimer("voxel_grid_filter");
        ROS_INFO("VoxelGrid filter: %.2f ms (%zu -> %zu points)", 
                 duration_ms, cloud_processed->size(), output_cloud->size());
    }

    if (output_cloud->empty()) {
        ROS_WARN_THROTTLE(1.0, "Cloud empty after VoxelGrid filter!");
        if (monitor) {
            monitor->endTimer("point_cloud_processing");
        }
        return false;
    }

    // End timing if monitor is provided
    if (monitor) {
        double duration_ms = monitor->endTimer("point_cloud_processing");
        ROS_INFO("PointCloud processing complete: %.2f ms (%zu -> %zu points)", 
                 duration_ms, input_cloud->size(), output_cloud->size());
    }

    ROS_DEBUG_THROTTLE(5.0, "PointCloud processing complete: %zu -> %zu points", 
                      input_cloud->size(), output_cloud->size());
    return true;
}

bool PointCloudProcessor::applyPassThroughFilter(const pcl::PointCloud<pcl::PointXYZ>::Ptr& input_cloud,
                                                pcl::PointCloud<pcl::PointXYZ>::Ptr& output_cloud,
                                                std::shared_ptr<PerformanceMonitor> monitor) {
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
                                              pcl::PointCloud<pcl::PointXYZ>::Ptr& output_cloud,
                                              std::shared_ptr<PerformanceMonitor> monitor) {
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
                                              pcl::PointCloud<pcl::PointXYZ>::Ptr& output_cloud,
                                              std::shared_ptr<PerformanceMonitor> monitor) {
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