#include "zed_obstacle_detector/performance_monitor.h"
#include <ros/ros.h>
#include <fstream>
#include <iomanip>

namespace zed_obstacle_detector {

PerformanceMonitor::PerformanceMonitor(const MonitorParams& params)
    : params_(params) {
    ROS_DEBUG("PerformanceMonitor initialized with detailed timing: %s, debug publishers: %s, timing report interval: %.1f s, performance logging: %s",
              params_.enable_detailed_timing ? "enabled" : "disabled",
              params_.enable_debug_publishers ? "enabled" : "disabled",
              params_.timing_report_interval,
              params_.enable_performance_logging ? "enabled" : "disabled");
}

void PerformanceMonitor::startFrame() {
    frame_start_time_ = std::chrono::high_resolution_clock::now();
    resetMetrics();
}

void PerformanceMonitor::endFrame() {
    frame_end_time_ = std::chrono::high_resolution_clock::now();
    current_metrics_.total_ms = getDurationMs(frame_start_time_, frame_end_time_);
    
    // Update statistics
    total_times_.push_back(current_metrics_.total_ms);
    processing_times_.push_back(current_metrics_.total_ms);
    frame_count_++;
    
    // Keep only last 100 frames for statistics
    if (total_times_.size() > 100) {
        total_times_.erase(total_times_.begin());
        processing_times_.erase(processing_times_.begin());
    }
    
    updateStatistics();
}

void PerformanceMonitor::startTimer(const std::string& stage) {
    active_timers_[stage] = std::chrono::high_resolution_clock::now();
}

long PerformanceMonitor::endTimer(const std::string& stage) {
    auto it = active_timers_.find(stage);
    if (it != active_timers_.end()) {
        auto end_time = std::chrono::high_resolution_clock::now();
        long duration_ms = getDurationMs(it->second, end_time);
        
        // Update corresponding metric
        if (stage == "input_validation") current_metrics_.input_validation_ms = duration_ms;
        else if (stage == "passthrough") current_metrics_.passthrough_ms = duration_ms;
        else if (stage == "uniform_sampling") current_metrics_.uniform_sampling_ms = duration_ms;
        else if (stage == "voxel_grid") current_metrics_.voxel_grid_ms = duration_ms;
        else if (stage == "transform") current_metrics_.transform_ms = duration_ms;
        else if (stage == "ground_filter") current_metrics_.ground_filter_ms = duration_ms;
        else if (stage == "clustering") current_metrics_.clustering_ms = duration_ms;
        else if (stage == "world_transform") current_metrics_.world_transform_ms = duration_ms;
        else if (stage == "tracking") current_metrics_.tracking_ms = duration_ms;
        else if (stage == "output") current_metrics_.output_ms = duration_ms;
        
        active_timers_.erase(it);
        return duration_ms;
    }
    return 0;  // Return 0 if timer not found
}

void PerformanceMonitor::recordInputPoints(size_t count) {
    current_metrics_.input_points = count;
}

void PerformanceMonitor::recordOutputPoints(size_t count) {
    current_metrics_.output_points = count;
}

void PerformanceMonitor::recordClustersDetected(size_t count) {
    current_metrics_.clusters_detected = count;
}

void PerformanceMonitor::recordObstaclesTracked(size_t count) {
    current_metrics_.obstacles_tracked = count;
}

void PerformanceMonitor::recordConfirmedObstacles(size_t count) {
    current_metrics_.confirmed_obstacles = count;
}

void PerformanceMonitor::recordEarlyExit(bool triggered) {
    current_metrics_.early_exit_triggered = triggered;
}

void PerformanceMonitor::recordAggressiveDownsampling(bool used) {
    current_metrics_.aggressive_downsampling_used = used;
}

void PerformanceMonitor::setFrameInfo(const std::string& frame_id, const ros::Time& timestamp) {
    current_metrics_.frame_id = frame_id;
    current_metrics_.timestamp = timestamp;
}

void PerformanceMonitor::logPerformanceMetrics(const PerformanceMetrics& metrics) {
    if (!params_.enable_performance_logging) {
        return;
    }

    // Print summary metrics
    printSummaryMetrics(metrics);
    
    // Print detailed timing if enabled
    if (params_.enable_detailed_timing) {
        printDetailedTiming(metrics);
    }
    
    // Log to file if path is specified
    if (!params_.log_file_path.empty()) {
        logToFile(metrics);
    }
}

void PerformanceMonitor::printDetailedTiming(const PerformanceMetrics& metrics) {
    ROS_INFO_THROTTLE(params_.timing_report_interval,
        "=== DETAILED TIMING BREAKDOWN ===\n"
        "Input Validation: %ld ms\n"
        "Passthrough: %ld ms\n"
        "Uniform Sampling: %ld ms\n"
        "Voxel Grid: %ld ms\n"
        "Transform: %ld ms\n"
        "Ground Filter: %ld ms\n"
        "Clustering: %ld ms\n"
        "World Transform: %ld ms\n"
        "Tracking: %ld ms\n"
        "Output: %ld ms",
        metrics.input_validation_ms,
        metrics.passthrough_ms,
        metrics.uniform_sampling_ms,
        metrics.voxel_grid_ms,
        metrics.transform_ms,
        metrics.ground_filter_ms,
        metrics.clustering_ms,
        metrics.world_transform_ms,
        metrics.tracking_ms,
        metrics.output_ms);
}

void PerformanceMonitor::printSummaryMetrics(const PerformanceMetrics& metrics) {
    ROS_INFO_THROTTLE(1.0, 
        "Total processing time: %ld ms | Confirmed obstacles: %zu | Frame: %s | Input points: %zu",
        metrics.total_ms, metrics.confirmed_obstacles, metrics.frame_id.c_str(), metrics.input_points);
    
    if (metrics.early_exit_triggered) {
        ROS_DEBUG_THROTTLE(5.0, "Early exit triggered for small point cloud");
    }
    
    if (metrics.aggressive_downsampling_used) {
        ROS_DEBUG_THROTTLE(5.0, "Aggressive downsampling used for large point cloud");
    }
}

void PerformanceMonitor::setParams(const MonitorParams& params) {
    params_ = params;
    ROS_DEBUG("PerformanceMonitor parameters updated");
}

PerformanceMetrics PerformanceMonitor::getCurrentMetrics() const {
    return current_metrics_;
}

void PerformanceMonitor::resetMetrics() {
    current_metrics_ = PerformanceMetrics();
    active_timers_.clear();
}

void PerformanceMonitor::updateStatistics() {
    if (total_times_.empty()) {
        return;
    }
    
    // Calculate average processing time
    long total_sum = 0;
    for (long time : total_times_) {
        total_sum += time;
    }
    double avg_time = static_cast<double>(total_sum) / total_times_.size();
    
    // Log statistics periodically
    ROS_DEBUG_THROTTLE(30.0, "Performance stats - Frames: %zu, Avg time: %.2f ms", 
                      frame_count_, avg_time);
}

void PerformanceMonitor::logToFile(const PerformanceMetrics& metrics) {
    try {
        std::ofstream log_file(params_.log_file_path, std::ios::app);
        if (log_file.is_open()) {
            log_file << std::fixed << std::setprecision(3)
                    << metrics.timestamp.toSec() << ","
                    << metrics.total_ms << ","
                    << metrics.input_points << ","
                    << metrics.output_points << ","
                    << metrics.clusters_detected << ","
                    << metrics.obstacles_tracked << ","
                    << metrics.confirmed_obstacles << ","
                    << metrics.input_validation_ms << ","
                    << metrics.passthrough_ms << ","
                    << metrics.uniform_sampling_ms << ","
                    << metrics.voxel_grid_ms << ","
                    << metrics.transform_ms << ","
                    << metrics.ground_filter_ms << ","
                    << metrics.clustering_ms << ","
                    << metrics.world_transform_ms << ","
                    << metrics.tracking_ms << ","
                    << metrics.output_ms << ","
                    << (metrics.early_exit_triggered ? "1" : "0") << ","
                    << (metrics.aggressive_downsampling_used ? "1" : "0") << ","
                    << metrics.frame_id
                    << std::endl;
            log_file.close();
        }
    } catch (const std::exception& e) {
        ROS_WARN_THROTTLE(1.0, "Failed to write performance log: %s", e.what());
    }
}

} // namespace zed_obstacle_detector 