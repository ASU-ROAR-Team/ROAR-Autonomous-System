#ifndef PERFORMANCE_MONITOR_H
#define PERFORMANCE_MONITOR_H

#include <chrono>
#include <string>
#include <vector>
#include <map>
#include <ros/ros.h>

namespace zed_obstacle_detector {

struct PerformanceMetrics {
    // Timing data
    long input_validation_ms = 0;
    long passthrough_ms = 0;
    long uniform_sampling_ms = 0;
    long voxel_grid_ms = 0;
    long transform_ms = 0;
    long ground_filter_ms = 0;
    long clustering_ms = 0;
    long world_transform_ms = 0;
    long tracking_ms = 0;
    long output_ms = 0;
    long total_ms = 0;
    
    // Processing statistics
    size_t input_points = 0;
    size_t output_points = 0;
    size_t clusters_detected = 0;
    size_t obstacles_tracked = 0;
    size_t confirmed_obstacles = 0;
    
    // Performance flags
    bool early_exit_triggered = false;
    bool aggressive_downsampling_used = false;
    
    // Frame information
    std::string frame_id;
    ros::Time timestamp;
};

struct MonitorParams {
    bool enable_detailed_timing = false;
    bool enable_debug_publishers = false;
    double timing_report_interval = 10.0;
    bool enable_performance_logging = true;
    std::string log_file_path = "";
};

class PerformanceMonitor {
public:
    PerformanceMonitor(const MonitorParams& params);
    ~PerformanceMonitor() = default;

    // Main interface
    void startFrame();
    void endFrame();
    
    // Timing measurement
    void startTimer(const std::string& stage);
    void endTimer(const std::string& stage);
    
    // Metrics collection
    void recordInputPoints(size_t count);
    void recordOutputPoints(size_t count);
    void recordClustersDetected(size_t count);
    void recordObstaclesTracked(size_t count);
    void recordConfirmedObstacles(size_t count);
    void recordEarlyExit(bool triggered);
    void recordAggressiveDownsampling(bool used);
    void setFrameInfo(const std::string& frame_id, const ros::Time& timestamp);

    // Reporting
    void logPerformanceMetrics(const PerformanceMetrics& metrics);
    void printDetailedTiming(const PerformanceMetrics& metrics);
    void printSummaryMetrics(const PerformanceMetrics& metrics);
    
    // Parameter management
    void setParams(const MonitorParams& params);
    MonitorParams getParams() const { return params_; }

    // Utility functions
    PerformanceMetrics getCurrentMetrics() const;
    void resetMetrics();
    
    // Helper function to get duration in milliseconds
    template<typename T>
    static long getDurationMs(const T& start, const T& end) {
        return std::chrono::duration_cast<std::chrono::microseconds>(end - start).count() / 1000;
    }

private:
    MonitorParams params_;
    PerformanceMetrics current_metrics_;
    std::map<std::string, std::chrono::high_resolution_clock::time_point> active_timers_;
    std::chrono::high_resolution_clock::time_point frame_start_time_;
    std::chrono::high_resolution_clock::time_point frame_end_time_;
    
    // Statistics tracking
    std::vector<long> total_times_;
    std::vector<long> processing_times_;
    size_t frame_count_ = 0;
    
    void updateStatistics();
    void logToFile(const PerformanceMetrics& metrics);
};

} // namespace zed_obstacle_detector

#endif // PERFORMANCE_MONITOR_H 