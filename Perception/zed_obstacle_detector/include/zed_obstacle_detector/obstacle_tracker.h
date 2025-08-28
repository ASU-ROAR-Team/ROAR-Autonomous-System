#ifndef OBSTACLE_TRACKER_H
#define OBSTACLE_TRACKER_H

#include <geometry_msgs/Point.h>
#include <ros/time.h>
#include <vector>
#include <memory>
#include "zed_obstacle_detector/performance_monitor.h"

namespace zed_obstacle_detector {

struct TrackedObstacle {
    int id;
    geometry_msgs::Point position_world;
    float radius_world;
    ros::Time last_seen;
    ros::Time first_seen;
    int detection_count;
    bool confirmed;
    bool matched_in_current_frame;
    
    TrackedObstacle() : id(0), radius_world(0.0f), detection_count(0), 
                       confirmed(false), matched_in_current_frame(false) {}
};

struct TrackingParams {
    double association_distance_sq;  // Squared distance for association
    double timeout_sec;             // Timeout for obstacles
    double position_smoothing_factor; // Smoothing factor for radius
    int min_detections_for_confirmation; // Min detections to confirm
};

class ObstacleTracker {
public:
    ObstacleTracker(const TrackingParams& params);
    ~ObstacleTracker() = default;

    // Main interface
    void updateTracks(const std::vector<std::pair<geometry_msgs::Point, float>>& current_detections,
                     const ros::Time& current_time,
                     std::shared_ptr<PerformanceMonitor> monitor);
    
    // Get results
    std::vector<TrackedObstacle> getConfirmedObstacles() const;
    std::vector<TrackedObstacle> getAllObstacles() const { return tracked_obstacles_; }
    
    // Set parameters
    void setParams(const TrackingParams& params);
    TrackingParams getParams() const { return params_; }

private:
    void cleanupTimedOutObstacles(const ros::Time& current_time);
    int findBestMatch(const geometry_msgs::Point& detection, float radius) const;
    
    std::vector<TrackedObstacle> tracked_obstacles_;
    TrackingParams params_;
    int next_obstacle_id_ = 1;
};

} // namespace zed_obstacle_detector

#endif // OBSTACLE_TRACKER_H 