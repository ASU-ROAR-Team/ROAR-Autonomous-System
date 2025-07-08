#include "zed_obstacle_detector/obstacle_tracker.h"
#include <ros/ros.h>
#include <algorithm>

namespace zed_obstacle_detector {

ObstacleTracker::ObstacleTracker(const TrackingParams& params)
    : params_(params) {
    ROS_DEBUG("ObstacleTracker initialized with association distance: %.2f, timeout: %.1f s",
              std::sqrt(params_.association_distance_sq), params_.timeout_sec);
}

void ObstacleTracker::updateTracks(const std::vector<std::pair<geometry_msgs::Point, float>>& current_detections,
                                  const ros::Time& current_time) {
    // Reset matched flags for all tracked obstacles
    for (auto& obs : tracked_obstacles_) {
        obs.matched_in_current_frame = false;
    }

    // Process each current detection
    for (const auto& detection_pair : current_detections) {
        const geometry_msgs::Point& detected_centroid = detection_pair.first;
        float detected_radius = detection_pair.second;

        // Find best match among tracked obstacles
        int best_match_idx = findBestMatch(detected_centroid, detected_radius);

        if (best_match_idx != -1) {
            // Update existing track
            TrackedObstacle& matched_obs = tracked_obstacles_[best_match_idx];
            matched_obs.last_seen = current_time;
            matched_obs.detection_count++;
            
            // Check if obstacle should be confirmed
            if (!matched_obs.confirmed && matched_obs.detection_count >= params_.min_detections_for_confirmation) {
                matched_obs.confirmed = true;
                ROS_INFO("CONFIRMED obstacle ID %d at world (%.2f, %.2f, %.2f), R: %.2f", 
                        matched_obs.id, matched_obs.position_world.x, matched_obs.position_world.y, 
                        matched_obs.position_world.z, matched_obs.radius_world);
            }
            
            // Smooth radius using exponential moving average
            matched_obs.radius_world = (1.0 - params_.position_smoothing_factor) * matched_obs.radius_world +
                                      params_.position_smoothing_factor * detected_radius;
            matched_obs.matched_in_current_frame = true;
            
        } else {
            // Create new track
            TrackedObstacle new_obs;
            new_obs.id = next_obstacle_id_++;
            new_obs.position_world = detected_centroid;
            new_obs.radius_world = detected_radius;
            new_obs.first_seen = current_time;
            new_obs.last_seen = current_time;
            new_obs.detection_count = 1;
            new_obs.confirmed = (params_.min_detections_for_confirmation <= 1);
            new_obs.matched_in_current_frame = true;
            
            tracked_obstacles_.push_back(new_obs);
            
            ROS_INFO("NEW %s obstacle ID %d at world (%.2f, %.2f, %.2f), R: %.2f",
                     new_obs.confirmed ? "confirmed" : "unconfirmed", new_obs.id,
                     new_obs.position_world.x, new_obs.position_world.y, new_obs.position_world.z, 
                     new_obs.radius_world);
        }
    }

    // Clean up timed out obstacles
    cleanupTimedOutObstacles(current_time);
}

std::vector<TrackedObstacle> ObstacleTracker::getConfirmedObstacles() const {
    std::vector<TrackedObstacle> confirmed_obstacles;
    confirmed_obstacles.reserve(tracked_obstacles_.size());
    
    for (const auto& obs : tracked_obstacles_) {
        if (obs.confirmed) {
            confirmed_obstacles.push_back(obs);
        }
    }
    
    return confirmed_obstacles;
}

void ObstacleTracker::setParams(const TrackingParams& params) {
    params_ = params;
    ROS_DEBUG("ObstacleTracker parameters updated");
}

void ObstacleTracker::cleanupTimedOutObstacles(const ros::Time& current_time) {
    auto it = std::remove_if(tracked_obstacles_.begin(), tracked_obstacles_.end(),
                             [&](const TrackedObstacle& obs) {
                                 bool timed_out = (current_time - obs.last_seen).toSec() > params_.timeout_sec;
                                 if (timed_out) {
                                     ROS_INFO("Obstacle ID %d timed out. Last seen %.2f s ago.", 
                                             obs.id, (current_time - obs.last_seen).toSec());
                                 }
                                 return timed_out;
                             });
    tracked_obstacles_.erase(it, tracked_obstacles_.end());
}

int ObstacleTracker::findBestMatch(const geometry_msgs::Point& detection, float radius) const {
    int best_match_idx = -1;
    double min_dist_sq = params_.association_distance_sq;

    for (size_t i = 0; i < tracked_obstacles_.size(); ++i) {
        if (tracked_obstacles_[i].matched_in_current_frame) {
            continue; // Already matched in this frame
        }

        // Calculate squared distance between detection and tracked obstacle
        double dx = detection.x - tracked_obstacles_[i].position_world.x;
        double dy = detection.y - tracked_obstacles_[i].position_world.y;
        double dz = detection.z - tracked_obstacles_[i].position_world.z;
        double dist_sq = dx * dx + dy * dy + dz * dz;

        if (dist_sq < min_dist_sq) {
            min_dist_sq = dist_sq;
            best_match_idx = i;
        }
    }

    return best_match_idx;
}

} // namespace zed_obstacle_detector 