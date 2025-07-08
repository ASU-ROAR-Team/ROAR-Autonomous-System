// zed_obstacle_detector_node.cpp
// ROS node to detect obstacles from ZED point cloud using PCL with tracking

#include <ros/ros.h>
#include <sensor_msgs/PointCloud2.h>
#include <pcl_ros/point_cloud.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/filters/passthrough.h>
#include <pcl/segmentation/sac_segmentation.h>
#include <pcl/filters/extract_indices.h>
#include <pcl/segmentation/extract_clusters.h>
#include <pcl/filters/uniform_sampling.h>
#include <pcl/common/centroid.h>
#include <pcl/common/geometry.h>

#include <roar_msgs/Obstacle.h>
#include <roar_msgs/ObstacleArray.h>
#include <visualization_msgs/Marker.h>
#include <visualization_msgs/MarkerArray.h>

#include <tf2_ros/transform_listener.h>
#include <tf2_ros/buffer.h>
#include <pcl_ros/transforms.h> // For pcl_ros::transformPointCloud
#include <geometry_msgs/PointStamped.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h> // For tf2::doTransform or tf_buffer->transform

#include <chrono>
#include <Eigen/Dense> // For Eigen::Vector3f
#include <cmath>       // For M_PI, std::sqrt
#include <vector>
#include <algorithm>   // For std::remove_if
#include <mutex>       // For protecting tracked_obstacles_list (not strictly needed with single callback thread)
#include <ctime>       // For srand

// --- Global Variables for Parameters and TF ---
// Publishers
ros::Publisher pub_filtered_transformed;
ros::Publisher pub_no_ground;
ros::Publisher pub_clusters_debug; // Renamed to avoid confusion with final markers
ros::Publisher pub_obstacle_array; // Tracked obstacles in world frame
ros::Publisher pub_markers;        // Tracked obstacle markers in world frame

// TF
std::shared_ptr<tf2_ros::Buffer> tf_buffer_ptr;
std::shared_ptr<tf2_ros::TransformListener> tf_listener_ptr;
std::string camera_frame_; 
std::string world_frame_param_; 

// Parameters (initialized in main)
double passthrough_z_min_camera_; 
double passthrough_z_max_camera_;
bool enable_uniform_downsampling_;
double uniform_sampling_radius_;
double voxel_leaf_size_;
bool enable_ground_filtering_;
double ground_filter_distance_threshold_;
double ground_filter_angle_threshold_deg_;
int ground_filter_max_iterations_;
double cluster_tolerance_;
int min_cluster_size_;
int max_cluster_size_;

// Tracking Parameters
double param_obstacle_association_distance_sq_;
double param_obstacle_timeout_sec_;
double param_position_smoothing_factor_; // For radius smoothing
int param_min_detections_for_confirmation_;

// PERFORMANCE IMPROVEMENT #3: Early exit thresholds
bool enable_early_exit_;
int min_points_for_processing_;
int max_points_for_processing_;

// PERFORMANCE IMPROVEMENT #4: Debug and timing parameters
bool enable_detailed_timing_;
bool enable_debug_publishers_;
double timing_report_interval_;

// PERFORMANCE IMPROVEMENT #5: TF timeout optimization
double tf_lookup_timeout_;
double tf_buffer_duration_;

bool enable_world_transform_;
// --- End Global Variables ---

// --- Tracking Data Structures ---
struct TrackedObstacle {
    int id;
    geometry_msgs::Point position_world; // Fixed position in world_frame_param_
    float radius_world;                 // Radius, can be smoothed
    ros::Time last_seen;
    ros::Time first_seen;
    int detection_count;
    bool confirmed;
    bool matched_in_current_frame; 

    std_msgs::ColorRGBA color;

    TrackedObstacle() : id(0), radius_world(0.0f), detection_count(0), confirmed(false), matched_in_current_frame(false) {
        color.r = 1.0f;
        color.g = 0.0f;
        color.b = 0.0f;
        color.a = 1.0f;
    }
};
std::vector<TrackedObstacle> tracked_obstacles_list_;
int next_static_obstacle_id_ = 1;
// std::mutex tracked_obstacles_mutex_; // If cleanup/access happens in a separate thread

// Performance timing structure
struct ProcessingTiming {
    std::chrono::high_resolution_clock::time_point start_total;
    std::chrono::high_resolution_clock::time_point end_total;
    
    std::chrono::high_resolution_clock::time_point start_input_validation;
    std::chrono::high_resolution_clock::time_point end_input_validation;
    
    std::chrono::high_resolution_clock::time_point start_passthrough;
    std::chrono::high_resolution_clock::time_point end_passthrough;
    
    std::chrono::high_resolution_clock::time_point start_uniform_sampling;
    std::chrono::high_resolution_clock::time_point end_uniform_sampling;
    
    std::chrono::high_resolution_clock::time_point start_voxel_grid;
    std::chrono::high_resolution_clock::time_point end_voxel_grid;
    
    std::chrono::high_resolution_clock::time_point start_transform;
    std::chrono::high_resolution_clock::time_point end_transform;
    
    std::chrono::high_resolution_clock::time_point start_ground_filter;
    std::chrono::high_resolution_clock::time_point end_ground_filter;
    
    std::chrono::high_resolution_clock::time_point start_clustering;
    std::chrono::high_resolution_clock::time_point end_clustering;
    
    std::chrono::high_resolution_clock::time_point start_world_transform;
    std::chrono::high_resolution_clock::time_point end_world_transform;
    
    std::chrono::high_resolution_clock::time_point start_tracking;
    std::chrono::high_resolution_clock::time_point end_tracking;
    
    std::chrono::high_resolution_clock::time_point start_output;
    std::chrono::high_resolution_clock::time_point end_output;
    
    // Helper function to get duration in milliseconds
    template<typename T>
    static long getDurationMs(const T& start, const T& end) {
        return std::chrono::duration_cast<std::chrono::microseconds>(end - start).count() / 1000;
    }
    
    // Helper function to get duration in microseconds
    template<typename T>
    static long getDurationUs(const T& start, const T& end) {
        return std::chrono::duration_cast<std::chrono::microseconds>(end - start).count();
    }
};

// Simple degree to radian conversion
inline double deg2rad(double degrees) {
    return degrees * M_PI / 180.0;
}

void pointCloudCallback(const sensor_msgs::PointCloud2ConstPtr& input)
{
    auto start_total = std::chrono::high_resolution_clock::now();    
    ROS_INFO_THROTTLE(5.0, "Received point cloud. Processing for obstacle tracking...");
    
    if (!tf_buffer_ptr || !tf_listener_ptr) {
        ROS_ERROR_THROTTLE(1.0, "TF buffer or listener not initialized!");
        return;
    }

    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_input_pcl(new pcl::PointCloud<pcl::PointXYZ>);
    pcl::fromROSMsg(*input, *cloud_input_pcl);

    if (cloud_input_pcl->empty()) {
        ROS_WARN_THROTTLE(1.0, "Input PCL cloud is empty!");
        return;
    }

    // 1. PassThrough filter (in original camera frame)
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_passthrough(new pcl::PointCloud<pcl::PointXYZ>);
    pcl::PassThrough<pcl::PointXYZ> pass;
    pass.setInputCloud(cloud_input_pcl);
    pass.setFilterFieldName("z"); 
    pass.setFilterLimits(passthrough_z_min_camera_, passthrough_z_max_camera_);
    pass.filter(*cloud_passthrough);

    if (cloud_passthrough->empty()) {
        ROS_WARN_THROTTLE(1.0, "Cloud empty after PassThrough (camera frame)!");
        return;
    }
    
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_processed_camera_frame = cloud_passthrough;

    // 2. Optional Uniform Downsampling (in original camera frame)
    if (enable_uniform_downsampling_) {
        pcl::UniformSampling<pcl::PointXYZ> uniform;
        uniform.setInputCloud(cloud_passthrough); 
        uniform.setRadiusSearch(uniform_sampling_radius_);
        pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_uniform(new pcl::PointCloud<pcl::PointXYZ>);
        uniform.filter(*cloud_uniform);
        cloud_processed_camera_frame = cloud_uniform; 
    }

    if (cloud_processed_camera_frame->empty()) {
        ROS_WARN_THROTTLE(1.0, "Cloud empty after optional uniform downsampling!");
        return;
    }

    // 3. VoxelGrid downsampling (in original camera frame)
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_downsampled_camera_frame(new pcl::PointCloud<pcl::PointXYZ>);
    pcl::VoxelGrid<pcl::PointXYZ> voxel;
    voxel.setInputCloud(cloud_processed_camera_frame);
    voxel.setLeafSize(voxel_leaf_size_, voxel_leaf_size_, voxel_leaf_size_);
    voxel.filter(*cloud_downsampled_camera_frame);

    if (cloud_downsampled_camera_frame->empty()) {
        ROS_WARN_THROTTLE(1.0, "Cloud empty after VoxelGrid (camera frame)!");
        return;
    }

    // 4. Skip transformation - work directly in camera frame
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_for_processing = cloud_downsampled_camera_frame;
    
    if (cloud_for_processing->empty()) {
        ROS_WARN_THROTTLE(1.0, "Cloud for processing is empty!");
        return;
    }
    
    // Debug publishing of filtered cloud
    sensor_msgs::PointCloud2 output_filtered_transformed_msg;
    pcl::toROSMsg(*cloud_for_processing, output_filtered_transformed_msg);
    output_filtered_transformed_msg.header.stamp = input->header.stamp;
    output_filtered_transformed_msg.header.frame_id = input->header.frame_id; // Use original camera frame
    pub_filtered_transformed.publish(output_filtered_transformed_msg);

    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_for_clustering = cloud_for_processing;

    // 5. Ground Filtering (in camera frame)
    if (enable_ground_filtering_) {
        pcl::SACSegmentation<pcl::PointXYZ> seg;
        seg.setOptimizeCoefficients(true);
        seg.setModelType(pcl::SACMODEL_PERPENDICULAR_PLANE); 
        seg.setMethodType(pcl::SAC_RANSAC);
        seg.setDistanceThreshold(ground_filter_distance_threshold_);
        seg.setAxis(Eigen::Vector3f(0.0, 0.0, 1.0));  
        seg.setEpsAngle(deg2rad(ground_filter_angle_threshold_deg_)); 
        seg.setMaxIterations(ground_filter_max_iterations_);

        pcl::ModelCoefficients::Ptr coefficients(new pcl::ModelCoefficients);
        pcl::PointIndices::Ptr inliers(new pcl::PointIndices);
        seg.setInputCloud(cloud_for_processing); 
        seg.segment(*inliers, *coefficients);

        if (!inliers->indices.empty()) {
            pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_no_ground_ptr(new pcl::PointCloud<pcl::PointXYZ>);
            pcl::ExtractIndices<pcl::PointXYZ> extract;
            extract.setInputCloud(cloud_for_processing);
            extract.setIndices(inliers);
            extract.setNegative(true); 
            extract.filter(*cloud_no_ground_ptr);

            if (!cloud_no_ground_ptr->empty()) {
                cloud_for_clustering = cloud_no_ground_ptr;
                sensor_msgs::PointCloud2 output_no_ground_msg;
                pcl::toROSMsg(*cloud_for_clustering, output_no_ground_msg);
                output_no_ground_msg.header.stamp = input->header.stamp;
                output_no_ground_msg.header.frame_id = input->header.frame_id; // Use original camera frame
                pub_no_ground.publish(output_no_ground_msg);
            }
        }
    }
    
    // --- Tracking Logic ---
    ros::Time current_scan_time = input->header.stamp;

    for (auto& obs : tracked_obstacles_list_) {
        obs.matched_in_current_frame = false;
    }

    std::vector<std::pair<geometry_msgs::Point, float>> current_world_detections; // Pair: <centroid_world, radius>

    if (!cloud_for_clustering->empty()) {
        pcl::search::KdTree<pcl::PointXYZ>::Ptr tree(new pcl::search::KdTree<pcl::PointXYZ>);
        tree->setInputCloud(cloud_for_clustering);

        std::vector<pcl::PointIndices> cluster_indices_vec; // Renamed to avoid conflict
        pcl::EuclideanClusterExtraction<pcl::PointXYZ> ec;
        ec.setClusterTolerance(cluster_tolerance_);
        ec.setMinClusterSize(min_cluster_size_);
        ec.setMaxClusterSize(max_cluster_size_);
        ec.setSearchMethod(tree);
        ec.setInputCloud(cloud_for_clustering);
        ec.extract(cluster_indices_vec);

        if (!cluster_indices_vec.empty()) {
            ROS_DEBUG("Found %ld raw clusters in camera frame.", cluster_indices_vec.size());
            pcl::PointCloud<pcl::PointXYZRGB>::Ptr clustered_cloud_rgb_debug(new pcl::PointCloud<pcl::PointXYZRGB>);

            for (size_t i = 0; i < cluster_indices_vec.size(); ++i) {
                pcl::PointCloud<pcl::PointXYZ> current_cluster_cloud;
                current_cluster_cloud.reserve(cluster_indices_vec[i].indices.size());
                
                // For debug cloud coloring
                uint8_t r_color = (rand() % 200) + 55; 
                uint8_t g_color = (rand() % 200) + 55;
                uint8_t b_color = (rand() % 200) + 55;

                for (const auto& idx : cluster_indices_vec[i].indices) {
                    pcl::PointXYZ pt = cloud_for_clustering->points[idx];
                    current_cluster_cloud.push_back(pt);

                    pcl::PointXYZRGB pt_rgb; // For debug cloud
                    pt_rgb.x = pt.x; pt_rgb.y = pt.y; pt_rgb.z = pt.z;
                    pt_rgb.r = r_color; pt_rgb.g = g_color; pt_rgb.b = b_color;
                    clustered_cloud_rgb_debug->points.push_back(pt_rgb);
                }
                if (current_cluster_cloud.empty()) continue;

                Eigen::Vector4f centroid_camera_eigen;
                pcl::compute3DCentroid(current_cluster_cloud, centroid_camera_eigen);

                float max_radius_sq = 0.0f;
                pcl::PointXYZ centroid_camera_pcl(centroid_camera_eigen[0], centroid_camera_eigen[1], centroid_camera_eigen[2]);
                for (const auto& point : current_cluster_cloud) {
                    float dist_sq = pcl::geometry::squaredDistance(point, centroid_camera_pcl);
                    if (dist_sq > max_radius_sq) {
                        max_radius_sq = dist_sq;
                    }
                }
                float cluster_radius_camera = std::sqrt(max_radius_sq);

                if (enable_world_transform_) {
                    geometry_msgs::PointStamped pt_camera;
                    pt_camera.header.frame_id = input->header.frame_id; // Use original camera frame
                    pt_camera.header.stamp = current_scan_time;
                    pt_camera.point.x = centroid_camera_eigen[0];
                    pt_camera.point.y = centroid_camera_eigen[1];
                    pt_camera.point.z = centroid_camera_eigen[2];

                    geometry_msgs::PointStamped pt_world;
                    try {
                        tf_buffer_ptr->transform(pt_camera, pt_world, world_frame_param_, ros::Duration(0.1)); // Shorter timeout
                        current_world_detections.push_back({pt_world.point, cluster_radius_camera});
                    } catch (tf2::TransformException &ex) {
                        ROS_WARN_THROTTLE(1.0, "TF transform failed for new cluster: %s. From %s to %s. Skipping. Stamp: %.3f, Target Time: %.3f",
                                          ex.what(), input->header.frame_id.c_str(), world_frame_param_.c_str(), pt_camera.header.stamp.toSec(), ros::Time(0).toSec());
                        continue;
                    }
                } else {
                    geometry_msgs::Point centroid_point;
                    centroid_point.x = centroid_camera_eigen[0];
                    centroid_point.y = centroid_camera_eigen[1];
                    centroid_point.z = centroid_camera_eigen[2];
                    current_world_detections.push_back({centroid_point, cluster_radius_camera});
                }
            }
             if(!clustered_cloud_rgb_debug->points.empty()){
                clustered_cloud_rgb_debug->width = clustered_cloud_rgb_debug->points.size();
                clustered_cloud_rgb_debug->height = 1;
                clustered_cloud_rgb_debug->is_dense = true;
                sensor_msgs::PointCloud2 output_clusters_debug_msg;
                pcl::toROSMsg(*clustered_cloud_rgb_debug, output_clusters_debug_msg);
                output_clusters_debug_msg.header.stamp = input->header.stamp;
                output_clusters_debug_msg.header.frame_id = input->header.frame_id; // Use original camera frame
                pub_clusters_debug.publish(output_clusters_debug_msg);
            }
        } else {
             ROS_INFO_THROTTLE(2.0, "No raw clusters found in camera frame.");
        }
    } else {
         ROS_WARN_THROTTLE(1.0, "Cloud for clustering is empty!");
    }

    for (const auto& detection_pair : current_world_detections) {
        const geometry_msgs::Point& detected_centroid_world = detection_pair.first;
        float detected_radius_world = detection_pair.second;

        int best_match_idx = -1;
        double min_dist_sq_to_tracked = param_obstacle_association_distance_sq_;

        for (size_t j = 0; j < tracked_obstacles_list_.size(); ++j) {
            if (tracked_obstacles_list_[j].matched_in_current_frame) continue;

            double dx = detected_centroid_world.x - tracked_obstacles_list_[j].position_world.x;
            double dy = detected_centroid_world.y - tracked_obstacles_list_[j].position_world.y;
            double dz = detected_centroid_world.z - tracked_obstacles_list_[j].position_world.z;
            double dist_sq = dx * dx + dy * dy + dz * dz;

            if (dist_sq < min_dist_sq_to_tracked) {
                min_dist_sq_to_tracked = dist_sq;
                best_match_idx = j;
            }
        }

        if (best_match_idx != -1) { 
            TrackedObstacle& matched_obs = tracked_obstacles_list_[best_match_idx];
            matched_obs.last_seen = current_scan_time;
            matched_obs.detection_count++;
            if (!matched_obs.confirmed && matched_obs.detection_count >= param_min_detections_for_confirmation_) {
                matched_obs.confirmed = true;
                ROS_INFO("CONFIRMED obstacle ID %d at world (%.2f, %.2f, %.2f), R: %.2f", matched_obs.id,
                         matched_obs.position_world.x, matched_obs.position_world.y, matched_obs.position_world.z, matched_obs.radius_world);
            }
            matched_obs.radius_world = (1.0 - param_position_smoothing_factor_) * matched_obs.radius_world +
                                       param_position_smoothing_factor_ * detected_radius_world;
            matched_obs.matched_in_current_frame = true;
        } else { 
            TrackedObstacle new_obs;
            new_obs.id = next_static_obstacle_id_++;
            new_obs.position_world = detected_centroid_world; 
            new_obs.radius_world = detected_radius_world;
            new_obs.first_seen = current_scan_time;
            new_obs.last_seen = current_scan_time;
            new_obs.detection_count = 1;
            new_obs.confirmed = (param_min_detections_for_confirmation_ <= 1);
            new_obs.matched_in_current_frame = true; 
            tracked_obstacles_list_.push_back(new_obs);
            ROS_INFO("NEW %s obstacle ID %d at world (%.2f, %.2f, %.2f), R: %.2f",
                     new_obs.confirmed ? "confirmed" : "unconfirmed", new_obs.id,
                     new_obs.position_world.x, new_obs.position_world.y, new_obs.position_world.z, new_obs.radius_world);
        }
    }

    auto it = std::remove_if(tracked_obstacles_list_.begin(), tracked_obstacles_list_.end(),
                             [&](const TrackedObstacle& obs) {
                                 bool timed_out = (current_scan_time - obs.last_seen).toSec() > param_obstacle_timeout_sec_;
                                 if (timed_out) {
                                     ROS_INFO("Obstacle ID %d timed out. Last seen %.2f s ago.", obs.id, (current_scan_time - obs.last_seen).toSec());
                                 }
                                 return timed_out;
                             });
    tracked_obstacles_list_.erase(it, tracked_obstacles_list_.end());

    roar_msgs::ObstacleArray world_obstacle_array_msg;
    world_obstacle_array_msg.header.stamp = current_scan_time;
    world_obstacle_array_msg.header.frame_id = enable_world_transform_ ? world_frame_param_ : camera_frame_;

    visualization_msgs::MarkerArray world_markers_array;
    visualization_msgs::Marker deleteAllMarker;
    deleteAllMarker.header.stamp = ros::Time::now(); 
    deleteAllMarker.header.frame_id = enable_world_transform_ ? world_frame_param_ : camera_frame_;
    deleteAllMarker.ns = "tracked_world_obstacles"; 
    deleteAllMarker.id = 0; 
    deleteAllMarker.action = visualization_msgs::Marker::DELETEALL;
    world_markers_array.markers.push_back(deleteAllMarker);

    for (const auto& tracked_obs : tracked_obstacles_list_) {
        if (tracked_obs.confirmed) {
            roar_msgs::Obstacle obs_msg;
            obs_msg.header.stamp = tracked_obs.last_seen; 
            obs_msg.header.frame_id = enable_world_transform_ ? world_frame_param_ : camera_frame_;
            obs_msg.id.data = tracked_obs.id;

            obs_msg.position.header.stamp = tracked_obs.last_seen; 
            obs_msg.position.header.frame_id = enable_world_transform_ ? world_frame_param_ : camera_frame_;
            obs_msg.position.pose.position = tracked_obs.position_world;
            obs_msg.position.pose.orientation.w = 1.0; 

            obs_msg.radius.data = tracked_obs.radius_world;
            world_obstacle_array_msg.obstacles.push_back(obs_msg);

            visualization_msgs::Marker marker;
            marker.header.stamp = ros::Time::now(); 
            marker.header.frame_id = enable_world_transform_ ? world_frame_param_ : camera_frame_;
            marker.ns = "tracked_world_obstacles";
            marker.id = tracked_obs.id; 
            marker.type = visualization_msgs::Marker::SPHERE;
            marker.action = visualization_msgs::Marker::ADD;
            marker.pose.position = tracked_obs.position_world;
            marker.pose.orientation.w = 1.0;
            marker.scale.x = 2.0 * tracked_obs.radius_world;
            marker.scale.y = 2.0 * tracked_obs.radius_world;
            marker.scale.z = 2.0 * tracked_obs.radius_world;
            marker.color = tracked_obs.color; 
            marker.lifetime = ros::Duration(param_obstacle_timeout_sec_ + 2.0); // Persist slightly longer than timeout
            world_markers_array.markers.push_back(marker);
        }
    }
    pub_obstacle_array.publish(world_obstacle_array_msg);
    pub_markers.publish(world_markers_array);


    auto end_total = std::chrono::high_resolution_clock::now();
    ROS_INFO_THROTTLE(1.0, "Total processing & tracking time: %ld ms. Tracked Confirmed Obstacles: %zu. Output Frame: %s",
             std::chrono::duration_cast<std::chrono::milliseconds>(end_total - start_total).count(),
             world_obstacle_array_msg.obstacles.size(), (enable_world_transform_ ? world_frame_param_.c_str() : camera_frame_.c_str()));
}

int main(int argc, char** argv)
{
    ros::init(argc, argv, "zed_obstacle_detector");
    ros::NodeHandle nh;
    ros::NodeHandle private_nh("~");

    srand(time(0)); // Seed random number generator for colors

    tf_buffer_ptr = std::make_shared<tf2_ros::Buffer>();
    tf_listener_ptr = std::make_shared<tf2_ros::TransformListener>(*tf_buffer_ptr);

    std::string point_cloud_topic;
    private_nh.param<std::string>("point_cloud_topic", point_cloud_topic, "/zed2i/zed_node/point_cloud/cloud_registered");
    private_nh.param<std::string>("camera_frame", camera_frame_, "zed2i_left_camera_frame");
    private_nh.param<std::string>("world_frame", world_frame_param_, "world");

    private_nh.param("passthrough_z_min_camera", passthrough_z_min_camera_, 0.2); 
    private_nh.param("passthrough_z_max_camera", passthrough_z_max_camera_, 7.0); 
    private_nh.param("enable_uniform_downsampling", enable_uniform_downsampling_, true);
    private_nh.param("uniform_sampling_radius", uniform_sampling_radius_, 0.05);
    private_nh.param("voxel_leaf_size", voxel_leaf_size_, 0.05); 

    private_nh.param("enable_ground_filtering", enable_ground_filtering_, true);
    private_nh.param("ground_filter_distance_threshold", ground_filter_distance_threshold_, 0.05); 
    private_nh.param("ground_filter_angle_threshold_deg", ground_filter_angle_threshold_deg_, 10.0); 
    private_nh.param("ground_filter_max_iterations", ground_filter_max_iterations_, 100);

    private_nh.param("cluster_tolerance", cluster_tolerance_, 0.25); 
    private_nh.param("min_cluster_size", min_cluster_size_, 15);   
    private_nh.param("max_cluster_size", max_cluster_size_, 10000); 

    double assoc_dist;
    private_nh.param("obstacle_association_distance", assoc_dist, 2.0);
    param_obstacle_association_distance_sq_ = assoc_dist * assoc_dist; 
    private_nh.param("obstacle_timeout_sec", param_obstacle_timeout_sec_, 20.0);
    private_nh.param("position_smoothing_factor", param_position_smoothing_factor_, 0.3);
    private_nh.param("min_detections_for_confirmation", param_min_detections_for_confirmation_, 2);

    // PERFORMANCE IMPROVEMENT #3: Early exit thresholds
    private_nh.param("enable_early_exit", enable_early_exit_, false);
    private_nh.param("min_points_for_processing", min_points_for_processing_, 100);
    private_nh.param("max_points_for_processing", max_points_for_processing_, 10000);

    // PERFORMANCE IMPROVEMENT #4: Debug and timing parameters
    private_nh.param("enable_detailed_timing", enable_detailed_timing_, false);
    private_nh.param("enable_debug_publishers", enable_debug_publishers_, false);
    private_nh.param("timing_report_interval", timing_report_interval_, 10.0);

    // PERFORMANCE IMPROVEMENT #5: TF timeout optimization
    private_nh.param("tf_lookup_timeout", tf_lookup_timeout_, 0.1);
    private_nh.param("tf_buffer_duration", tf_buffer_duration_, 10.0);

    private_nh.param("enable_world_transform", enable_world_transform_, true);

    ROS_INFO("--- Obstacle Detector Configuration (Rosbag Mode) ---");
    ROS_INFO("Input Point Cloud Topic: %s", point_cloud_topic.c_str());
    ROS_INFO("Camera Frame (processing): %s", camera_frame_.c_str());
    ROS_INFO("World Frame (tracking & output): %s", world_frame_param_.c_str());
    ROS_INFO("Camera Frame PassThrough Z limits: [%.2f, %.2f] m", passthrough_z_min_camera_, passthrough_z_max_camera_);
    ROS_INFO("Uniform Downsampling: %s, Radius: %.3f m", enable_uniform_downsampling_ ? "Enabled" : "Disabled", uniform_sampling_radius_);
    ROS_INFO("Voxel Leaf Size: %.3f m", voxel_leaf_size_);
    ROS_INFO("Ground Filtering: %s", enable_ground_filtering_ ? "Enabled" : "Disabled");
    if (enable_ground_filtering_) {
        ROS_INFO("  Ground Dist Thresh: %.3f m, Angle Thresh: %.2f deg (around Z-axis of camera), Max Iter: %d",
                 ground_filter_distance_threshold_, ground_filter_angle_threshold_deg_, ground_filter_max_iterations_);
    }
    ROS_INFO("Clustering: Tol: %.2f m, MinSize: %d, MaxSize: %d",
             cluster_tolerance_, min_cluster_size_, max_cluster_size_);
    ROS_INFO("Tracking: Assoc. Dist: %.2f m (%.2f m^2), Timeout: %.1f s, Radius Smooth: %.2f, Min Confirm: %d",
             assoc_dist, param_obstacle_association_distance_sq_, param_obstacle_timeout_sec_,
             param_position_smoothing_factor_, param_min_detections_for_confirmation_);
    ROS_INFO("World Transform Enabled: %s", enable_world_transform_ ? "true" : "false");
    ROS_INFO("NOTE: Working directly in camera frame - no base_link transformation");
    ROS_INFO("------------------------------------");

    ros::Subscriber sub = nh.subscribe("/zed2i/zed_node/point_cloud/cloud_registered", 1, pointCloudCallback);

    pub_filtered_transformed = nh.advertise<sensor_msgs::PointCloud2>("/zed_obstacle/debug/filtered_transformed_pc", 1);
    pub_no_ground = nh.advertise<sensor_msgs::PointCloud2>("/zed_obstacle/debug/pc_no_ground", 1); 
    pub_clusters_debug = nh.advertise<sensor_msgs::PointCloud2>("/zed_obstacle/debug/raw_clusters_rgb", 1); 
    
    pub_obstacle_array = nh.advertise<roar_msgs::ObstacleArray>("/zed_obstacle/obstacle_array", 10); 
    pub_markers = nh.advertise<visualization_msgs::MarkerArray>("/zed_obstacle/markers", 10); 

    ROS_INFO("Obstacle Detector with Tracking initialized (Rosbag Mode). Waiting for point clouds on %s...", point_cloud_topic.c_str());
    ros::spin();
    return 0;
}