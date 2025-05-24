// zed_obstacle_detector_node.cpp
// ROS node to detect obstacles from ZED point cloud using PCL (PassThrough + VoxelGrid + RANSAC + Euclidean Clustering)

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
#include <chrono>

// Default parameters
#define DEFAULT_CLUSTER_TOLERANCE 0.2
#define DEFAULT_MIN_CLUSTER_SIZE 50
#define DEFAULT_MAX_CLUSTER_SIZE 25000
#define DEFAULT_GROUND_FILTER_DISTANCE_THRESHOLD 0.05
#define DEFAULT_GROUND_FILTER_ANGLE_THRESHOLD 10.0
#define DEFAULT_GROUND_FILTER_MAX_ITERATIONS 1000
#define DEFAULT_ENABLE_GROUND_FILTERING true
#define DEFAULT_ENABLE_DOWN_SAMPLING true


// Simple degree to radian conversion
inline double deg2rad(double degrees) {
    return degrees * 3.14159265358979323846 / 180.0;
}

ros::Publisher pub_filtered;
ros::Publisher pub_no_ground;
ros::Publisher pub_clusters;
ros::Publisher pub_closest_point;
ros::Publisher pub_obstacle_array;

void pointCloudCallback(const sensor_msgs::PointCloud2ConstPtr& input)
{
    auto start_total = std::chrono::high_resolution_clock::now();
    
    ROS_INFO("Received point cloud with %d points", input->width * input->height);
    ROS_INFO("Frame ID: %s", input->header.frame_id.c_str());
    
    auto start_convert = std::chrono::high_resolution_clock::now();
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZ>);
    pcl::fromROSMsg(*input, *cloud);
    auto end_convert = std::chrono::high_resolution_clock::now();
    auto convert_time = std::chrono::duration_cast<std::chrono::milliseconds>(end_convert - start_convert);
    ROS_INFO("Conversion time: %ld ms", convert_time.count());

    if (cloud->empty()) {
        ROS_WARN("Input cloud is empty!");
        return;
    }

    // 1. PassThrough filter - more lenient range
    auto start_passthrough = std::chrono::high_resolution_clock::now();
    pcl::PassThrough<pcl::PointXYZ> pass;
    pass.setInputCloud(cloud);
    pass.setFilterFieldName("z");
    pass.setFilterLimits(0.0, 3.5);
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_filtered(new pcl::PointCloud<pcl::PointXYZ>);
    pass.filter(*cloud_filtered);
    auto end_passthrough = std::chrono::high_resolution_clock::now();
    auto passthrough_time = std::chrono::duration_cast<std::chrono::milliseconds>(end_passthrough - start_passthrough);
    ROS_INFO("PassThrough time: %ld ms", passthrough_time.count());

    if (cloud_filtered->empty()) {
        ROS_WARN("Filtered cloud is empty after PassThrough!");
        return;
    }

    // 2. Optional uniform downsampling
    if(DEFAULT_ENABLE_DOWN_SAMPLING) {
        // 2.1. Uniform downsampling
        auto start_uniform = std::chrono::high_resolution_clock::now();
        pcl::UniformSampling<pcl::PointXYZ> uniform;
        uniform.setInputCloud(cloud_filtered);
        uniform.setRadiusSearch(0.05f);
        pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_uniform(new pcl::PointCloud<pcl::PointXYZ>);
        uniform.filter(*cloud_uniform);
        auto end_uniform = std::chrono::high_resolution_clock::now();
        auto uniform_time = std::chrono::duration_cast<std::chrono::milliseconds>(end_uniform - start_uniform);
        ROS_INFO("Uniform downsampling time: %ld ms", uniform_time.count());
        
        // Update cloud_filtered to use the uniformly sampled cloud
        cloud_filtered = cloud_uniform;
    }
    
    

    // 3. VoxelGrid downsampling
    auto start_voxel = std::chrono::high_resolution_clock::now();
    pcl::VoxelGrid<pcl::PointXYZ> voxel;
    voxel.setInputCloud(cloud_filtered);
    voxel.setLeafSize(0.05f, 0.05f, 0.05f);
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_downsampled(new pcl::PointCloud<pcl::PointXYZ>);
    voxel.filter(*cloud_downsampled);
    auto end_voxel = std::chrono::high_resolution_clock::now();
    auto voxel_time = std::chrono::duration_cast<std::chrono::milliseconds>(end_voxel - start_voxel);
    ROS_INFO("VoxelGrid time: %ld ms", voxel_time.count());

    if (cloud_downsampled->empty()) {
        ROS_WARN("Downsampled cloud is empty!");
        return;
    }

    // Publish preprocessed point cloud
    sensor_msgs::PointCloud2 output_filtered;
    pcl::toROSMsg(*cloud_downsampled, output_filtered);
    output_filtered.header = input->header;
    ROS_INFO("Publishing filtered cloud with %ld points", cloud_downsampled->size());
    pub_filtered.publish(output_filtered);

    // Get ground filtering parameter
    bool enable_ground_filtering;
    ros::param::param<bool>("~enable_ground_filtering", enable_ground_filtering, true);

    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_no_ground;
    if (enable_ground_filtering) {
        // 4. RANSAC ground removal
        auto start_ransac = std::chrono::high_resolution_clock::now();
        pcl::SACSegmentation<pcl::PointXYZ> seg;
        seg.setOptimizeCoefficients(true);
        seg.setModelType(pcl::SACMODEL_PLANE);
        seg.setMethodType(pcl::SAC_RANSAC);
        seg.setDistanceThreshold(0.05);
        seg.setAxis(Eigen::Vector3f(0.0, 0.0, 1.0));
        seg.setEpsAngle(deg2rad(10.0));
        seg.setMaxIterations(1000);

        pcl::ModelCoefficients::Ptr coefficients(new pcl::ModelCoefficients);
        pcl::PointIndices::Ptr inliers(new pcl::PointIndices);
        seg.setInputCloud(cloud_downsampled);
        seg.segment(*inliers, *coefficients);
        auto end_ransac = std::chrono::high_resolution_clock::now();
        auto ransac_time = std::chrono::duration_cast<std::chrono::milliseconds>(end_ransac - start_ransac);
        ROS_INFO("RANSAC time: %ld ms", ransac_time.count());

        if (inliers->indices.empty()) {
            ROS_WARN("No ground plane found!");
            return;
        }

        // Print ground plane coefficients for debugging
        ROS_INFO("Ground plane coefficients: a=%.3f, b=%.3f, c=%.3f, d=%.3f", 
                 coefficients->values[0], coefficients->values[1], 
                 coefficients->values[2], coefficients->values[3]);

        pcl::ExtractIndices<pcl::PointXYZ> extract;
        extract.setInputCloud(cloud_downsampled);
        extract.setIndices(inliers);
        extract.setNegative(true);
        cloud_no_ground.reset(new pcl::PointCloud<pcl::PointXYZ>);
        extract.filter(*cloud_no_ground);

        if (cloud_no_ground->empty()) {
            ROS_WARN("No points left after ground removal!");
            return;
        }

        // Publish point cloud without ground
        sensor_msgs::PointCloud2 output_no_ground;
        pcl::toROSMsg(*cloud_no_ground, output_no_ground);
        output_no_ground.header = input->header;
        ROS_INFO("Publishing ground-free cloud with %ld points", cloud_no_ground->size());
        pub_no_ground.publish(output_no_ground);
    } else {
        // Skip ground removal, use downsampled cloud directly
        cloud_no_ground = cloud_downsampled;
        ROS_INFO("Ground filtering disabled, using downsampled cloud with %ld points", cloud_no_ground->size());
    }

    // 4. Euclidean Clustering
    auto start_clustering = std::chrono::high_resolution_clock::now();
    pcl::search::KdTree<pcl::PointXYZ>::Ptr tree(new pcl::search::KdTree<pcl::PointXYZ>);
    tree->setInputCloud(cloud_no_ground);

    std::vector<pcl::PointIndices> cluster_indices;
    pcl::EuclideanClusterExtraction<pcl::PointXYZ> ec;
    ec.setClusterTolerance(0.2);
    ec.setMinClusterSize(50);
    ec.setMaxClusterSize(25000);
    ec.setSearchMethod(tree);
    ec.setInputCloud(cloud_no_ground);
    ec.extract(cluster_indices);
    auto end_clustering = std::chrono::high_resolution_clock::now();
    auto clustering_time = std::chrono::duration_cast<std::chrono::milliseconds>(end_clustering - start_clustering);
    ROS_INFO("Clustering time: %ld ms", clustering_time.count());

    if (cluster_indices.empty()) {
        ROS_WARN("No clusters found!");
        return;
    }

    pcl::PointCloud<pcl::PointXYZRGB>::Ptr clustered_cloud(new pcl::PointCloud<pcl::PointXYZRGB>);
    pcl::PointXYZ closest_point;
    double min_distance = std::numeric_limits<double>::max();

    for (size_t i = 0; i < cluster_indices.size(); ++i)
    {
        uint8_t r = rand() % 256;
        uint8_t g = rand() % 256;
        uint8_t b = rand() % 256;

        for (const auto& idx : cluster_indices[i].indices)
        {
            pcl::PointXYZ pt = cloud_no_ground->points[idx];
            pcl::PointXYZRGB pt_rgb;
            pt_rgb.x = pt.x;
            pt_rgb.y = pt.y;
            pt_rgb.z = pt.z;
            pt_rgb.r = r;
            pt_rgb.g = g;
            pt_rgb.b = b;
            clustered_cloud->points.push_back(pt_rgb);

            double dist = std::sqrt(pt.x * pt.x + pt.y * pt.y + pt.z * pt.z);
            if (dist < min_distance)
            {
                min_distance = dist;
                closest_point = pt;
            }
        }
    }

    clustered_cloud->width = clustered_cloud->points.size();
    clustered_cloud->height = 1;
    clustered_cloud->is_dense = true;

    sensor_msgs::PointCloud2 output_clusters;
    pcl::toROSMsg(*clustered_cloud, output_clusters);
    output_clusters.header = input->header;
    ROS_INFO("Publishing %ld clusters with %ld total points", cluster_indices.size(), clustered_cloud->size());
    pub_clusters.publish(output_clusters);

    // Publish closest point
    sensor_msgs::PointCloud2 output_closest;
    pcl::PointCloud<pcl::PointXYZ> closest_cloud;
    closest_cloud.push_back(closest_point);
    pcl::toROSMsg(closest_cloud, output_closest);
    output_closest.header = input->header;
    pub_closest_point.publish(output_closest);

    // After clustering, create ObstacleArray message
    roar_msgs::ObstacleArray obstacle_array;
    obstacle_array.header = input->header;
    obstacle_array.obstacles.resize(cluster_indices.size());

    // Create a timer for calculating obstacle array
    auto start_time = std::chrono::high_resolution_clock::now();
    for (size_t i = 0; i < cluster_indices.size(); ++i)
    {
        // Calculate cluster centroid and radius
        pcl::PointCloud<pcl::PointXYZ> cluster_cloud;
        for (const auto& idx : cluster_indices[i].indices)
        {
            cluster_cloud.push_back(cloud_no_ground->points[idx]);
        }

        // Calculate centroid
        Eigen::Vector4f centroid;
        pcl::compute3DCentroid(cluster_cloud, centroid);

        // Calculate radius (maximum distance from centroid to any point)
        float max_radius = 0.0f;
        for (const auto& point : cluster_cloud)
        {
            float dist = pcl::geometry::distance(point, pcl::PointXYZ(centroid[0], centroid[1], centroid[2]));
            if (dist > max_radius)
            {
                max_radius = dist;
            }
        }

        // Create obstacle message
        roar_msgs::Obstacle& obstacle = obstacle_array.obstacles[i];
        obstacle.header = input->header;
        obstacle.position.header = input->header;
        obstacle.position.pose.position.x = centroid[0];
        obstacle.position.pose.position.y = centroid[1];
        obstacle.position.pose.position.z = centroid[2];
        obstacle.position.pose.orientation.w = 1.0;  // No rotation
        obstacle.radius.data = max_radius;
        obstacle.id.data = i;
    }
    auto end_time = std::chrono::high_resolution_clock::now();
    auto obstacle_array_time = std::chrono::duration_cast<std::chrono::milliseconds>(end_time - start_time);
    ROS_INFO("Obstacle array time: %ld ms", obstacle_array_time.count());

    // Publish obstacle array
    pub_obstacle_array.publish(obstacle_array);

    auto end_total = std::chrono::high_resolution_clock::now();
    auto total_time = std::chrono::duration_cast<std::chrono::milliseconds>(end_total - start_total);
    ROS_INFO("Total processing time: %ld ms", total_time.count());
}

int main(int argc, char** argv)
{
    ros::init(argc, argv, "zed_obstacle_detector");
    ros::NodeHandle nh;
    ros::NodeHandle private_nh("~");

    // Get parameters
    std::string camera_name;
    private_nh.param<std::string>("camera_name", camera_name, "zed2i");
    bool enable_ground_filtering;
    private_nh.param<bool>("enable_ground_filtering", enable_ground_filtering, false);
    
    // Construct the point cloud topic name
    // std::string point_cloud_topic = "/" + camera_name + "/zed_node/point_cloud/cloud_registered";
    std::string point_cloud_topic = "/camera/depth/points";
    ROS_INFO("Subscribing to point cloud topic: %s", point_cloud_topic.c_str());
    ROS_INFO("Ground filtering: %s", enable_ground_filtering ? "enabled" : "disabled");

    ros::Subscriber sub = nh.subscribe(point_cloud_topic, 1, pointCloudCallback);

    pub_filtered = nh.advertise<sensor_msgs::PointCloud2>("/zed_obstacle/pre_processed_pc", 1);
    pub_no_ground = nh.advertise<sensor_msgs::PointCloud2>("/zed_obstacle/pc_no_ground", 1);
    pub_clusters = nh.advertise<sensor_msgs::PointCloud2>("/zed_obstacle/total_clusters", 1);
    pub_closest_point = nh.advertise<sensor_msgs::PointCloud2>("/zed_obstacle/cluster_closest_point", 1);
    pub_obstacle_array = nh.advertise<roar_msgs::ObstacleArray>("/zed_obstacle/obstacle_array", 1);

    ROS_INFO("ZED Obstacle Detector initialized for camera: %s", camera_name.c_str());
    ros::spin();
    return 0;
}