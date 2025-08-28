#include <pcl/io/ply_io.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/registration/icp.h>
#include <sensor_msgs/PointCloud2.h>
#include <pcl_conversions/pcl_conversions.h>
#include <ros/ros.h>
#include <nav_msgs/Odometry.h>
#include <Eigen/Dense>

double x;
double y;
double z;
double qx;
double qy;
double qz;
double qw;
Eigen::Matrix4f initial_guess = Eigen::Matrix4f::Identity();
// === ODOM CALLBACK ===
void odomCallback(const nav_msgs::Odometry::ConstPtr& msg)
{
    x = msg->pose.pose.position.x;
    y = msg->pose.pose.position.y;
    z = msg->pose.pose.position.z;

    qx = msg->pose.pose.orientation.x;
    qy = msg->pose.pose.orientation.y;
    qz = msg->pose.pose.orientation.z;
    qw = msg->pose.pose.orientation.w;

    Eigen::Quaternionf q(qw, qx, qy, qz);
    Eigen::Matrix3f R = q.toRotationMatrix();

    initial_guess.setIdentity();
    initial_guess.block<3,3>(0,0) = R;
    initial_guess(0,3) = x;
    initial_guess(1,3) = y;
    initial_guess(2,3) = z;
}

// Global
pcl::PointCloud<pcl::PointXYZ>::Ptr map_cloud(new pcl::PointCloud<pcl::PointXYZ>());

void cloudCallback(const sensor_msgs::PointCloud2ConstPtr& msg) {
    ROS_INFO("[+] Recieved Cloud Call Back");
    pcl::PointCloud<pcl::PointXYZ>::Ptr live_cloud(new pcl::PointCloud<pcl::PointXYZ>());
    pcl::fromROSMsg(*msg, *live_cloud);

    // ICP
    pcl::IterativeClosestPoint<pcl::PointXYZ, pcl::PointXYZ> icp;
    icp.setInputSource(live_cloud);
    icp.setInputTarget(map_cloud);
    icp.setMaximumIterations(20);
    icp.setMaxCorrespondenceDistance(0.3); // Set max correspondence distance

    pcl::PointCloud<pcl::PointXYZ> aligned;
    ROS_INFO("[*] Starting Alignment");
    icp.align(aligned, initial_guess);
    ROS_INFO("[*] Finished Alignment");

    if (icp.hasConverged()) {
        Eigen::Matrix4f transform = icp.getFinalTransformation();
        ROS_INFO_STREAM("Pose:\n" << transform);
    } else {
        ROS_WARN("ICP did not converge");
    }
    ROS_INFO("[+] Finished Cloud Call Back");
}

int main(int argc, char** argv) {
    ros::init(argc, argv, "localization_icp");
    ros::NodeHandle nh;

    // Load map once (PLY)
    if (pcl::io::loadPLYFile("/home/rvx/ttt/src/ROAR-Autonomous-System/Localization/icp_localization/maps/map.ply", *map_cloud) == -1) {
        ROS_ERROR("Couldn't read map.ply file");
        return -1;
    }

    // Subscribe to ZED2i filtered cloud
    ros::Subscriber sub = nh.subscribe("/zed_obstacle/debug/filtered_transformed_pc", 1, cloudCallback);
    ros::Subscriber odom_sub = nh.subscribe("/filtered_state", 1, odomCallback);

    ros::spin();
    return 0;
}