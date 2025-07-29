// Include necessary libraries and headers
#include "ROAR_UKF.h"
#include <ros/ros.h>
#include <geometry_msgs/Vector3Stamped.h>
#include <sensor_msgs/Imu.h>
#include <sensor_msgs/NavSatFix.h>
#include <sensor_msgs/JointState.h>
#include <nav_msgs/Odometry.h>
#include <std_msgs/Float64MultiArray.h>
#include <vector>
#include "roar_msgs/Landmark.h"
#include "roar_msgs/LandmarkArray.h"
#include <tf2/LinearMath/Quaternion.h>
#include <tf2/LinearMath/Matrix3x3.h>
#include "WGS84toCartesian.hpp"
#include <tf2_ros/transform_broadcaster.h>
#include <geometry_msgs/TransformStamped.h>
#include <XmlRpcValue.h>
#include "WGS84toCartesian.hpp"

using namespace std;

// Define global variables
Eigen::VectorXd z_measurement(14); // Vector for measurement
Eigen::VectorXd encoder_measurement(2); // Vector for encoder measurement
Eigen::VectorXd encoder_prev_measurement(2); // Vector for encoder measurement

// Define constants
const int n_state_dim = 9;  // State dimension
const float alpha = 0.3;
const float beta_ = 2.0;
const float kappa = 0.1;
ros::Time prev_time_stamp; // Timestamp
ros::Time gps_prev_time_stamp; // Timestamp for GPS
double dt_encoder = 0.0; // encoder time step
double dt_imu = 0.0; //IMU time step
bool new_measurement_received = false; // Flag for new measurement
bool initial_measurement = true; // Flag for initial measurement
double lat0 = 0.0; // Initial latitude
double lon0 = 0.0; // Initial longitude
double yaw = 0.0;
double pitch = 0.0; 
double roll = 0.0;
std::array<double, 2> WGS84Reference;
XmlRpc::XmlRpcValue landmarks;
XmlRpc::XmlRpcValue iPose;
Eigen::Vector3d initialPosition;
tf2::Quaternion initialOrientation;
tf2::Quaternion IMUorientation;

// Initialize Sigma Points and UKF
MerwedSigmaPoints sigma_points(n_state_dim, alpha, beta_, kappa); // Initialize sigma points
UKF ukf(sigma_points); // Initialize UKF

nav_msgs::Odometry state_msg;

// Define ROS subscribers
ros::Subscriber encoder_sub; // Encoder subscriber
ros::Subscriber imu_sub; // IMU subscriber
ros::Subscriber gps_sub; // GPS subscriber
ros::Subscriber trueLandmarkSub; // Landmarks true position subscriber
ros::Subscriber landmarkSub; // Landmark subscriber
ros::Subscriber mag_sub; // Magnetometer subscriber
// Define ROS publisher
ros::Publisher state_publisher; // State publisher

Eigen::Vector3d rotateXYZbyXYZW(const Eigen::Vector3d& rel_pos_rover, const tf2::Quaternion& rover_quat) {
    // Convert tf2 quaternion to Eigen quaternion
    Eigen::Quaterniond q(rover_quat.x(), rover_quat.y(), rover_quat.z(), rover_quat.w());
    // Get rotation matrix
    Eigen::Matrix3d R = q.toRotationMatrix();

    //std::cout << "R: \n" << R << std::endl;
    // Transform the vector
    return R * rel_pos_rover;
}

void publishState(bool showInPlotter = false)
{
    // Initialize state message
    state_msg.header.stamp = ros::Time::now();
    state_msg.header.frame_id = "world";
    state_msg.child_frame_id = "base_link";
    // Transform to world frame
    Eigen::Vector3d rotatedXYZ = rotateXYZbyXYZW({ukf.x_post[7], ukf.x_post[8], 0}, initialOrientation) + initialPosition;
    // Publish the state message
    state_msg.pose.pose.orientation.w = ukf.x_post[0];
    state_msg.pose.pose.orientation.x = ukf.x_post[1];
    state_msg.pose.pose.orientation.y = ukf.x_post[2];
    state_msg.pose.pose.orientation.z = ukf.x_post[3];
    state_msg.twist.twist.angular.x = ukf.x_post[4];
    state_msg.twist.twist.angular.y = ukf.x_post[5];
    state_msg.twist.twist.angular.z = ukf.x_post[6];
    state_msg.pose.pose.position.x = rotatedXYZ[0];
    state_msg.pose.pose.position.y = rotatedXYZ[1];

    //std::cout << "State: " << state_msg.pose.pose.position.x << ", " << state_msg.pose.pose.position.y << std::endl;

    // Publish covariance
    state_msg.pose.covariance[0] = ukf.P_post.col(7)(7);
    state_msg.pose.covariance[1] = ukf.P_post.col(8)(7);
    state_msg.pose.covariance[6] = ukf.P_post.col(7)(8);
    state_msg.pose.covariance[7] = ukf.P_post.col(8)(8);
    state_msg.pose.covariance[2] = ukf.P_post.col(0)(0);
    state_msg.pose.covariance[3] = ukf.P_post.col(1)(1);
    state_msg.pose.covariance[4] = ukf.P_post.col(2)(2);
    state_msg.pose.covariance[5] = ukf.P_post.col(3)(3);
    /***
    Publish the state message to the /filtered_state topic
    ***/
    if (showInPlotter) {
        state_msg.pose.covariance[35] = 1;
    } else {
        state_msg.pose.covariance[35] = 0;
    }
    // Publish the message
    state_publisher.publish(state_msg);
}

//Pose callback function
void poseCallback(){
    static tf2_ros::TransformBroadcaster br;
    geometry_msgs::TransformStamped transformStamped;
    
    transformStamped.header.stamp = ros::Time::now();
    transformStamped.header.frame_id = "world";
    transformStamped.child_frame_id = "base_link";
    transformStamped.transform.translation.x = ukf.x_post[7];
    transformStamped.transform.translation.y = ukf.x_post[8];
    transformStamped.transform.translation.z = 0.0;

    tf2::Quaternion q(ukf.x_post[1], ukf.x_post[2], ukf.x_post[3], ukf.x_post[0]);
    transformStamped.transform.rotation.x = q.x();
    transformStamped.transform.rotation.y = q.y();
    transformStamped.transform.rotation.z = q.z();
    transformStamped.transform.rotation.w = q.w();  
    br.sendTransform(transformStamped);
}

// Callback function for encoder data
void encoderCallback(const sensor_msgs::JointState::ConstPtr& msg)
{
    double reading_left = msg->position[1];
    double reading_right = msg->position[4];
    
    if(!(reading_left == 0 || reading_right == 0)){
        // Check if prev_time_stamp is zero
        if (prev_time_stamp.isZero()) 
        {
            prev_time_stamp = msg->header.stamp; // Initialize prev_time_stamp
            encoder_prev_measurement[0] = reading_left;
            encoder_prev_measurement[1] = reading_right;
            return;
        }

        // Calculate time difference
        ros::Time current_time_stamp = msg->header.stamp;
        dt_encoder = (current_time_stamp - prev_time_stamp).toSec();
        
        // Store encoder measurements
        encoder_measurement[0] = (reading_left - encoder_prev_measurement[0]) / dt_encoder;
        encoder_measurement[1] = (reading_right - encoder_prev_measurement[1]) / dt_encoder;
        
        // Call UKF encoder callback function
        ukf.encoder_callback(encoder_measurement, dt_encoder, yaw, pitch);
        // Update prev_time_stamp
        prev_time_stamp = current_time_stamp;

        encoder_prev_measurement[0] = reading_left;
        encoder_prev_measurement[1] = reading_right;
        
        publishState(); // Publish the state message

        std::cout << "Encoder Updated" << std::endl;
        std::cout << "P: " << ukf.P_post.col(7)(7) << std::endl;
    }
    
    
}

// Callback function for GPS data
void gpsCallback(const sensor_msgs::NavSatFix::ConstPtr& msg)
{
    if (initial_measurement == true)
    {
        lat0 = msg->latitude; // Initialize lat0
        lon0 = msg->longitude; // Initialize lon0
        initial_measurement = false;
        WGS84Reference = {lon0, lat0};
    }

    // Store GPS measurements
    z_measurement[9] = msg->latitude;
    z_measurement[10] = msg->longitude;

    //std::array<double, 2> gpsXYZ{wgs84::toCartesian(WGS84Reference, {z_measurement[10], z_measurement[9]})};

    // Transform to world frame
    //Eigen::Vector3d rotatedXYZ = rotateXYZbyXYZW({gpsXYZ[0], -gpsXYZ[1], 0}, initialOrientation) + initialPosition;

    //std::cout << "initial Position: " << initialPosition[0] << ", " << initialPosition[1] << ", " << initialPosition[2] << std::endl;

    //std::cout << "GPS XYZ: " << rotatedXYZ[0] << ", " << rotatedXYZ[1] << ", " << rotatedXYZ[2] << std::endl;

    //std::array<double, 2> rotatedLonLat{wgs84::fromCartesian(WGS84Reference, {rotatedXYZ[0], rotatedXYZ[1]})};

    // Store GPS measurements
    //z_measurement[9] = rotatedLonLat[0];
    //z_measurement[10] = rotatedLonLat[1];

    // Call UKF GPS callback function
    ukf.gps_callback(z_measurement, lon0, lat0);

    publishState(true); // Publish the state message

    std::cout << "GPS Updated" << std::endl;
    std::cout << "P: " << ukf.P_post.col(7)(7) << std::endl;
}

// Callback function for IMU data
void imuCallback(const sensor_msgs::Imu::ConstPtr& msg)
{
    // yaw = msg->orientation.z;
    // yaw = yaw * PI / 180.0;
    if (prev_time_stamp.isZero()) 
    {
        prev_time_stamp = msg->header.stamp; // Initialize prev_time_stamp
        return;
    }

    // Calculate time difference
    ros::Time current_time_stamp = msg->header.stamp;
    dt_imu = (current_time_stamp - prev_time_stamp).toSec();

    tf2::Quaternion quat (msg->orientation.x, msg->orientation.y, msg->orientation.z, msg->orientation.w);
    tf2::Matrix3x3 m(IMUorientation * quat);
    m.getRPY(roll, pitch, yaw);

    z_measurement[0] = msg->angular_velocity.x;
    z_measurement[1] = msg->angular_velocity.y;
    z_measurement[2] = msg->angular_velocity.z;
    z_measurement[3] = msg->linear_acceleration.x;
    z_measurement[4] = msg->linear_acceleration.y;
    z_measurement[5] = msg->linear_acceleration.z + 9.81;

    ukf.imu_callback(encoder_measurement, z_measurement, dt_imu, roll, yaw, pitch); //////////need to be fixed
    prev_time_stamp = current_time_stamp;

    publishState(); // Publish the state message
    poseCallback(); // Call pose callback to publish the transform

    std::cout << "IMU Updated" << std::endl;
    std::cout << "P: " << ukf.P_post.col(7)(7) << std::endl;
}

void bnoCallback(const sensor_msgs::Imu::ConstPtr& msg)
{
    tf2::Quaternion quat (msg->orientation.x, msg->orientation.y, msg->orientation.z, msg->orientation.w);
    tf2::Matrix3x3 m(IMUorientation * quat);
    m.getRPY(roll, pitch, yaw);

    ukf.bno_callback(roll, pitch, yaw);

    publishState(); // Publish the state message

    poseCallback(); // Call pose callback to publish the transform

    //std::cout << "BNO Updated" << std::endl;
    //std::cout << "P: " << ukf.P_post.col(7)(7) << std::endl;
}

void landmarkCallback(const roar_msgs::Landmark::ConstPtr& landmark_poses) {

    //Calculate the rover position
    //Rover's position: P
	//Landmark TRUE position
	//Landmark RELATIVE position: landmark_poses

    if(landmarks.size() > 0){

        double rel_x = landmark_poses->pose.pose.position.x;
        double rel_y = landmark_poses->pose.pose.position.z;
        double rel_z = landmark_poses->pose.pose.position.y;

        Eigen::Vector3d rel_pos_rover(rel_x, rel_y, rel_z);

        // Get rover orientation as quaternion (from your UKF state or message)
        //make sure which is w and which is x y z
        tf2::Quaternion rover_quat(
            ukf.x_post[0], // w
            ukf.x_post[1], // x
            ukf.x_post[2], // y
            ukf.x_post[3]  // z
        );

        // Transform to world frame
        Eigen::Vector3d rel_pos_world = rotateXYZbyXYZW(rel_pos_rover, rover_quat);

        rel_x = rel_pos_world.x() + 0.0; // Adjusting y position to match the gps postion "diff between camera and gps"
        rel_y = rel_pos_world.y() + 0.0; // Adjusting y position to match the gps postion "diff between camera and gps"

        double rov_x = static_cast<double>(landmarks[std::to_string(landmark_poses->id)]["x"]);
        double rov_y = static_cast<double>(landmarks[std::to_string(landmark_poses->id)]["y"]);
        
        //All Relative Positions 
        std::vector<Eigen::Vector2d> rel_pos_all = {
            {rov_x + rel_x, rov_y + rel_y},
            {rov_x + rel_x, rov_y - rel_y},
            {rov_x + rel_x, - rov_y + rel_y},
            {rov_x + rel_x, - rov_y - rel_y},

            {rov_x - rel_x, rov_y + rel_y},
            {rov_x - rel_x, rov_y - rel_y},
            {rov_x - rel_x, - rov_y + rel_y},
            {rov_x - rel_x, - rov_y - rel_y},

            {- rov_x + rel_x, rov_y + rel_y},
            {- rov_x + rel_x, rov_y - rel_y},
            {- rov_x + rel_x, - rov_y + rel_y},
            {- rov_x + rel_x, - rov_y - rel_y},

            {- rov_x - rel_x, rov_y + rel_y},
            {- rov_x - rel_x, rov_y - rel_y},
            {- rov_x - rel_x, - rov_y + rel_y},
            {- rov_x - rel_x, - rov_y - rel_y},
            
            {rov_x + rel_y, rov_y + rel_x},
            {rov_x + rel_y, rov_y - rel_x},
            {rov_x + rel_y, - rov_y + rel_x},
            {rov_x + rel_y, - rov_y - rel_x},

            {rov_x - rel_y, rov_y + rel_x},
            {rov_x - rel_y, rov_y - rel_x},
            {rov_x - rel_y, - rov_y + rel_x},
            {rov_x - rel_y, - rov_y - rel_x},

            {- rov_x + rel_y, rov_y + rel_x},
            {- rov_x + rel_y, rov_y - rel_x},
            {- rov_x + rel_y, - rov_y + rel_x},
            {- rov_x + rel_y, - rov_y - rel_x},

            {- rov_x - rel_y, rov_y + rel_x},
            {- rov_x - rel_y, rov_y - rel_x},
            {- rov_x - rel_y, - rov_y + rel_x},
            {- rov_x - rel_y, - rov_y - rel_x}
        };

        Eigen::Vector2d nearestPos = rel_pos_all[0];
        double minDist = 1000000.0;
        double dist = 0.0;
        
        double rovCurrentX = ukf.x_post[7];
        double rovCurrentY = ukf.x_post[8];

        for (int i = 0; i < 32; i++){
            //calc dist
            dist = sqrt(pow((rel_pos_all[i][0] - rovCurrentX) ,2) + pow((rel_pos_all[i][1] - rovCurrentY) ,2));

            if (dist < minDist)
            {
                minDist = dist;
                nearestPos = rel_pos_all[i];
            }
        }
        
        z_measurement[11] = nearestPos[0];
        z_measurement[12] = nearestPos[1];
        z_measurement[13] = 0;

        ukf.LL_Callback(z_measurement);

        poseCallback();
        std::cout << "Landmark Updated" << std::endl;
        std::cout << "P: " << ukf.P_post.col(7)(7) << std::endl;
    }

}

void magnetometerCallback(const geometry_msgs::Vector3Stamped::ConstPtr& msg)
{
    z_measurement[6] = msg->vector.x;
    z_measurement[7] = msg->vector.y;
    z_measurement[8] = msg->vector.z;

}

bool loadInitialPose(ros::NodeHandle& nh, Eigen::Vector3d& position, tf2::Quaternion& orientation, tf2::Quaternion& IMUorientation) {
    XmlRpc::XmlRpcValue iPose;
    if (!nh.getParam("iPose", iPose)) {
        ROS_ERROR("Failed to get param 'iPose'");
        return false;
    }

    // --- Extract position ---
    if (iPose.getType() != XmlRpc::XmlRpcValue::TypeStruct ||
        !iPose.hasMember("position") ||
        iPose["position"].getType() != XmlRpc::XmlRpcValue::TypeStruct) {
        ROS_ERROR("Invalid or missing 'position' in iPose param");
        return false;
    }

    try {
        position.x() = static_cast<double>(iPose["position"]["x"]);
        position.y() = static_cast<double>(iPose["position"]["y"]);
        position.z() = static_cast<double>(iPose["position"]["z"]);
    } catch (...) {
        ROS_ERROR("Failed to read 'position' values from iPose");
        return false;
    }

    // --- Extract orientation ---
    if (!iPose.hasMember("orientation") ||
        iPose["orientation"].getType() != XmlRpc::XmlRpcValue::TypeStruct) {
        ROS_ERROR("Invalid or missing 'orientation' in iPose param");
        return false;
    }

    try {
        double w = static_cast<double>(iPose["orientation"]["w"]);
        double x = static_cast<double>(iPose["orientation"]["x"]);
        double y = static_cast<double>(iPose["orientation"]["y"]);
        double z = static_cast<double>(iPose["orientation"]["z"]);
        orientation = tf2::Quaternion(x, y, z, w);  // tf2 uses (x, y, z, w)
    } catch (...) {
        ROS_ERROR("Failed to read 'orientation' values from iPose");
        return false;
    }

    // --- Extract orientation ---
    if (!iPose.hasMember("IMUorientation") ||
        iPose["IMUorientation"].getType() != XmlRpc::XmlRpcValue::TypeStruct) {
        ROS_ERROR("Invalid or missing 'IMUorientation' in iPose param");
        return false;
    }

    try {
        double w = static_cast<double>(iPose["IMUorientation"]["w"]);
        double x = static_cast<double>(iPose["IMUorientation"]["x"]);
        double y = static_cast<double>(iPose["IMUorientation"]["y"]);
        double z = static_cast<double>(iPose["IMUorientation"]["z"]);
        IMUorientation = tf2::Quaternion(x, y, z, w);  // tf2 uses (x, y, z, w)
    } catch (...) {
        ROS_ERROR("Failed to read 'orientation' values from iPose");
        return false;
    }

    return true;
}

// Main function
int main(int argc, char **argv) 
{
    ros::init(argc, argv, "ukf_localization"); // Initialize ROS node
    ros::NodeHandle nh; // Create ROS node handle
    
    // Initialize ROS subscribers
    mag_sub = nh.subscribe("/magnetometer", 1000, magnetometerCallback);
    gps_sub = nh.subscribe("/gps", 1000, gpsCallback);
    imu_sub = nh.subscribe("/imu", 1000, bnoCallback);
    encoder_sub = nh.subscribe("/joint_states", 1000, encoderCallback);
    landmarkSub = nh.subscribe("/landmark_topic", 1000, landmarkCallback);

    if (loadInitialPose(nh, initialPosition, initialOrientation, IMUorientation)) {
        ROS_INFO_STREAM("Initial Position: " << initialPosition.transpose());
        ROS_INFO_STREAM("Initial Orientation (x y z w): "
                        << initialOrientation.x() << " "
                        << initialOrientation.y() << " "
                        << initialOrientation.z() << " "
                        << initialOrientation.w());
        ROS_INFO_STREAM("IMU Mounting (x y z w): "
                        << IMUorientation.x() << " "
                        << IMUorientation.y() << " "
                        << IMUorientation.z() << " "
                        << IMUorientation.w());
    } else {
        ROS_ERROR("Could not load initial pose");
    }

    // Initialize ROS publisher
    state_publisher = nh.advertise<nav_msgs::Odometry>("/filtered_state", 1000);
    ros::Rate loop_rate(10); // Set loop rate

    // Main ROS loop
    while (ros::ok())
    {
        ros::spinOnce(); // Process callbacks
    }
    return 0;
}