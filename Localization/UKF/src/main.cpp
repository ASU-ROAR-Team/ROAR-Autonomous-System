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
ros::Time encoder_prev_time_stamp; // Timestamp for encoder
ros::Time imu_prev_time_stamp; // Timestamp for IMU
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
vector<roar_msgs::Landmark> ROVlandmarks;

// Initialize Sigma Points and UKF
MerwedSigmaPoints sigma_points(n_state_dim, alpha, beta_, kappa); // Initialize sigma points
UKF ukf(sigma_points); // Initialize UKF

// Define ROS subscribers
ros::Subscriber encoder_sub; // Encoder subscriber
ros::Subscriber imu_sub; // IMU subscriber
ros::Subscriber gps_sub; // GPS subscriber
ros::Subscriber trueLandmarkSub; // Landmarks true position subscriber
ros::Subscriber landmarkSub; // Landmark subscriber


// Define ROS publisher
ros::Publisher state_publisher; // State publisher

//Pose callback function
void poseCallback(){
    static tf2_ros::TransformBroadcaster br;
    geometry_msgs::TransformStamped transformStamped;
    
    transformStamped.header.stamp = ros::Time::now();
    transformStamped.header.frame_id = "base_link";
    transformStamped.child_frame_id = "camera_link";
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
    //std::cout << "encoooder" << endl;
    nav_msgs::Odometry state_msg;
    double reading_left = msg->position[1];
    double reading_right = msg->position[4];
    
    if(!(reading_left == 0 || reading_right == 0)){
        // Check if encoder_prev_time_stamp is zero
        if (encoder_prev_time_stamp.isZero()) 
        {
            encoder_prev_time_stamp = msg->header.stamp; // Initialize encoder_prev_time_stamp
            encoder_prev_measurement[0] = reading_left;
            encoder_prev_measurement[1] = reading_right;
            return;
        }

        // Calculate time difference
        ros::Time encoder_current_time_stamp = msg->header.stamp;
        dt_encoder = (encoder_current_time_stamp - encoder_prev_time_stamp).toSec();
        // Store encoder measurements

        /*
        encoder_measurement[0] = (msg->position[1])* 0.1047;
        encoder_measurement[1] = (msg->position[4])* 0.1047;
        */

        encoder_measurement[0] = (reading_left - encoder_prev_measurement[0]) / dt_encoder;
        encoder_measurement[1] = (reading_right - encoder_prev_measurement[1]) / dt_encoder;

        /*
        std::cout << "msg->position[0]: " << reading_right << endl;
        std::cout << "encoder_prev_lef: " << encoder_prev_measurement[0] << endl;
        std::cout << "dt_encoder" << dt_encoder << endl;
        std::cout << "Wl: " << encoder_measurement[0] << std::endl;
        std::cout << "Wr: " << encoder_measurement[1] << std::endl;
        std::cout << "*******************************************************************" << std::endl;
        */
        
        // Call UKF encoder callback function
        ukf.encoder_callback(encoder_measurement, dt_encoder, yaw, pitch);
        // Update encoder_prev_time_stamp
        encoder_prev_time_stamp = encoder_current_time_stamp;

        encoder_prev_measurement[0] = reading_left;
        encoder_prev_measurement[1] = reading_right;
        
        // Publish state message
        state_msg.pose.pose.orientation.x = ukf.x_post[0];
        state_msg.pose.pose.orientation.y = ukf.x_post[1];
        state_msg.pose.pose.orientation.z = ukf.x_post[2];
        state_msg.pose.pose.orientation.w = ukf.x_post[3];
        state_msg.twist.twist.angular.x = ukf.x_post[4];
        state_msg.twist.twist.angular.y = ukf.x_post[5];
        state_msg.twist.twist.angular.z = ukf.x_post[6];
        state_msg.pose.pose.position.x = ukf.x_post[7];
        state_msg.pose.pose.position.y = ukf.x_post[8];

        state_msg.pose.covariance[0] = ukf.P_post.col(7)(7);
        state_msg.pose.covariance[1] = ukf.P_post.col(8)(7);
        state_msg.pose.covariance[6] = ukf.P_post.col(7)(8);
        state_msg.pose.covariance[7] = ukf.P_post.col(8)(8);
        state_msg.pose.covariance[2] = ukf.P_post.col(0)(0);
        state_msg.pose.covariance[3] = ukf.P_post.col(1)(1);
        state_msg.pose.covariance[4] = ukf.P_post.col(2)(2);
        state_msg.pose.covariance[5] = ukf.P_post.col(3)(3);
        

        
        state_publisher.publish(state_msg);
    }
    
    
}

// Callback function for GPS data
void gpsCallback(const sensor_msgs::NavSatFix::ConstPtr& msg)
{
    nav_msgs::Odometry state_msg;

    if (initial_measurement == true)
    {
        lat0 = msg->latitude; // Initialize lat0
        lon0 = msg->longitude; // Initialize lon0
        initial_measurement = false;
    }

    // Store GPS measurements
    z_measurement[9] = msg->latitude;
    z_measurement[10] = msg->longitude;

    // Call UKF GPS callback function
    ukf.gps_callback(z_measurement, lon0, lat0);

    // Publish state message
    state_msg.pose.pose.orientation.x = ukf.x_post[0];
    state_msg.pose.pose.orientation.y = ukf.x_post[1];
    state_msg.pose.pose.orientation.z = ukf.x_post[2];
    state_msg.pose.pose.orientation.w = ukf.x_post[3];
    state_msg.twist.twist.angular.x = ukf.x_post[4];
    state_msg.twist.twist.angular.y = ukf.x_post[5];
    state_msg.twist.twist.angular.z = ukf.x_post[6];
    
    //state_msg.pose.pose.position.x = ukf.x_post[7];
    //state_msg.pose.pose.position.y = ukf.x_post[8];
    std::array<double, 2> WGS84Reference{lon0, lat0};
    std::array<double, 2> result{wgs84::toCartesian(WGS84Reference, {z_measurement[10], z_measurement[9]})};
    state_msg.pose.pose.position.x = result[0];
    state_msg.pose.pose.position.y = result[1];

    state_msg.pose.covariance[0] = ukf.P_post.col(7)(7);
    state_msg.pose.covariance[1] = ukf.P_post.col(8)(7);
    state_msg.pose.covariance[6] = ukf.P_post.col(7)(8);
    state_msg.pose.covariance[7] = ukf.P_post.col(8)(8);
    state_msg.pose.covariance[2] = ukf.P_post.col(0)(0);
    state_msg.pose.covariance[3] = ukf.P_post.col(1)(1);
    state_msg.pose.covariance[4] = ukf.P_post.col(2)(2);
    state_msg.pose.covariance[5] = ukf.P_post.col(3)(3);

    state_msg.pose.covariance[35] = 1; //Used to update the plotter only

    state_publisher.publish(state_msg);
    
    //std::cout <<  ukf.P_post.col(7)(7) << endl;
}

// Callback function for IMU data
void imuCallback(const sensor_msgs::Imu::ConstPtr& msg)
{
    // yaw = msg->orientation.z;
    // yaw = yaw * PI / 180.0;
    if (imu_prev_time_stamp.isZero()) 
    {
        imu_prev_time_stamp = msg->header.stamp; // Initialize encoder_prev_time_stamp
        return;
    }

    // Calculate time difference
    ros::Time imu_current_time_stamp = msg->header.stamp;
    dt_imu = (imu_current_time_stamp - imu_prev_time_stamp).toSec();
    nav_msgs::Odometry state_msg;

    tf2::Quaternion quat (msg->orientation.w, msg->orientation.x, msg->orientation.y, msg->orientation.z);
    tf2::Matrix3x3 m(quat);
    m.getRPY(roll, pitch, yaw);

    z_measurement[0] = msg->angular_velocity.x;
    z_measurement[1] = msg->angular_velocity.y;
    z_measurement[2] = msg->angular_velocity.z;
    z_measurement[3] = msg->linear_acceleration.x;
    z_measurement[4] = msg->linear_acceleration.y;
    z_measurement[5] = msg->linear_acceleration.z;

    ukf.imu_callback(encoder_measurement, z_measurement, dt_imu); //////////need to be fixed
    imu_prev_time_stamp = imu_current_time_stamp;

    // Publish state message
    state_msg.pose.pose.orientation.x = ukf.x_post[0];
    state_msg.pose.pose.orientation.y = ukf.x_post[1];
    state_msg.pose.pose.orientation.z = ukf.x_post[2];
    state_msg.pose.pose.orientation.w = ukf.x_post[3];
    state_msg.twist.twist.angular.x = ukf.x_post[4];
    state_msg.twist.twist.angular.y = ukf.x_post[5];
    state_msg.twist.twist.angular.z = ukf.x_post[6];
    state_msg.pose.pose.position.x = ukf.x_post[7];
    state_msg.pose.pose.position.y = ukf.x_post[8];

    state_msg.pose.covariance[0] = ukf.P_post.col(7)(7);
    state_msg.pose.covariance[1] = ukf.P_post.col(8)(7);
    state_msg.pose.covariance[6] = ukf.P_post.col(7)(8);
    state_msg.pose.covariance[7] = ukf.P_post.col(8)(8);

    state_publisher.publish(state_msg);
}

void bnoCallback(const sensor_msgs::Imu::ConstPtr& msg)
{
    nav_msgs::Odometry state_msg;

    tf2::Quaternion quat (msg->orientation.x, msg->orientation.y, msg->orientation.z, msg->orientation.w);
    tf2::Matrix3x3 m(quat);
    m.getRPY(roll, pitch, yaw);

    // (yaw = msg->orientation.z;)
    // yaw = yaw * PI / 180.0;
    // pitch = msg->orientation.y; //????????????
    // pitch = pitch * PI / 180.0;  //???????????
    ukf.bno_callback(roll, pitch, yaw);

    // Publish state message
    state_msg.pose.pose.orientation.x = ukf.x_post[0];
    state_msg.pose.pose.orientation.y = ukf.x_post[1];
    state_msg.pose.pose.orientation.z = ukf.x_post[2];
    state_msg.pose.pose.orientation.w = ukf.x_post[3];
    state_msg.twist.twist.angular.x = ukf.x_post[4];
    state_msg.twist.twist.angular.y = ukf.x_post[5];
    state_msg.twist.twist.angular.z = ukf.x_post[6];
    state_msg.pose.pose.position.x = ukf.x_post[7];
    state_msg.pose.pose.position.y = ukf.x_post[8];

    /*
    state_msg.pose.covariance[0] = ukf.P_post.col(7)(7);
    state_msg.pose.covariance[1] = ukf.P_post.col(8)(7);
    state_msg.pose.covariance[6] = ukf.P_post.col(7)(8);
    state_msg.pose.covariance[7] = ukf.P_post.col(8)(8);
    */

    state_msg.pose.covariance[2] = ukf.P_post.col(0)(0);
    state_msg.pose.covariance[3] = ukf.P_post.col(1)(1);
    state_msg.pose.covariance[4] = ukf.P_post.col(2)(2);
    state_msg.pose.covariance[5] = ukf.P_post.col(3)(3);
    state_msg.pose.covariance[0] = ukf.P_post.col(7)(7);
    state_msg.pose.covariance[1] = ukf.P_post.col(8)(7);
    state_msg.pose.covariance[6] = ukf.P_post.col(7)(8);
    state_msg.pose.covariance[7] = ukf.P_post.col(8)(8);

    state_publisher.publish(state_msg);

    poseCallback(); // Call pose callback to publish the transform
}

void trueLandmarkCallback(const roar_msgs::LandmarkArray::ConstPtr& msg){

    //Set the landmarks
    ROVlandmarks = msg->landmarks;
    std::cout << "[*] Landmarks recieved! \n";

    nav_msgs::Odometry state_msg;

    state_msg.pose.pose.orientation.x = ukf.x_post[0];
    state_msg.pose.pose.orientation.y = ukf.x_post[1];
    state_msg.pose.pose.orientation.z = ukf.x_post[2];
    state_msg.pose.pose.orientation.w = ukf.x_post[3];
    state_msg.twist.twist.angular.x = ukf.x_post[4];
    state_msg.twist.twist.angular.y = ukf.x_post[5];
    state_msg.twist.twist.angular.z = ukf.x_post[6];
    state_msg.pose.pose.position.x = ukf.x_post[7];
    state_msg.pose.pose.position.y = ukf.x_post[8];

    state_msg.pose.covariance[0] = ukf.P_post.col(7)(7);
    state_msg.pose.covariance[1] = ukf.P_post.col(8)(7);
    state_msg.pose.covariance[6] = ukf.P_post.col(7)(8);
    state_msg.pose.covariance[7] = ukf.P_post.col(8)(8);

    state_publisher.publish(state_msg);
}

Eigen::Vector3d roverToWorld(const Eigen::Vector3d& rel_pos_rover, const tf2::Quaternion& rover_quat) {
    // Convert tf2 quaternion to Eigen quaternion
    Eigen::Quaterniond q(rover_quat.x(), rover_quat.y(), rover_quat.z(), rover_quat.w());
    // Get rotation matrix
    Eigen::Matrix3d R = q.toRotationMatrix();
    // Transform the vector
    return R * rel_pos_rover;
}

void landmarkCallback(const roar_msgs::Landmark::ConstPtr& landmark_poses) {

    //Calculate the rover position
    //Rover's position: P
	//Landmark TRUE position: ROVlandmarks
	//Landmark RELATIVE position: landmark_poses

    if(ROVlandmarks.size() > 0){

        double rel_x = landmark_poses->pose.pose.position.x;
        double rel_y = landmark_poses->pose.pose.position.z;
        double rel_z = landmark_poses->pose.pose.position.y;

        Eigen::Vector3d rel_pos_rover(rel_x, rel_y, rel_z);

        // Get rover orientation as quaternion (from your UKF state or message)
        //make sure which is w and which is x y z
        tf2::Quaternion rover_quat(
            ukf.x_post[1], // x
            ukf.x_post[2], // y
            ukf.x_post[3], // z
            ukf.x_post[0]  // w
        );

        // Transform to world frame
        Eigen::Vector3d rel_pos_world = roverToWorld(rel_pos_rover, rover_quat);

        rel_x = rel_pos_world.x();
        rel_y = rel_pos_world.y();

        std::cout << "Relative position in world frame: " 
                  << rel_pos_world.x() << ", " 
                  << rel_pos_world.y() << ", " 
                  << rel_pos_world.z() << std::endl;
        
        //All Relative Positions 
        std::vector<Eigen::Vector2d> rel_pos_all = {
            {ROVlandmarks[(landmark_poses->id)-51].pose.pose.position.x + rel_x, ROVlandmarks[(landmark_poses->id)-51].pose.pose.position.y + rel_y},
            {ROVlandmarks[(landmark_poses->id)-51].pose.pose.position.x + rel_x, ROVlandmarks[(landmark_poses->id)-51].pose.pose.position.y - rel_y},
            {ROVlandmarks[(landmark_poses->id)-51].pose.pose.position.x + rel_x, - ROVlandmarks[(landmark_poses->id)-51].pose.pose.position.y + rel_y},
            {ROVlandmarks[(landmark_poses->id)-51].pose.pose.position.x + rel_x, - ROVlandmarks[(landmark_poses->id)-51].pose.pose.position.y - rel_y},

            {ROVlandmarks[(landmark_poses->id)-51].pose.pose.position.x - rel_x, ROVlandmarks[(landmark_poses->id)-51].pose.pose.position.y + rel_y},
            {ROVlandmarks[(landmark_poses->id)-51].pose.pose.position.x - rel_x, ROVlandmarks[(landmark_poses->id)-51].pose.pose.position.y - rel_y},
            {ROVlandmarks[(landmark_poses->id)-51].pose.pose.position.x - rel_x, - ROVlandmarks[(landmark_poses->id)-51].pose.pose.position.y + rel_y},
            {ROVlandmarks[(landmark_poses->id)-51].pose.pose.position.x - rel_x, - ROVlandmarks[(landmark_poses->id)-51].pose.pose.position.y - rel_y},

            {- ROVlandmarks[(landmark_poses->id)-51].pose.pose.position.x + rel_x, ROVlandmarks[(landmark_poses->id)-51].pose.pose.position.y + rel_y},
            {- ROVlandmarks[(landmark_poses->id)-51].pose.pose.position.x + rel_x, ROVlandmarks[(landmark_poses->id)-51].pose.pose.position.y - rel_y},
            {- ROVlandmarks[(landmark_poses->id)-51].pose.pose.position.x + rel_x, - ROVlandmarks[(landmark_poses->id)-51].pose.pose.position.y + rel_y},
            {- ROVlandmarks[(landmark_poses->id)-51].pose.pose.position.x + rel_x, - ROVlandmarks[(landmark_poses->id)-51].pose.pose.position.y - rel_y},

            {- ROVlandmarks[(landmark_poses->id)-51].pose.pose.position.x - rel_x, ROVlandmarks[(landmark_poses->id)-51].pose.pose.position.y + rel_y},
            {- ROVlandmarks[(landmark_poses->id)-51].pose.pose.position.x - rel_x, ROVlandmarks[(landmark_poses->id)-51].pose.pose.position.y - rel_y},
            {- ROVlandmarks[(landmark_poses->id)-51].pose.pose.position.x - rel_x, - ROVlandmarks[(landmark_poses->id)-51].pose.pose.position.y + rel_y},
            {- ROVlandmarks[(landmark_poses->id)-51].pose.pose.position.x - rel_x, - ROVlandmarks[(landmark_poses->id)-51].pose.pose.position.y - rel_y}
        };

        Eigen::Vector2d nearestPos = rel_pos_all[0];
        double minDist = 1000000.0;

        std::cout << "x_post[7]: " << ukf.x_post[7] << " | x_post[8]: " << ukf.x_post[8] << std::endl; 
        std::cout << "rel_x: " << rel_x << " | rel_y: " << rel_y << std::endl; 

        for (int i = 0; i < 16; i++){
            //calc dist
            double dist = sqrt(pow((rel_pos_all[i][0] - ukf.x_post[7]) ,2) + pow((rel_pos_all[i][1] - ukf.x_post[8]) ,2));

            if (dist < minDist){
                minDist = dist;
                nearestPos = rel_pos_all[i];
            }
            //std::cout << "case[" << i << "] X: " << rel_pos_all[i][0] << " | Y: " << rel_pos_all[i][1] << " | Dist: " << dist << std::endl;
        }
        
        z_measurement[11] = nearestPos[0];
        z_measurement[12] = nearestPos[1];
        z_measurement[13] = ROVlandmarks[(landmark_poses->id)-51].pose.pose.position.z - landmark_poses->pose.pose.position.y;
        
        std::cout << "LL-> X: " << z_measurement[11] << " | Y: " << z_measurement[12] << std::endl;
        std::cout << "****************************" << std::endl;

        ukf.LL_Callback(z_measurement);

        nav_msgs::Odometry state_msg;

        state_msg.pose.pose.orientation.x = ukf.x_post[0];
        state_msg.pose.pose.orientation.y = ukf.x_post[1];
        state_msg.pose.pose.orientation.z = ukf.x_post[2];
        state_msg.pose.pose.orientation.w = ukf.x_post[3];
        state_msg.twist.twist.angular.x = ukf.x_post[4];
        state_msg.twist.twist.angular.y = ukf.x_post[5];
        state_msg.twist.twist.angular.z = ukf.x_post[6];
        state_msg.pose.pose.position.x = ukf.x_post[7];
        state_msg.pose.pose.position.y = ukf.x_post[8];

        state_msg.pose.covariance[0] = ukf.P_post.col(7)(7);
        state_msg.pose.covariance[1] = ukf.P_post.col(8)(7);
        state_msg.pose.covariance[6] = ukf.P_post.col(7)(8);
        state_msg.pose.covariance[7] = ukf.P_post.col(8)(8);

        state_msg.pose.covariance[35] = 1; //Used to update the plotter only

        state_publisher.publish(state_msg);
    }

}

// Main function
int main(int argc, char **argv) 
{
    ros::init(argc, argv, "ukf_localization"); // Initialize ROS node
    ros::NodeHandle nh; // Create ROS node handle
    
    // Initialize ROS subscribers
    gps_sub = nh.subscribe("/gps", 1000, gpsCallback);
    imu_sub = nh.subscribe("/imu", 1000, bnoCallback);
    encoder_sub = nh.subscribe("/joint_states", 1000, encoderCallback);
    
  
    trueLandmarkSub = nh.subscribe("/landmark_array_topic", 1000, trueLandmarkCallback);
    landmarkSub = nh.subscribe("/landmark_topic", 1000, landmarkCallback);

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