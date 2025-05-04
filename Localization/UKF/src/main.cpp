// Include necessary libraries and headers
#include "ROAR_UKF.h"
#include <ros/ros.h>
#include <geometry_msgs/Vector3Stamped.h>
#include <sensor_msgs/Imu.h>
#include <std_msgs/Float64MultiArray.h>
#include <vector>
#include "roar_msgs/Landmark.h"
#include "roar_msgs/LandmarkArray.h"
#include <tf2/LinearMath/Quaternion.h>
#include <tf2/LinearMath/Matrix3x3.h>

using namespace std;

// Define global variables
Eigen::VectorXd z_measurement(14); // Vector for measurement
Eigen::VectorXd encoder_measurement(2); // Vector for encoder measurement

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
ros::Subscriber imu_sub; // IMU subscriber
ros::Subscriber encoder_sub; // Encoder subscriber
ros::Subscriber gps_sub; // GPS subscriber
ros::Subscriber trueLandmarkSub; // Landmarks true position subscriber
ros::Subscriber landmarkSub; // Landmark subscriber


// Define ROS publisher
ros::Publisher state_publisher; // State publisher

// Callback function for encoder data
void encoderCallback(const geometry_msgs::Vector3Stamped::ConstPtr& msg)
{
    std_msgs::Float64MultiArray state_msg;

    // Check if encoder_prev_time_stamp is zero
    if (encoder_prev_time_stamp.isZero()) 
    {
        encoder_prev_time_stamp = msg->header.stamp; // Initialize encoder_prev_time_stamp
        return;
    }

    // Calculate time difference
    ros::Time encoder_current_time_stamp = msg->header.stamp;
    dt_encoder = (encoder_current_time_stamp - encoder_prev_time_stamp).toSec();
    // Store encoder measurements
    encoder_measurement[0] = (msg->vector.x)* 0.1047;
    encoder_measurement[1] = (msg->vector.y)* 0.1047;
    // Call UKF encoder callback function
    ukf.encoder_callback(encoder_measurement, dt_encoder,yaw);
    // Update encoder_prev_time_stamp
    encoder_prev_time_stamp = encoder_current_time_stamp;
    
    // Publish state message
    state_msg.data = {ukf.x_post[0], ukf.x_post[1], ukf.x_post[2], ukf.x_post[3], ukf.x_post[4], ukf.x_post[5], ukf.x_post[6], ukf.x_post[7], ukf.x_post[8]};
    state_publisher.publish(state_msg);
}

// Callback function for GPS data
void gpsCallback(const geometry_msgs::Vector3Stamped::ConstPtr& msg)
{
    std_msgs::Float64MultiArray state_msg;

    if (initial_measurement == true)
    {
        lat0 = msg->vector.x; // Initialize lat0
        lon0 = msg->vector.y; // Initialize lon0
        initial_measurement = false;
    }

    // Store GPS measurements
    z_measurement[9] = msg->vector.x;
    z_measurement[10] = msg->vector.y;

    // Call UKF GPS callback function
    ukf.gps_callback(z_measurement, lon0, lat0);

    // Publish state message
    state_msg.data = {ukf.x_post[0], ukf.x_post[1], ukf.x_post[2], ukf.x_post[3], ukf.x_post[4], ukf.x_post[5], ukf.x_post[6], ukf.x_post[7], ukf.x_post[8]};
    state_publisher.publish(state_msg);
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
    std_msgs::Float64MultiArray state_msg;

    tf2::Quaternion quat (msg->orientation.w, msg->orientation.x, msg->orientation.y, msg->orientation.z);
    tf2::Matrix3x3 m(quat);
    m.getRPY(roll, pitch, yaw);

    ukf.imu_callback(z_measurement, dt_imu);
    imu_prev_time_stamp = imu_current_time_stamp;

    // Publish state message
    state_msg.data = {ukf.x_post[0], ukf.x_post[1], ukf.x_post[2], ukf.x_post[3], ukf.x_post[4], ukf.x_post[5], ukf.x_post[6], ukf.x_post[7], ukf.x_post[8]};
    state_publisher.publish(state_msg);
}

void bnoCallback(const sensor_msgs::Imu::ConstPtr& msg)
{
    std_msgs::Float64MultiArray state_msg;

    tf2::Quaternion quat (msg->orientation.w, msg->orientation.x, msg->orientation.y, msg->orientation.z);
    tf2::Matrix3x3 m(quat);
    m.getRPY(roll, pitch, yaw);
    // (yaw = msg->orientation.z;)
    // yaw = yaw * PI / 180.0;
    // pitch = msg->orientation.y; //????????????
    // pitch = pitch * PI / 180.0;  //???????????
    ukf.bno_callback(roll, pitch, yaw);

    // Publish state message
    state_msg.data = {ukf.x_post[0], ukf.x_post[1], ukf.x_post[2], ukf.x_post[3], ukf.x_post[4], ukf.x_post[5], ukf.x_post[6], ukf.x_post[7], ukf.x_post[8]};
    state_publisher.publish(state_msg);
}

void trueLandmarkCallback(const roar_msgs::LandmarkArray::ConstPtr& msg){

    //Set the landmarks
    ROVlandmarks = msg->landmarks;
}

void landmarkCallback(const roar_msgs::Landmark::ConstPtr& landmark_poses) {

    //Calculate the rover position
    //Rover's position: P
	//Landmark TRUE position: rov.landmarks
	//Landmark RELATIVE position: landmark_poses

    if(ROVlandmarks.size() > 0){
        z_measurement[11] = ROVlandmarks[(landmark_poses->id)-1].pose.pose.position.x - landmark_poses->pose.pose.position.x;
        z_measurement[12] = ROVlandmarks[(landmark_poses->id)-1].pose.pose.position.y - landmark_poses->pose.pose.position.y;
        z_measurement[13] = ROVlandmarks[(landmark_poses->id)-1].pose.pose.position.z - landmark_poses->pose.pose.position.z;

        ukf.LL_Callback(z_measurement);
    }

}

// Main function
int main(int argc, char **argv) 
{
    ros::init(argc, argv, "ukf_localization"); // Initialize ROS node
    ros::NodeHandle nh; // Create ROS node handle
    
    // Initialize ROS subscribers
    imu_sub = nh.subscribe("/sensors/imu", 1000, bnoCallback);
    encoder_sub = nh.subscribe("/sensors/encoders", 1000, encoderCallback);
    gps_sub = nh.subscribe("/sensors/gps", 1000, gpsCallback);
  
    trueLandmarkSub = nh.subscribe("landmarkTruePoses", 1000, trueLandmarkCallback);
    landmarkSub = nh.subscribe("landmarkPoses", 1000, landmarkCallback);

    // Initialize ROS publisher
    state_publisher = nh.advertise<std_msgs::Float64MultiArray>("/filtered_state", 1000);
    ros::Rate loop_rate(10); // Set loop rate

    // Main ROS loop
    while (ros::ok())
    {
        ros::spinOnce(); // Process callbacks
    }
    return 0;
}