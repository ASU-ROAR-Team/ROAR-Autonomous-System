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

ros::NodeHandle* nh_ptr = nullptr;

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
XmlRpc::XmlRpcValue landmarks;
XmlRpc::XmlRpcValue iPose;
Eigen::Vector3d initialPosition;
tf2::Quaternion initialOrientation;
tf2::Quaternion IMUorientation;
Eigen::Vector3d CAMERAwrtGPS;
int noOfFails = 0;
double Rgps = 5.0; // GPS noise
double RLL = 10.0; // Landmark noise
double LLMax = 0.3; //allowable range in meters
double MaxRelativeDistance = 7.0; // Maximum relative distance in meters

int noOfPass = 0;
int noOfdone = 0;

Eigen::VectorXd planBstate = Eigen::VectorXd::Zero(9); // State for plan B [oientation (w, x, y, z), angular velocity (x, y, z), position (lat, lon)] 

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
    ROS_DEBUG("[*] Entering rotateXYZbyXYZW function");
    // Convert tf2 quaternion to Eigen quaternion
    Eigen::Quaterniond q(rover_quat.x(), rover_quat.y(), rover_quat.z(), rover_quat.w());
    // Get rotation matrix
    Eigen::Matrix3d R = q.toRotationMatrix();

    //std::cout << "[+] R: \n" << R << std::endl;
    ROS_DEBUG("[+] Transforming relative position by rover orientation");
    // Transform the vector
    return R * rel_pos_rover;
}

//Pose callback function
void poseCallback(bool rotate = true){
    ROS_DEBUG("[*] Entering poseCallback function");
    // Create a TransformStamped message to publish the pose
    static tf2_ros::TransformBroadcaster br;
    geometry_msgs::TransformStamped transformStamped;
    
    Eigen::Vector3d rotatedXYZ;

    if(rotate){
        rotatedXYZ = rotateXYZbyXYZW({ukf.x_post[7], ukf.x_post[8], 0}, initialOrientation) + initialPosition;
    } else {
        rotatedXYZ(0) = ukf.x_hat[7];
	    rotatedXYZ(1) = ukf.x_hat[8];
    }
    /*******
    TALYEESA
    *******/
    //rotatedXYZ[0] = -rotatedXYZ[0]; // Invert x-coordinate for Gazebo world

    transformStamped.header.stamp = ros::Time::now();
    transformStamped.header.frame_id = "world";
    transformStamped.child_frame_id = "dummy_root";
    transformStamped.transform.translation.x = rotatedXYZ[0];
    transformStamped.transform.translation.y = rotatedXYZ[1];
    transformStamped.transform.translation.z = 0.0;

    tf2::Quaternion q(ukf.x_post[1], ukf.x_post[2], ukf.x_post[3], ukf.x_post[0]);
    transformStamped.transform.rotation.x = q.x();
    transformStamped.transform.rotation.y = q.y();
    transformStamped.transform.rotation.z = q.z();
    transformStamped.transform.rotation.w = q.w();  
    // Publish the transform
    ROS_DEBUG("[+] Publishing transform from world to base_link");
    br.sendTransform(transformStamped);
    ROS_DEBUG("[*] Exiting poseCallback function");
}

void planB(){
    ROS_WARN("[!] CALLED PLAN B");
    noOfFails++;
    ROS_WARN("[!] Current number of fails: %d", noOfFails);
    ukf.planBCallback(planBstate, lat0, lon0);
    //Add the state of GPS on/off to decide which plan to use or add a parameter for this plan B 
    //Store the state of the rover with the covariences in a queue of 20 elements
    // If the state is not finite, use the last known valid state where the covarience is logical and the position is within a range
    //or the position deference between it and the previous is not more than 0.5 meters
}

void publishState(bool showInPlotter = false, bool rotate = true)
{
    ROS_DEBUG("[*] Entering publishState function");
    // Create a state message to publish
    // Initialize state message
    Eigen::Vector3d rotatedXYZ;
    state_msg.header.stamp = ros::Time::now();
    state_msg.header.frame_id = "world";
    state_msg.child_frame_id = "dummy_root";

    if(!ukf.x_post.allFinite()) {
        ROS_ERROR("[!] State Space x_post is not finite");
        /*
        TAKE GPS VALUES AND BNO VALUES AS THE NEW ITITIAL STATE "make an array that stores their latest values to take them"
        */
        planB(); // Call plan B if state is not finite
    } 
    else {
        ROS_DEBUG("[+] Transforming state to world frame");
        // Transform to world frame
	if(rotate){
            rotatedXYZ = rotateXYZbyXYZW({ukf.x_post[7], ukf.x_post[8], 0}, initialOrientation) + initialPosition;
        } else {
	    std::cout << "Landmark in now publishing\n";
	    rotatedXYZ(0) = ukf.x_hat[7];
	    rotatedXYZ(1) = ukf.x_hat[8];
        Eigen::Vector3d gpsFrame;
        ROS_DEBUG("[*] Entering inverse rotation");
        // Convert tf2 quaternion to Eigen quaternion
        Eigen::Quaterniond q(initialOrientation.x(), initialOrientation.y(), initialOrientation.z(), initialOrientation.w());
        // Get rotation matrix
        Eigen::Matrix3d R = q.toRotationMatrix();

        //std::cout << "[+] R: \n" << R << std::endl;
        ROS_DEBUG("[+] Transforming abs position by relative orientation");
        // Transform the vector
        Eigen::Vector3d llestimated(ukf.x_hat[7], ukf.x_hat[8], 0);
        gpsFrame = R.transpose() * (llestimated-initialPosition);
        ukf.x_post(7) = gpsFrame(0);
        ukf.x_post(8) = gpsFrame(1);
	}
        /*******
        TALYEESA
        *******/
        //rotatedXYZ[0] = -rotatedXYZ[0]; // Invert x-coordinate for Gazebo world

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

	std::cout << "published X: " << state_msg.pose.pose.position.x << " || published Y: " << state_msg.pose.pose.position.y << std::endl; 

        nh_ptr->setParam("/rover/position/x", rotatedXYZ[0]);
        nh_ptr->setParam("/rover/position/y", rotatedXYZ[1]);
        nh_ptr->setParam("/rover/position/z", 0.0); // Assuming z is always 0 in this context
        nh_ptr->setParam("/rover/orientation/w", ukf.x_post[0]);
        nh_ptr->setParam("/rover/orientation/x", ukf.x_post[1]);
        nh_ptr->setParam("/rover/orientation/y", ukf.x_post[2]);
        nh_ptr->setParam("/rover/orientation/z", ukf.x_post[3]);
    }

    ROS_WARN("[!] Current number of fails: %d", noOfFails);
    ROS_DEBUG("[*] Publishing state message to /filtered_state topic");
    // Print state for debugging
    ROS_INFO("[*] Current state:");
    std::cout << "Orientation: " << ukf.x_post[0] << ", " << ukf.x_post[1] << ", " << ukf.x_post[2] << ", " << ukf.x_post[3] << std::endl;
    std::cout << "Position: " << rotatedXYZ[0] << ", " << rotatedXYZ[1] << std::endl;
    std::cout << "Linear Velocity: " << ukf.x_post[4] << ", " << ukf.x_post[5] << ", "  << ukf.x_post[6] << std::endl;
    
    if(!ukf.P_post.allFinite()) {
        ROS_ERROR("[!] Covariance matrix P is not finite, setting to zero");
        state_msg.pose.covariance.fill(0.0);
        planB(); // Call plan B if state is not finite
    } 
    else {
        // Publish covariance
        state_msg.pose.covariance[0] = ukf.P_post.col(7)(7);
        state_msg.pose.covariance[1] = ukf.P_post.col(8)(7);
        state_msg.pose.covariance[6] = ukf.P_post.col(7)(8);
        state_msg.pose.covariance[7] = ukf.P_post.col(8)(8);
        state_msg.pose.covariance[2] = ukf.P_post.col(0)(0);
        state_msg.pose.covariance[3] = ukf.P_post.col(1)(1);
        state_msg.pose.covariance[4] = ukf.P_post.col(2)(2);
        state_msg.pose.covariance[5] = ukf.P_post.col(3)(3);
    }

    if (showInPlotter) {
        state_msg.pose.covariance[35] = 1;
    } else {
        state_msg.pose.covariance[35] = 0;
    }
    // Publish the message
    poseCallback(rotate); // Call pose callback to publish the transform
    state_publisher.publish(state_msg);
    ROS_DEBUG("[+] Published state message to /filtered_state topic");
    std::cout << "final Published x: " << state_msg.pose.pose.position.x << " | final Published y: " << state_msg.pose.pose.position.y << std::endl;
    ROS_DEBUG("[*] Exiting publishState function");
}

// Callback function for encoder data
void encoderCallback(const sensor_msgs::JointState::ConstPtr& msg)
{
    ROS_DEBUG("[*] Entering encoderCallback function");
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
        ROS_DEBUG("[+] Calculating time difference for encoder measurements");
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
        ROS_DEBUG("[*] Exiting encoderCallback function");
    }
    
    
}

// Callback function for GPS data
void gpsCallback(const sensor_msgs::NavSatFix::ConstPtr& msg)
{
    ROS_DEBUG("[*] Entering gpsCallback function");
    // Check if prev_time_stamp is zero
    if (initial_measurement == true)
    {
        lat0 = msg->latitude; // Initialize lat0
        lon0 = msg->longitude; // Initialize lon0
        initial_measurement = false;
        /*
        ROS_DEBUG("[+] Initializing GPS reference coordinates");
        // Initialize lat0 and lon0 with the first GPS measurement
        if (nh_ptr->hasParam("/rover/initial/lat") && nh_ptr->hasParam("/rover/initial/lon")) {
            ROS_DEBUG("[+] Initial GPS coordinates set successfully");
            XmlRpc::XmlRpcValue initial;
            nh_ptr->getParam("/rover/initial", initial);
            lat0 = static_cast<double>(initial["lat"]);
            lon0 = static_cast<double>(initial["lon"]);
            initial_measurement = false;
            ROS_DEBUG("Loaded initial GPS readings from parameter server.");
        } else {
            ROS_DEBUG("[+] Initializing initial GPS coordinates");
            lat0 = msg->latitude; // Initialize lat0
            lon0 = msg->longitude; // Initialize lon0
            initial_measurement = false;
            nh_ptr->setParam("/rover/initial/lat", lat0);
            nh_ptr->setParam("/rover/initial/lon", lon0);
        }
            */
    }

    // Store GPS measurements
    z_measurement[9] = msg->latitude;
    z_measurement[10] = msg->longitude;

    planBstate(7) = msg->latitude;
    planBstate(8) = msg->longitude;


    // Call UKF GPS callback function
    ukf.gps_callback(z_measurement, lon0, lat0, Rgps);

    publishState(true); // Publish the state message
    ROS_DEBUG("[*] Exiting gpsCallback function");
}

// Callback function for IMU data
void imuCallback(const sensor_msgs::Imu::ConstPtr& msg)
{
    ROS_DEBUG("[*] Entering imuCallback function");
    // Check if prev_time_stamp is zero
    // yaw = msg->orientation.z;
    // yaw = yaw * PI / 180.0;
    if (prev_time_stamp.isZero()) 
    {
        ROS_DEBUG("[+] Initializing IMU timestamp");
        // Initialize prev_time_stamp with the first IMU measurement
        prev_time_stamp = msg->header.stamp; // Initialize prev_time_stamp
        return;
    }

    ROS_DEBUG("[+] IMU Callback: Calculating time difference");
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

    ROS_DEBUG("[+] IMU Callback: Storing IMU Orientation");
    // Store IMU measurements
    planBstate(0) = msg->orientation.w;
    planBstate(1) = msg->orientation.x;
    planBstate(2) = msg->orientation.y;
    planBstate(3) = msg->orientation.z;
    ROS_DEBUG("[+] IMU Callback: Storing IMU anglar velocity");
    planBstate(4) = msg->angular_velocity.x;
    planBstate(5) = msg->angular_velocity.y;
    planBstate(6) = msg->angular_velocity.z;

    ukf.imu_callback(encoder_measurement, z_measurement, dt_imu, roll, yaw, pitch); //////////need to be fixed
    prev_time_stamp = current_time_stamp;

    publishState(); // Publish the state message
    ROS_DEBUG("[*] Exiting imuCallback function");
}

void landmarkCallback(const roar_msgs::Landmark::ConstPtr& landmark_poses) {

    //Calculate the rover position
    //Rover's position: P
	//Landmark TRUE position
	//Landmark RELATIVE position: landmark_poses
    ROS_DEBUG("[*] Entering landmarkCallback function");
    double rovCurrentX;
    double rovCurrentY;
    nh_ptr->getParam("landmarks", landmarks);
    if(landmarks.size() > 0){

        double rel_x = landmark_poses->pose.pose.position.x;
        double rel_y = landmark_poses->pose.pose.position.z;
        double rel_z = landmark_poses->pose.pose.position.y;

        std::cout << "Landmark Relative untransformed Position: " << rel_x << ", " << rel_y << ", " << rel_z << std::endl;

        Eigen::Vector3d rel_pos_rover(rel_x, rel_y, rel_z); // add offset to the center of the rover

        // Get rover orientation as quaternion (from your UKF state or message)
        //make sure which is w and which is x y z
        tf2::Quaternion rover_quat(
            ukf.x_post[1], // x
            ukf.x_post[2], // y
            ukf.x_post[3], // z
            ukf.x_post[0]  // w
        );

        std::cout << "Rover Quaternion: " << rover_quat.x() << ", " << rover_quat.y() << ", " << rover_quat.z() << ", " << rover_quat.w() << std::endl;

        // Transform to world frame
        // Rotate the relative position by the rover's orientation
        // This will give you the position of the landmark in the world frame
        ROS_DEBUG("[+] Transforming relative position by rover orientation");
        // Use the rotateXYZbyXYZW function to get the world position
        Eigen::Vector3d rel_pos_world = rotateXYZbyXYZW(rel_pos_rover, rover_quat);

        rel_x = rel_pos_world.x() + CAMERAwrtGPS.x(); // Adjusting y position to match the gps postion "diff between camera and gps"
        rel_y = rel_pos_world.y() + CAMERAwrtGPS.y(); // Adjusting y position to match the gps postion "diff between camera and gps"

        std::cout << "Landmark Relative Transformed Position: " << rel_x << ", " << rel_y << ", " << rel_z << std::endl;

        double relativeDistance = sqrt(pow(rel_x, 2) + pow(rel_y, 2));

        double rov_x = static_cast<double>(landmarks[std::to_string(landmark_poses->id)]["x"]);
        double rov_y = static_cast<double>(landmarks[std::to_string(landmark_poses->id)]["y"]);

        std::cout << "Landmark Position: " << rov_x << ", " << rov_y << std::endl;

        std::cout << "Landmark ID: " << landmark_poses->id << std::endl;

        if(relativeDistance > MaxRelativeDistance) {
            ROS_WARN("[!] Relative distance is greater than %f meters, skipping landmark", MaxRelativeDistance);
            noOfPass++;
            ROS_DEBUG("[*] Exiting landmarkCallback function");
            return; // Skip this landmark if the relative distance is greater than 7 meters
        }
        
        //All Relative Positions 
        // 32 relative positions around the rover
        ROS_DEBUG("[+] Calculating all relative positions around the rover");
        std::vector<Eigen::Vector2d> rel_pos_all = {
            {rov_x + rel_x, rov_y + rel_y},
            {rov_x + rel_x, rov_y - rel_y},
        };

        XmlRpc::XmlRpcValue position;
        if(nh_ptr->hasParam("/rover/position")){
            nh_ptr->getParam("/rover/position", position);
            rovCurrentX = static_cast<double>(position["x"]);
            rovCurrentY = static_cast<double>(position["y"]);
        } else {
            ROS_ERROR("[!] No rover position found in parameter server");
            rovCurrentX = initialPosition.x();
            rovCurrentY = initialPosition.y();
        }
        

        Eigen::Vector2d nearestPos(rovCurrentX, rovCurrentY); // Initialize nearest position to rover's current position
        double minDist = 1000000.0;
        double dist = 0.0;
        

        // Find the nearest position to the rover
        ROS_DEBUG("[+] Finding the nearest position to the rover");
        for (int i = 0; i < 2; i++){
            //calc dist
            dist = sqrt(pow((rel_pos_all[i][0] - rovCurrentX) ,2) + pow((rel_pos_all[i][1] - rovCurrentY) ,2));

            if ((dist < minDist) && (dist < LLMax)) // Check if the distance is less than the minimum distance and less than 10 meters
            {
                minDist = dist;
                nearestPos = rel_pos_all[i];
                std::cout << "Nearest Position: " << nearestPos.transpose() << std::endl;
            }
            std::cout << "Relative Position " << i << ": " << rel_pos_all[i].transpose() << std::endl;
            std::cout << "Rover Position: " << rovCurrentX << ", " << rovCurrentY << std::endl;
            std::cout << "Distance to Rover: " << dist << std::endl;
            std::cout << "Minimum Distance: " << minDist << std::endl;
            std::cout << "----------------------------------------" << std::endl;
        }
        
        if (minDist < LLMax) { // Check if the nearest position is within 0.3 meters
            ROS_DEBUG("[+] Nearest position found within %f meters", LLMax);
            z_measurement[11] = nearestPos[0];
            z_measurement[12] = nearestPos[1];
            std::cout << "Nearest Position: " << nearestPos.transpose() << std::endl;
            std::cout << "Measurement: " << z_measurement[11] << ", " << z_measurement[12] << std::endl;
            z_measurement[13] = 0;
            ukf.LL_Callback(z_measurement, rovCurrentX, rovCurrentY, RLL); // Call UKF landmark callback function
            publishState(true, false); // Publish the state message
            noOfdone++;
        } else {
            ROS_WARN("[!] No Landmark found within %f meters", LLMax);
            noOfPass++;
        }
        
        
        ROS_DEBUG("[*] Exiting landmarkCallback function");
        std::cout << "No of skips: " << noOfPass << std::endl;
        std::cout << "No of done: " << noOfdone << std::endl;
    }

}

void magnetometerCallback(const geometry_msgs::Vector3Stamped::ConstPtr& msg)
{
    z_measurement[6] = msg->vector.x;
    z_measurement[7] = msg->vector.y;
    z_measurement[8] = msg->vector.z;

}

bool loadInitialPose(ros::NodeHandle& nh, Eigen::Vector3d& position, tf2::Quaternion& orientation, tf2::Quaternion& IMUorientation, Eigen::Vector3d& CAMERAwrtGPS) {
    XmlRpc::XmlRpcValue iPose;
    XmlRpc::XmlRpcValue init_position;
    XmlRpc::XmlRpcValue init_orientation;

    ROS_DEBUG("Loading local initial parameters");
    if (!nh.getParam("iPose", iPose)) {
        ROS_ERROR("Failed to get param 'iPose'");
        return false;
    }

    if(nh.hasParam("/rover/position") && nh.hasParam("/rover/orientation")) {
        ROS_DEBUG("[+] Loading initial pose from parameter server");
        nh.getParam("/rover/position", init_position);
        nh.getParam("/rover/orientation", init_orientation);
        
        // --- Extract position ---
        position.x() = static_cast<double>(init_position["x"]);
        position.y() = static_cast<double>(init_position["y"]);
        position.z() = static_cast<double>(init_position["z"]);
        ROS_DEBUG("[+] Initial position loaded: %f, %f, %f", position.x(), position.y(), position.z());
        // --- Extract orientation ---
        orientation = tf2::Quaternion(
            static_cast<double>(init_orientation["x"]),
            static_cast<double>(init_orientation["y"]),
            static_cast<double>(init_orientation["z"]),
            static_cast<double>(init_orientation["w"])
        );
        ROS_DEBUG("[+] Initial orientation loaded: %f, %f, %f, %f", 
                  orientation.x(), orientation.y(), orientation.z(), orientation.w());

    } else {
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

    // --- Extract position ---
    if (iPose.getType() != XmlRpc::XmlRpcValue::TypeStruct ||
        !iPose.hasMember("CAMERAwrtGPS") ||
        iPose["CAMERAwrtGPS"].getType() != XmlRpc::XmlRpcValue::TypeStruct) {
        ROS_ERROR("Invalid or missing 'CAMERAwrtGPS' in iPose param");
        return false;
    }
    try {
        CAMERAwrtGPS.x() = static_cast<double>(iPose["CAMERAwrtGPS"]["x"]);
        CAMERAwrtGPS.y() = static_cast<double>(iPose["CAMERAwrtGPS"]["y"]);
        CAMERAwrtGPS.z() = static_cast<double>(iPose["CAMERAwrtGPS"]["z"]);
    } catch (...) {
        ROS_ERROR("Failed to read 'CAMERAwrtGPS' values from iPose");
        return false;
    }
    

    return true;
}

void bnoCallback(const sensor_msgs::Imu::ConstPtr& msg)
{
    nav_msgs::Odometry state_msg;

    tf2::Quaternion quat (msg->orientation.x, msg->orientation.y, msg->orientation.z, msg->orientation.w);
    tf2::Matrix3x3 m(quat);
    m.getRPY(roll, pitch, yaw);

    ukf.bno_callback(roll, pitch, yaw);

    //publishState(); // Call pose callback to publish the transform
}

// Main function
int main(int argc, char **argv) 
{
    ROS_DEBUG("DEBUG MODE: ON");

    ros::init(argc, argv, "ukf_localization"); // Initialize ROS node
    ros::NodeHandle nh; // Create ROS node handle
    nh_ptr = &nh; // Assign the node handle to the global pointer

    XmlRpc::XmlRpcValue UKF_PARAMS;
    nh.getParam("UKF", UKF_PARAMS);

    ukf.g0 << static_cast<int>(UKF_PARAMS["g0"]["x"]), static_cast<int>(UKF_PARAMS["g0"]["y"]), static_cast<int>(UKF_PARAMS["g0"]["z"]);
    ukf.m0 << static_cast<double>(UKF_PARAMS["B_INTENSITY"]) * cos(static_cast<double>(UKF_PARAMS["INCLINATION"])),   // Magnetic Field Intensity Vector
              0.0,
              static_cast<double>(UKF_PARAMS["B_INTENSITY"])* sin(static_cast<double>(UKF_PARAMS["INCLINATION"]));
    double noiseQ = static_cast<double>(UKF_PARAMS["Q"]); // Process noise
    ukf.Q = Eigen::MatrixXd::Identity(9, 9) * noiseQ;
    double noiseR = static_cast<double>(UKF_PARAMS["R"]); // Process noise
    ukf.R = Eigen::MatrixXd::Identity(14, 14) * noiseR;

    ukf.P(0) = static_cast<double>(UKF_PARAMS["P_q0"]); // Initial covariance for quaternion
    ukf.P(1) = static_cast<double>(UKF_PARAMS["P_q1"]); // Initial covariance for quaternion
    ukf.P(2) = static_cast<double>(UKF_PARAMS["P_q2"]); // Initial covariance for quaternion
    ukf.P(3) = static_cast<double>(UKF_PARAMS["P_q3"]); // Initial covariance for quaternion
    ukf.P(4) = static_cast<double>(UKF_PARAMS["P_omega_x"]); // Initial covariance for quaternion
    ukf.P(5) = static_cast<double>(UKF_PARAMS["P_omega_y"]); // Initial covariance for quaternion
    ukf.P(6) = static_cast<double>(UKF_PARAMS["P_omega_z"]); // Initial covariance for quaternion
    ukf.P(7) = static_cast<double>(UKF_PARAMS["P_x"]); // Initial covariance for quaternion
    ukf.P(8) = static_cast<double>(UKF_PARAMS["P_y"]); // Initial covariance for quaternion
    
    Rgps = static_cast<double>(UKF_PARAMS["R_gps"]); // GPS noise
    RLL = static_cast<double>(UKF_PARAMS["RLL"]); // Landmark noise
    LLMax = static_cast<double>(UKF_PARAMS["LLMax"]); // Landmark max distance allowable
    MaxRelativeDistance = static_cast<double>(UKF_PARAMS["MaxRelativeDistance"]); // Landmark max distance allowable
    
    ROS_DEBUG("UKF parameters loaded successfully");
    // Initialize ROS subscribers
    if(static_cast<bool>(UKF_PARAMS["GPS_State"])) {
        ROS_DEBUG("GPS State is enabled");
        gps_sub = nh.subscribe("/gps", 1000, gpsCallback);
    } else {
        ROS_DEBUG("GPS State is disabled");
    }
    if(static_cast<bool>(UKF_PARAMS["IMU_State"])) {
        ROS_DEBUG("IMU State is enabled");
        imu_sub = nh.subscribe("/imu", 1000, imuCallback);
    } else {
        ROS_DEBUG("IMU State is disabled");
    }
    if(static_cast<bool>(UKF_PARAMS["ENCODER_State"])) {
        ROS_DEBUG("Encoder State is enabled");
        encoder_sub = nh.subscribe("/joint_states", 1000, encoderCallback);
    } else {
        ROS_DEBUG("Encoder State is disabled");
    }
    if(static_cast<bool>(UKF_PARAMS["Landmark_State"])) {
        ROS_DEBUG("Landmark State is enabled");
        landmarkSub = nh.subscribe("/landmark_topic", 1000, landmarkCallback);
    } else {
        ROS_DEBUG("Landmark State is disabled");
    }
    if(static_cast<bool>(UKF_PARAMS["BNO_State"])) {
        ROS_DEBUG("BNO State is enabled");
        imu_sub = nh.subscribe("/imu", 1000, bnoCallback);
    } else {
        ROS_DEBUG("Landmark State is disabled");
    }

    if (loadInitialPose(nh, initialPosition, initialOrientation, IMUorientation, CAMERAwrtGPS)) {
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
        ROS_INFO_STREAM("CAMERA Frame w.r.t IMU: " << CAMERAwrtGPS.transpose());
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
