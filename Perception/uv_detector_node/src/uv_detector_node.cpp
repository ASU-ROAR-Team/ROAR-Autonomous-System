/**
 * @file uv_detector_node.cpp
 * @brief ROS node implementation for UV-based object detection and tracking
 * @defgroup uv_detector UV Detector ROS Node
 * @{
 * 
 * This node interfaces with a RealSense depth camera to perform object detection
 * and tracking using the UV detector algorithm. It subscribes to depth image data,
 * processes it through the UV detector pipeline, and publishes visualization markers
 * for RViz display.
 */

#include <ros/ros.h>
#include <image_transport/image_transport.h>
#include <opencv2/opencv.hpp>
#include <cv_bridge/cv_bridge.h>
#include <sensor_msgs/Image.h>
#include <visualization_msgs/Marker.h>
#include <sstream>
#include <math.h>
#include <vector>
#include <time.h>
#include <UV_detector.h>
#include <kalman_filter.h>
#include <Eigen/Dense>

using namespace cv; 
using namespace std;

/**
 * @brief Main detector class that handles ROS integration and UV detection pipeline
 * 
 * This class manages:
 * - ROS topic subscriptions and publications
 * - Depth image processing
 * - UV detection and tracking pipeline
 * - Visualization of results
 */
class my_detector
{  
	public:  
        /**
         * @brief Constructor - initializes ROS communications
         * 
         * Sets up:
         * - Image transport subscriber for depth images
         * - Publisher for visualization markers
         */
		my_detector()  
		{  
			image_transport::ImageTransport it(nh);
			//Topic subscribed 
			// depsub = it.subscribe("/camera/aligned_depth_to_color/image_raw", 1, &my_detector::run, this);
			depsub = it.subscribe("/zed2i/zed_node/depth/depth_registered", 1, &my_detector::run, this);
			marker_pub = nh.advertise<visualization_msgs::Marker>("visualization_marker", 1);
		}  

        /**
         * @brief Main callback for processing depth images
         * @param msg Incoming depth image message
         * 
         * Processing pipeline:
         * 1. Converts ROS image to OpenCV format
         * 2. Runs UV detection
         * 3. Performs object tracking
         * 4. Updates visualizations
         * 5. Publishes detection results
         */
    void run(const sensor_msgs::ImageConstPtr& msg)  
		{  
			// Convert ROS image message to OpenCV format
			//Realsense
			//cv_bridge::CvImagePtr cv_ptr = cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::TYPE_16UC1);
			
			// ZED 2i
			cv_bridge::CvImagePtr cv_ptr = cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::TYPE_32FC1);
			
			cv::Mat depth = cv_ptr->image;

			// Check if zed camera
			if(depth.type() == CV_32FC1)
			{
				depth.convertTo(depth, CV_16UC1, 1000.0);
			}
			 // Check if RealSense camera
			else if(depth.type() == CV_16UC1)
			{
				// Do nothing
			}
			else
			{
				cerr << "Unsupported depth image type: " << depth.type() << endl;
				return;
			}

			// Run detection pipeline
			this->uv_detector.readdata(depth);
			this->uv_detector.detect();
			this->uv_detector.track();

			// Update visualizations
			this->uv_detector.display_depth();
			this->uv_detector.display_U_map();
			this->uv_detector.display_bird_view();

			// Output detection results
			cout << this->uv_detector.bounding_box_B.size() << endl;

			// Process detections for RViz visualization
			for(int i = 0; i < this->uv_detector.bounding_box_B.size(); i++)
			{
				Point2f obs_center = Point2f(this->uv_detector.bounding_box_B[i].x + this->uv_detector.bounding_box_B[i].width,
											this->uv_detector.bounding_box_B[i].y + this->uv_detector.bounding_box_B[i].height);
				cout << obs_center << endl;
			}
		}

	private:  
		ros::NodeHandle nh;   		        // ROS node handle
    	image_transport::Subscriber depsub;	// Subscriber for depth images
		UVdetector uv_detector;              // UV detector instance
		ros::Publisher marker_pub;           // Publisher for visualization markers
};

/**
 * @brief Main entry point for the UV detector node
 * @param argc Number of command line arguments
 * @param argv Command line arguments
 * @return Exit status
 */
int main(int argc, char **argv)  
{  
	// Initialize ROS node
	ros::init(argc, argv, "my_realsense_recorder");  

	// Create detector instance
	my_detector SAPObject; 

	// Spin ROS event loop
	ros::spin();  
	return 0;  
} 

/** @} */ // end of uv_detector group
