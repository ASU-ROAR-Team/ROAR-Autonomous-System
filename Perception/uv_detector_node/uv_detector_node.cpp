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
 * @brief ROS node wrapper for UV detection
 * 
 * This class manages:
 * - ROS topic subscriptions and publications
 * - Depth image processing
 * - UV detection and tracking pipeline
 * - Visualization of results
 */
class UvDetectorNode
{  
	public:  
        /**
         * @brief Constructor - initializes ROS communications
         * 
         * Sets up:
         * - Image transport subscriber for depth images
         * - Publisher for visualization markers
         */
		UvDetectorNode()  
		{  
			image_transport::ImageTransport it(nodeHandle);
			//Topic subscribed 
			depthSub = it.subscribe("/camera/aligned_depth_to_color/image_raw", 1, &UvDetectorNode::run, this);
			markerPub = nodeHandle.advertise<visualization_msgs::Marker>("visualization_marker", 1);
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
			cv_bridge::CvImagePtr cvPtr = cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::TYPE_16UC1);
			cv::Mat depth = cvPtr->image;

			// Run detection pipeline
			this->uvDetector.readData(depth);
			this->uvDetector.detect();
			this->uvDetector.track();

			// Update visualizations
			this->uvDetector.displayDepth();
			this->uvDetector.displayUMap();
			this->uvDetector.displayBirdView();

			// Output detection results
			cout << this->uvDetector.boundingBoxB.size() << endl;

			// Process detections for RViz visualization
			for(int i = 0; i < this->uvDetector.boundingBoxB.size(); i++)
			{
				// Calculate 2D center point of the bounding box
				Point2f center2D = Point2f(this->uvDetector.boundingBoxB[i].x + this->uvDetector.boundingBoxB[i].width/2,
										this->uvDetector.boundingBoxB[i].y + this->uvDetector.boundingBoxB[i].height/2);
				
				// Get depth value at the center point
				float depthValue = depth.at<uint16_t>(center2D.y, center2D.x) / 1000.0f; // Convert from mm to meters
				
				// Create 3D point (x, y, z)
				Point3f obsCenter3D(center2D.x, center2D.y, depthValue);
				
				cout << "3D Center Point: " << obsCenter3D << endl;
			}
		}

	private:  
		ros::NodeHandle nodeHandle;   		        ///< ROS node handle
    	image_transport::Subscriber depthSub;	    ///< Subscriber for depth images
		UVdetector uvDetector;                      ///< UV detector instance
		ros::Publisher markerPub;                   ///< Publisher for visualization markers
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
	UvDetectorNode sapObject; 

	// Spin ROS event loop
	ros::spin();  
	return 0;  
} 

/** @} */ // end of uv_detector group
