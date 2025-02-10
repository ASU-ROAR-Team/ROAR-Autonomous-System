#!/usr/bin/env python3
import rospy
import cv2
import numpy as np
from sensor_msgs.msg import Image
from std_msgs.msg import String
from cv_bridge import CvBridge
import cv2.aruco as aruco

# Define the dictionary for ArUco
ARUCO_DICT = {
    "DICT_4X4_50": aruco.DICT_4X4_50,
    "DICT_5X5_100": aruco.DICT_5X5_100,
    # You can add more dictionaries as needed
}

# Initialize the CvBridge
bridge = CvBridge()

# Publisher for ArUco tag data
aruco_pub = rospy.Publisher('/aruco_tags', String, queue_size=10)

def aruco_display(corners, ids, image):
    if len(corners) > 0:
        ids = ids.flatten()
        for (markerCorner, markerID) in zip(corners, ids):
            corners = markerCorner.reshape((4, 2))
            (topLeft, topRight, bottomRight, bottomLeft) = corners

            # Draw the marker's bounding box
            cv2.line(image, tuple(topLeft), tuple(topRight), (0, 255, 0), 2)
            cv2.line(image, tuple(topRight), tuple(bottomRight), (0, 255, 0), 2)
            cv2.line(image, tuple(bottomRight), tuple(bottomLeft), (0, 255, 0), 2)
            cv2.line(image, tuple(bottomLeft), tuple(topLeft), (0, 255, 0), 2)

            # Find the center of the marker
            cX = int((topLeft[0] + bottomRight[0]) / 2.0)
            cY = int((topLeft[1] + bottomRight[1]) / 2.0)
            cv2.circle(image, (cX, cY), 4, (0, 0, 255), -1)

            # Display the marker ID
            cv2.putText(image, str(markerID), (topLeft[0], topLeft[1] - 10), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 255, 0), 2)
            rospy.loginfo(f"Detected ArUco marker ID: {markerID}")
            
            # Publish the detection information (marker ID and position)
            aruco_pub.publish(f"ArUco Marker ID: {markerID}, Position: ({cX}, {cY})")
            
    return image

def image_callback(image_data):
    try:
        # Convert ROS image message to OpenCV image
        cv_image = bridge.imgmsg_to_cv2(image_data, "bgr8")

        # Define the ArUco dictionary and parameters
        aruco_dict = aruco.Dictionary_get(ARUCO_DICT["DICT_5X5_100"])  # Choose your dictionary
        aruco_params = aruco.DetectorParameters_create()

        # Detect ArUco markers in the image
        corners, ids, rejected = aruco.detectMarkers(cv_image, aruco_dict, parameters=aruco_params)

        # Display and publish the results
        cv_image = aruco_display(corners, ids, cv_image)

        # Show the image with detected markers
        cv2.imshow("ArUco Detection", cv_image)
        cv2.waitKey(1)

    except Exception as e:
        rospy.logerr(f"Error processing image: {e}")

def aruco_tracking_node():
    # Initialize the ROS node
    rospy.init_node('aruco_tracking_node', anonymous=True)

    # Subscribe to the camera topic (replace with your topic if needed)
    rospy.Subscriber('/camera/color/image_raw', Image, image_callback)

    # Keep the node running
    rospy.spin()

if __name__ == '__main__':
    try:
        aruco_tracking_node()
    except rospy.ROSInterruptException:
        pass
    finally:
        cv2.destroyAllWindows()