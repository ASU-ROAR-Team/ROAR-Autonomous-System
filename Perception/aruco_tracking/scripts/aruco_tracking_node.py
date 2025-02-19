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
    "DICT_5X5_250": aruco.DICT_5X5_250,
    "DICT_6X6_50": aruco.DICT_6X6_50,
    "DICT_6X6_100": aruco.DICT_6X6_100,
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
            topLeft = tuple(int(x) for x in topLeft)
            topRight = tuple(int(x) for x in topRight)
            bottomRight = tuple(int(x) for x in bottomRight)
            bottomLeft = tuple(int(x) for x in bottomLeft)

            cv2.line(image, topLeft, topRight, (0, 255, 0), 2)
            cv2.line(image, topRight, bottomRight, (0, 255, 0), 2)
            cv2.line(image, bottomRight, bottomLeft, (0, 255, 0), 2)
            cv2.line(image, bottomLeft, topLeft, (0, 255, 0), 2)

            # Find the center of the marker
            cX = int((topLeft[0] + bottomRight[0]) / 2.0)
            cY = int((topLeft[1] + bottomRight[1]) / 2.0)
            cv2.circle(image, (cX, cY), 4, (0, 0, 255), -1)

            # Display the marker ID
            cv2.putText(image, str(markerID), (topLeft[0], topLeft[1] - 10), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 255, 0), 2)
            rospy.loginfo(f"Detected ArUco marker ID: {markerID}")

    return image

def image_callback(image_data):
    cv_image = bridge.imgmsg_to_cv2(image_data, "bgr8")

    # Use the correct method for dictionary retrieval
    aruco_dict = cv2.aruco.getPredefinedDictionary(aruco.DICT_5X5_100)
    
    # Initialize the DetectorParameters
    aruco_params = cv2.aruco.DetectorParameters()

    # Detect ArUco markers in the image
    corners, ids, rejected = cv2.aruco.detectMarkers(cv_image, aruco_dict, parameters=aruco_params)

    if ids is not None:
        # Display the markers in the image
        cv_image = aruco_display(corners, ids, cv_image)

    # Show the image with detected markers
    cv2.imshow("ArUco Detection", cv_image)
    cv2.waitKey(1)


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
