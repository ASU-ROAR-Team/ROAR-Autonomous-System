#!/usr/bin/env python3
# pylint: disable=all
# mypy: ignore-errors
"""
Node for aruco marker
"""
import rospy
import cv2
import tf
import numpy as np
from sensor_msgs.msg import Image, CameraInfo
from geometry_msgs.msg import PoseStamped
from roar_msgs.msg import LandmarkArray, Landmark
from cv_bridge import CvBridge
from aruco_tracking import ArucoTracker


class ArucoTrackingNode:
    """
    A ROS node for detecting and tracking ArUco markers in a camera stream.

    Attributes
    ----------
    bridge : CvBridge
        Object for converting ROS Image messages to OpenCV images.
    arucoTracker : ArucoTracker
        Object for detecting ArUco markers.
    cameraMatrix : Optional[np.ndarray]
        The camera matrix for visualization.
    distCoeffs : Optional[np.ndarray]
        The distortion coefficients for visualization.
    visualize : bool
        Whether to visualize the detected markers.
    showRejected : bool
        Whether to show rejected marker candidates.
    posePub : rospy.Publisher
        Publisher for PoseStamped messages.
    """

    def __init__(self):
        """
        Initializes the ArucoTrackingNode instance.
        
        This constructor sets up the ROS node ("arucoTrackingNode") and initializes core
        components for ArUco marker detection. It creates a CvBridge for image conversion,
        loads configuration parameters from the ROS parameter server, instantiates the
        ArucoTracker with the specified dictionary type, and initializes camera parameters.
        Additionally, it subscribes to camera image and camera info topics and sets up a
        publisher for broadcasting detected marker poses.
        """
        rospy.init_node("arucoTrackingNode", anonymous=True)
        self.bridge = CvBridge()

        # Load parameters
        self.loadParameters()

        # Initialize the ArUco tracker
        self.arucoTracker = ArucoTracker(self.arucoDictType)

        # Initialize camera matrix and distortion coefficients
        self.cameraMatrix = None
        self.distCoeffs = None

        # Subscribers
        rospy.Subscriber(self.cameraTopic, Image, self.imageCallback)
        rospy.Subscriber(self.cameraInfoTopic, CameraInfo, self.cameraInfoCallback)

        # Publishers
        self.posePub = rospy.Publisher("/landmark_topic", PoseStamped, queue_size=10)

    def loadParameters(self):
        """
        Load and validate ROS parameters for the ArucoTrackingNode.
        
        This method retrieves configuration parameters from the ROS parameter server, including the topics
        for camera images and camera info, the ArUco dictionary type, visualization options, and marker size.
        Default values are used when parameters are not explicitly set. If the mandatory camera topic is
        missing, an error is logged and the node is signaled to shut down.
        """
        self.cameraTopic = rospy.get_param("~cameraTopic", "/camera/color/image_raw")
        self.cameraInfoTopic = rospy.get_param("~cameraInfoTopic", "/camera/color/camera_info")
        self.arucoDictType = rospy.get_param("~arucoDictType", cv2.aruco.DICT_5X5_10)
        self.visualize = rospy.get_param("~visualize", True)
        self.showRejected = rospy.get_param("~showRejected", True)
        self.markerSize = rospy.get_param("~markerSize", 0.05)

        # Validate mandatory parameters
        if self.cameraTopic is None:
            rospy.logerr("Camera topic not specified. Shutting down.")
            rospy.signal_shutdown("Camera topic not specified.")

    def imageCallback(self, imageMsg: Image):
        """
        Processes a ROS Image message to detect ArUco markers and publish their poses.
        
        This callback converts the incoming ROS Image to an OpenCV image using CvBridge, then detects
        ArUco markers with an ArUco tracker. For each detected marker, it estimates the pose using the
        configured marker size and camera calibration parameters, draws the markers (and axes) on the image,
        and creates a Landmark message containing a PoseStamped. The landmark pose is then published.
        If enabled, the function also draws rejected marker candidates and visualizes the processed image.
          
        Parameters:
            imageMsg (Image): Incoming ROS Image message.
        """
        # try:
        cvImage = self.bridge.imgmsg_to_cv2(imageMsg, "bgr8")
        corners, ids, rejected = self.arucoTracker.detectMarkers(cvImage)
        if ids is not None:
            # Estimate pose
            rvecs, tvecs = self.arucoTracker.estimatePose(
                corners, self.markerSize, self.cameraMatrix, self.distCoeffs
            )
            print(rvecs[0][1], tvecs[0][1])
            # Draw markers, axes, and pose information
            cvImage = self.arucoTracker.drawMarkers(
                cvImage, corners, ids, rvecs, tvecs, self.cameraMatrix, self.distCoeffs
            )

            # Publish pose for each marker
            landmarkArrMsg = LandmarkArray()
            landmarkArrMsg.header = imageMsg.header
            for i in range(len(ids)):
                print(i)
                rvec = rvecs[i].flatten()
                tvec = tvecs[i].flatten()

                # Convert rotation vector to quaternion
                rotationMatrix, _ = cv2.Rodrigues(rvec)
                quaternion = tf.transformations.quaternion_from_matrix(
                    np.vstack([np.hstack([rotationMatrix, [[0], [0], [0]]]), [0, 0, 0, 1]])
                )

                # Create PoseStamped message
                poseMsg = PoseStamped()
                poseMsg.header = imageMsg.header
                poseMsg.pose.position.x = tvec[0]
                poseMsg.pose.position.y = tvec[1]
                poseMsg.pose.position.z = tvec[2]
                poseMsg.pose.orientation.x = quaternion[0]
                poseMsg.pose.orientation.y = quaternion[1]
                poseMsg.pose.orientation.z = quaternion[2]
                poseMsg.pose.orientation.w = quaternion[3]
                landmarkMsg = Landmark()
                landmarkMsg.header = imageMsg.header
                landmarkMsg.id = ids[i]
                landmarkMsg.pose = poseMsg
                landmarkArrMsg.landmarks.append(landmarkMsg)

            # Publish pose
            self.posePub.publish(poseMsg)

        if self.showRejected:
            cvImage = self.arucoTracker.drawRejectedMarkers(cvImage, rejected)

        if self.visualize:
            cv2.imshow("ArUco Detection", cvImage)
            cv2.waitKey(1)

        # except Exception as e:
        #     rospy.logerr(f"Error processing image: {e}")

    def cameraInfoCallback(self, cameraInfoMsg: CameraInfo):
        """
        Updates camera calibration parameters from a ROS CameraInfo message.
        
        Extracts the camera matrix and distortion coefficients from the provided CameraInfo
        message and updates the node's internal parameters accordingly.
        
        Parameters
        ----------
        cameraInfoMsg : CameraInfo
            The ROS message containing calibration data.
        """
        self.cameraMatrix = np.array(cameraInfoMsg.K).reshape(3, 3)
        self.distCoeffs = np.array(cameraInfoMsg.D)

    def run(self):
        """
        Start the ROS event loop.
        
        This method calls rospy.spin() to keep the node active, continuously processing
        incoming messages and callbacks until the node is externally shut down.
        """
        rospy.spin()


if __name__ == "__main__":
    try:
        node = ArucoTrackingNode()
        node.run()
    except rospy.ROSInterruptException:
        pass
    finally:
        cv2.destroyAllWindows()
