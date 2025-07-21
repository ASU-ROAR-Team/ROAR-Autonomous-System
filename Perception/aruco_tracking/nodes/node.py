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
        Initialize the ArucoTrackingNode and load parameters.
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
        self.posePub = rospy.Publisher("/landmark_topic", Landmark, queue_size=10)

    def loadParameters(self):
        """
        Load parameters from the ROS parameter server.
        """
        self.cameraTopic = rospy.get_param("~cameraTopic", "/camera/color/image_raw")
        self.cameraInfoTopic = rospy.get_param("~cameraInfoTopic", "/camera/color/camera_info")
        self.arucoDictType = rospy.get_param("~arucoDictType", cv2.aruco.DICT_5X5_100)
        self.visualize = rospy.get_param("~visualize", True)
        self.showRejected = rospy.get_param("~showRejected", True)
        self.markerSize = rospy.get_param("~markerSize", 0.205) # 0.116

        # Validate mandatory parameters
        if self.cameraTopic is None:
            rospy.logerr("Camera topic not specified. Shutting down.")
            rospy.signal_shutdown("Camera topic not specified.")

    def imageCallback(self, imageMsg: Image):
        """
        Callback for processing incoming camera images.

        Parameters
        ----------
        imageMsg : Image
            The incoming ROS Image message.
        """
        # try:
        cvImage = self.bridge.imgmsg_to_cv2(imageMsg, "bgr8")
        corners, ids, rejected = self.arucoTracker.detectMarkers(cvImage)
        if ids is not None:
            # Estimate pose
            if self.cameraMatrix is None or self.distCoeffs is None:
                rospy.logwarn_throttle(1, "Camera calibration not available. Cannot estimate pose.")
                return
            else:
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

                # | NEW |

                # Instead of complex face detection or mapping
                # Move 0.105 m *opposite* to face normal in camera frame

                # Compute face normal in camera frame
                face_normal_camera = rotationMatrix @ np.array([0, 0, 1])  # local +Z → camera frame

                # Offset inward to cube centroid
                offset_world = -0.105 * face_normal_camera
                centroid = tvec + offset_world

                # === STEP 3: Use centroid for publishing ===
                tvec[0] = centroid[0]
                tvec[1] = centroid[1]
                tvec[2] = centroid[2]


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
                landmarkMsg.id = ids[i][0]
                landmarkMsg.pose = poseMsg
                self.posePub.publish(landmarkMsg)
                landmarkArrMsg.landmarks.append(landmarkMsg)

            # Publish pose
            print(ids)
            #self.posePub.publish(landmarkArrMsg)

        if self.showRejected:
            cvImage = self.arucoTracker.drawRejectedMarkers(cvImage, rejected)

        if self.visualize:
            cv2.imshow("ArUco Detection", cvImage)
            cv2.waitKey(1)

        # except Exception as e:
        #     rospy.logerr(f"Error processing image: {e}")

    def cameraInfoCallback(self, cameraInfoMsg: CameraInfo):
        """
        Callback for processing camera calibration information.

        Parameters
        ----------
        cameraInfoMsg : CameraInfo
            The incoming ROS CameraInfo message.
        """
        print("Camera info Updated Successfully!")
        self.cameraMatrix = np.array(cameraInfoMsg.K).reshape(3, 3)
        self.distCoeffs = np.array(cameraInfoMsg.D)

    def run(self):
        """
        Run the node.
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
