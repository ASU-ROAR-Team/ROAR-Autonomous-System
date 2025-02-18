import numpy as np
import cv2
import rospy
import tf
from sensor_msgs.msg import Image, CameraInfo
from std_msgs.msg import Header
from geometry_msgs.msg import PoseStamped
from cv_bridge import CvBridge
import cv2.aruco as aruco

# Initialize ROS node
rospy.init_node('aruco_pose_publisher', anonymous=True)

# ArUco dictionary
ARUCO_DICT = {
    "DICT_4X4_50": aruco.DICT_4X4_50,
    "DICT_5X5_100": aruco.DICT_5X5_100
}

aruco_type = "DICT_5X5_100"
aruco_dict = aruco.getPredefinedDictionary(ARUCO_DICT[aruco_type])
aruco_params = aruco.DetectorParameters()

bridge = CvBridge()

# Global variables for camera parameters
camera_matrix = None
distortion = None

# ROS publisher
pose_pub = rospy.Publisher("/landmark_topic", PoseStamped, queue_size=10)

def camera_info_callback(msg):
    """ Callback function to get camera intrinsics from /camera/color/camera_info """
    global camera_matrix, distortion
    camera_matrix = np.array(msg.K).reshape(3, 3)
    distortion = np.array(msg.D)

def image_callback(msg):
    """ Callback function to process frames from /camera/color/image_raw """
    global camera_matrix, distortion

    if camera_matrix is None or distortion is None:
        rospy.logwarn("Camera parameters not received yet!")
        return

    # Convert ROS Image message to OpenCV format
    frame = bridge.imgmsg_to_cv2(msg, desired_encoding="bgr8")
    pose_estimation(frame, aruco_dict, aruco_params, camera_matrix, distortion, msg.header)

    # Display the output image with detected ArUco markers
    cv2.imshow("ArUco Pose Estimation", frame)
    if cv2.waitKey(1) & 0xFF == ord('q'):
        rospy.signal_shutdown("User exited")

def pose_estimation(frame, aruco_dict, aruco_params, camera_matrix, distortion, header):
    """ Detects ArUco markers, estimates their pose, converts to PoseStamped, and publishes """
    gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
    corners, ids, _ = aruco.detectMarkers(gray, aruco_dict, parameters=aruco_params)

    if ids is not None:
        for i in range(len(ids)):
            # Estimate pose of the detected marker
            rvec, tvec, _ = aruco.estimatePoseSingleMarkers(corners[i], 0.05, camera_matrix, distortion)

            # Convert rotation vector to quaternion
            rotation_matrix, _ = cv2.Rodrigues(rvec)
            quaternion = tf.transformations.quaternion_from_matrix(np.vstack([np.hstack([rotation_matrix, [[0], [0], [0]]]), [0, 0, 0, 1]]))

            # Create and publish PoseStamped message
            pose_msg = PoseStamped()
            pose_msg.header = header
            pose_msg.pose.position.x = tvec[0][0][0]
            pose_msg.pose.position.y = tvec[0][0][1]
            pose_msg.pose.position.z = tvec[0][0][2]
            pose_msg.pose.orientation.x = quaternion[0]
            pose_msg.pose.orientation.y = quaternion[1]
            pose_msg.pose.orientation.z = quaternion[2]
            pose_msg.pose.orientation.w = quaternion[3]

            pose_pub.publish(pose_msg)

            # Draw detected markers and axis using cv2.drawFrameAxes
            aruco.drawDetectedMarkers(frame, corners)
            cv2.drawFrameAxes(frame, camera_matrix, distortion, rvec, tvec, 0.03)

            # Print debug info
            print(f"Marker {ids[i][0]} Pose:")
            print(f"  Position: x={tvec[0][0][0]}, y={tvec[0][0][1]}, z={tvec[0][0][2]}")
            print(f"  Quaternion: {quaternion}")

# Subscribe to camera topics
rospy.Subscriber("/camera/color/camera_info", CameraInfo, camera_info_callback)
rospy.Subscriber("/camera/color/image_raw", Image, image_callback)

rospy.spin()
cv2.destroyAllWindows()
