#!usr/bin/env python3
# pylint: disable=all
# mypy: ignore-errors
"""
Aruco tracking module
"""
from typing import Optional, Tuple, List
import numpy as np
import cv2

class ArucoTracker:
    """
    A class for detecting and tracking ArUco markers in images.

    Attributes
    ----------
    arucoDict : cv2.aruco_Dictionary
        The ArUco dictionary used for marker detection.
    arucoParams : cv2.aruco_DetectorParameters
        Parameters for the ArUco marker detector.
    arucoDetector : cv2.aruco.ArucoDetector
        The ArUco detector object.
    """

    def __init__(self, arucoDictType: int = cv2.aruco.DICT_5X5_100):
        """
        Initialize the ArucoTracker with the specified ArUco dictionary.

        Parameters
        ----------
        arucoDictType : int, optional
            The type of ArUco dictionary to use (default is DICT_5X5_100).
        """
        # This line is already correct for OpenCV 4.7.0+
        self.arucoDict = cv2.aruco.getPredefinedDictionary(arucoDictType)
        
        # These lines are also correct for OpenCV 4.7.0+
        self.arucoParams = cv2.aruco.DetectorParameters()
        self.arucoDetector = cv2.aruco.ArucoDetector(self.arucoDict, self.arucoParams)

    def detectMarkers(
        self, image: np.ndarray
    ) -> Tuple[List[np.ndarray], Optional[np.ndarray], List[np.ndarray]]:
        """
        Detect ArUco markers in the given image.

        Parameters
        ----------
        image : np.ndarray
            The input image in which to detect ArUco markers.

        Returns
        -------
        corners : List[np.ndarray]
            List of detected marker corners.
        ids : Optional[np.ndarray]
            Array of detected marker IDs (or None if no markers are detected).
        rejected : List[np.ndarray]
            List of rejected marker candidates.
        """
        # This method signature and call are compatible with OpenCV 4.8
        corners, ids, rejected = self.arucoDetector.detectMarkers(image)
        return corners, ids, rejected

    def estimatePose(
        self,
        corners: List[np.ndarray],
        markerSize: float,
        cameraMatrix: np.ndarray,
        distCoeffs: np.ndarray,
    ) -> Tuple[np.ndarray, np.ndarray]:
        """
        Estimate the pose of detected ArUco markers using solvePnP.
        """
        if not corners:
            return np.array([]), np.array([])

        rvecs = []
        tvecs = []

        # Define the 3D object points for the marker
        objPoints = np.array(
            [
                [-markerSize / 2, markerSize / 2, 0],
                [markerSize / 2, markerSize / 2, 0],
                [markerSize / 2, -markerSize / 2, 0],
                [-markerSize / 2, -markerSize / 2, 0],
            ],
            dtype=np.float32,
        )

        for corner in corners:
            try:
                # Reshape corner points for solvePnP
                imgPoints = corner.reshape(-1, 2).astype(np.float32)

                # Use IPPE_SQUARE method which is optimized for square markers
                # This flag is available and correctly used in OpenCV 4.8
                success, rvec, tvec = cv2.solvePnP(
                    objPoints, imgPoints, cameraMatrix, distCoeffs, flags=cv2.SOLVEPNP_IPPE_SQUARE
                )

                if success:
                    rvecs.append(rvec)
                    tvecs.append(tvec)

            except cv2.error as e:
                print(f"Error in pose estimation: {e}")
                continue

        return np.array(rvecs), np.array(tvecs)

    def drawMarkers(
        self,
        image: np.ndarray,
        corners: List[np.ndarray],
        ids: Optional[np.ndarray],
        rvecs: np.ndarray,
        tvecs: np.ndarray,
        cameraMatrix: np.ndarray,
        distCoeffs: np.ndarray,
    ) -> np.ndarray:
        """
        Draw detected ArUco markers, axes, and pose information on the image.

        Parameters
        ----------
        image : np.ndarray
            The input image on which to draw the markers.
        corners : List[np.ndarray]
            List of detected marker corners.
        ids : Optional[np.ndarray]
            Array of detected marker IDs.
        rvecs : np.ndarray
            Rotation vectors for each marker.
        tvecs : np.ndarray
            Translation vectors for each marker.
        cameraMatrix : np.ndarray
            The camera matrix (3x3).
        distCoeffs : np.ndarray
            The distortion coefficients.

        Returns
        -------
        np.ndarray
            The image with drawn markers, axes, and pose information.
        """
        if ids is not None:
            ids = ids.flatten()
            for (markerCorners, markerId, rvec, tvec) in zip(corners, ids, rvecs, tvecs):
                # Draw the bounding box
                markerCorners = markerCorners.reshape((4, 2))
                (topLeft, topRight, bottomRight, bottomLeft) = markerCorners

                # Convert corners to integers
                topLeft = tuple(map(int, topLeft))
                topRight = tuple(map(int, topRight))
                bottomRight = tuple(map(int, bottomRight))
                bottomLeft = tuple(map(int, bottomLeft))

                cv2.line(image, topLeft, topRight, (0, 255, 0), 2)
                cv2.line(image, topRight, bottomRight, (0, 255, 0), 2)
                cv2.line(image, bottomRight, bottomLeft, (0, 255, 0), 2)
                cv2.line(image, bottomLeft, topLeft, (0, 255, 0), 2)

                # Draw the marker center
                centerX = int((topLeft[0] + bottomRight[0]) / 2.0)
                centerY = int((topLeft[1] + bottomRight[1]) / 2.0)
                cv2.circle(image, (centerX, centerY), 4, (0, 0, 255), -1)

                # Display the marker ID
                cv2.putText(
                    image,
                    f"ID: {markerId}",
                    (topLeft[0], topLeft[1] - 10),
                    cv2.FONT_HERSHEY_SIMPLEX,
                    0.5,
                    (0, 255, 0),
                    2,
                )

                # Draw the axis (compatible with OpenCV 4.8)
                cv2.drawFrameAxes(image, cameraMatrix, distCoeffs, rvec, tvec, 0.01)
                
                # Flattening rvec and tvec for display is good practice
                tvec = tvec.flatten()
                rvec = rvec.flatten()

                # Display position and angles (compatible with OpenCV 4.8)
                cv2.putText(
                    image,
                    f"Position: ({tvec[0]:.2f}, {tvec[1]:.2f}, {tvec[2]:.2f})",
                    (topLeft[0], topLeft[1] - 30),
                    cv2.FONT_HERSHEY_SIMPLEX,
                    0.5,
                    (255, 0, 0),
                    2,
                )
                cv2.putText(
                    image,
                    f"Rotation: ({rvec[0]:.2f}, {rvec[1]:.2f}, {rvec[2]:.2f})",
                    (topLeft[0], topLeft[1] - 50),
                    cv2.FONT_HERSHEY_SIMPLEX,
                    0.5,
                    (255, 0, 0),
                    2,
                )
        return image

    def drawRejectedMarkers(self, image: np.ndarray, rejected: List[np.ndarray]) -> np.ndarray:
        """
        Draw rejected marker candidates on the image.

        Parameters
        ----------
        image : np.ndarray
            The input image on which to draw the rejected markers.
        rejected : List[np.ndarray]
            List of rejected marker candidates.

        Returns
        -------
        np.ndarray
            The image with drawn rejected markers.
        """
        for rejectedMarker in rejected:
            rejectedMarker = rejectedMarker.reshape((4, 2))
            for i in range(4):
                cv2.line(
                    image,
                    tuple(map(int, rejectedMarker[i])),
                    tuple(map(int, rejectedMarker[(i + 1) % 4])),
                    (0, 0, 255),
                    2,
                )
        return image